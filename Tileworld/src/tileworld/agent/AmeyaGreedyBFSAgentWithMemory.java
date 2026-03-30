package tileworld.agent;

import java.util.List;
import tileworld.Parameters;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.AmeyaGreedyBFSPathGenerator;
import tileworld.planners.TWPath;
import tileworld.planners.TWPathStep;

public class AmeyaGreedyBFSAgentWithMemory extends TWAgent {

    private static final double FUEL_THRESHOLD_RATIO      = 0.20;
    private static final double FUEL_UNKNOWN_STATION_RATIO = 0.30;
    private static final int    CARRY_CAPACITY             = 3;

    private static final double CARRY_BIAS_DISTANCE = 5.0;

    private final String name;
    private final AmeyaGreedyBFSPathGenerator pathGenerator;
    private AmeyaCustomTWAgentMemory customMemory;

    private int fuelStationX = -1;
    private int fuelStationY = -1;

    private Phase1Strategy phase1;

    public AmeyaGreedyBFSAgentWithMemory(String name, int xpos, int ypos,
                                 TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.name = name;

        this.customMemory = new AmeyaCustomTWAgentMemory(
                this, env.schedule,
                env.getxDimension(), env.getyDimension());
        this.memory = customMemory;

        this.pathGenerator = new AmeyaGreedyBFSPathGenerator(env, this, env.getxDimension() * env.getyDimension());
        this.phase1 = new Phase1Strategy(this);
    }

    @Override
    public void communicate() {
        phase1.communicate();
        // No Phase 2 message passing yet; left empty intentionally.
    }

    @Override
    protected TWThought think() {
        if (!phase1.isComplete()) {
            TWThought t = phase1.think();
            if (t != null) return t;
            // Phase 1 just completed; fall through to Phase 2
        }
        // Sync fuel station from Phase 1 result (once)
        if (fuelStationX == -1 && phase1.getFuelStation() != null) {
            fuelStationX = phase1.getFuelStation().x;
            fuelStationY = phase1.getFuelStation().y;
        }
        return customThink();
    }

    private TWThought customThink() {
        locateFuelStation();

        int ax = getX();
        int ay = getY();

        double threshold = (fuelStationX == -1)
                ? Parameters.defaultFuelLevel * FUEL_UNKNOWN_STATION_RATIO
                : Parameters.defaultFuelLevel * FUEL_THRESHOLD_RATIO;

        if (fuelLevel <= threshold) {
            if (fuelStationX != -1) {
                if (ax == fuelStationX && ay == fuelStationY) {
                    return new TWThought(TWAction.REFUEL, TWDirection.Z);
                }
                return new TWThought(TWAction.MOVE,
                        getPathDirection(fuelStationX, fuelStationY));
            }
            return new TWThought(TWAction.MOVE,
                    getPathDirection(
                            getEnvironment().getxDimension() / 2,
                            getEnvironment().getyDimension() / 2));
        }

        TWEntity currentCell = (TWEntity) getEnvironment()
                .getObjectGrid().get(ax, ay);

        if (currentCell instanceof TWHole && hasTile()) {
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }
        if (currentCell instanceof TWTile
                && carriedTiles.size() < CARRY_CAPACITY) {
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }

        AmeyaCustomTWAgentMemory.MemoryEntry target = selectTarget();
        if (target != null) {
            return new TWThought(TWAction.MOVE,
                    getPathDirection(target.x, target.y));
        }

        return new TWThought(TWAction.MOVE, getExploreDirection());
    }

    @Override
    protected void act(TWThought thought) {
        try {
            switch (thought.getAction()) {

                case MOVE:
                    move(thought.getDirection());
                    break;

                case PICKUP:
                    TWTile tile = (TWTile) getEnvironment()
                            .getObjectGrid().get(getX(), getY());
                    if (tile != null) pickUpTile(tile);
                    break;

                case PUTDOWN:
                    TWHole hole = (TWHole) getEnvironment()
                            .getObjectGrid().get(getX(), getY());
                    if (hole != null) putTileInHole(hole);
                    break;

                case REFUEL:
                    refuel();
                    break;
            }
        } catch (CellBlockedException e) {
            // GBFS replans from scratch on next think() cycle
        }
    }


    private void locateFuelStation() {
        if (fuelStationX != -1) return;
        for (int x = 0; x < getEnvironment().getxDimension(); x++) {
            for (int y = 0; y < getEnvironment().getyDimension(); y++) {
                if (getEnvironment().getObjectGrid()
                        .get(x, y) instanceof TWFuelStation) {
                    fuelStationX = x;
                    fuelStationY = y;
                    return;
                }
            }
        }
    }

    private AmeyaCustomTWAgentMemory.MemoryEntry selectTarget() {

        List<AmeyaCustomTWAgentMemory.MemoryEntry> tiles = customMemory.getKnownTiles();
        List<AmeyaCustomTWAgentMemory.MemoryEntry> holes = customMemory.getKnownHoles();

        AmeyaCustomTWAgentMemory.MemoryEntry bestTile = bestEntry(tiles);
        AmeyaCustomTWAgentMemory.MemoryEntry bestHole = bestEntry(holes);

        int carried = carriedTiles.size();

        if (carried == CARRY_CAPACITY) {
            return bestHole;
        }

        if (carried == 2) {
            if (bestHole == null) return bestTile;
            if (bestTile == null) return bestHole;

            double distTile = manhattan(getX(), getY(), bestTile.x, bestTile.y);
            double distHole = manhattan(getX(), getY(), bestHole.x, bestHole.y);

            if (distTile + CARRY_BIAS_DISTANCE < distHole) return bestTile;
            return bestHole;
        }

        if (carried == 1) {
            if (bestTile == null) return bestHole;
            if (bestHole == null) return bestTile;
            return bestTile.utility >= bestHole.utility ? bestTile : bestHole;
        }

        return bestTile;
    }

    private AmeyaCustomTWAgentMemory.MemoryEntry bestEntry(
            List<AmeyaCustomTWAgentMemory.MemoryEntry> entries) {
        AmeyaCustomTWAgentMemory.MemoryEntry best = null;
        for (AmeyaCustomTWAgentMemory.MemoryEntry e : entries) {
            if (best == null || e.utility > best.utility) {
                best = e;
            }
        }
        return best;
    }

    private double manhattan(int x1, int y1, int x2, int y2) {
        return Math.abs(x1 - x2) + Math.abs(y1 - y2);
    }

    private TWDirection getPathDirection(int tx, int ty) {
        TWPath path = pathGenerator.findPath(getX(), getY(), tx, ty);
        if (path != null && path.hasNext()) {
            TWPathStep step = path.popNext();
            if (step.getDirection() != TWDirection.Z) {
                return step.getDirection();
            }
        }
        return getExploreDirection();
    }

    private TWDirection getExploreDirection() {
        int ax = getX();
        int ay = getY();
        int maxX = getEnvironment().getxDimension() - 1;
        int maxY = getEnvironment().getyDimension() - 1;

        if (ax >= maxX) return TWDirection.W;
        if (ax <= 0)    return TWDirection.E;
        if (ay <= 0)    return TWDirection.S;
        if (ay >= maxY) return TWDirection.N;

        TWDirection[] dirs = {
            TWDirection.N, TWDirection.S,
            TWDirection.E, TWDirection.W
        };
        int start = getEnvironment().random.nextInt(4);
        for (int i = 0; i < 4; i++) {
            TWDirection d = dirs[(start + i) % 4];
            if (!getEnvironment().isCellBlocked(getX() + d.dx, getY() + d.dy)) {
                return d;
            }
        }
        return TWDirection.Z;
    }

    @Override
    public String getName() {
        return name;
    }
}