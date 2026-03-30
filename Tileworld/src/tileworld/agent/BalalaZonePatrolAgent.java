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
import tileworld.planners.AstarPathGenerator;
import tileworld.planners.TWPath;

/**
 * ZonePatrolAgent

“Zone” → The map is treated like an area (or zones) that need to be fully covered
“Patrol” → The agent systematically moves through the map, like a guard patrolling an area

Covers the entire map using a back-and-forth (lawnmower) pattern
Avoids random movement to reduce wasted time and missed areas

Fuel management

Monitors its fuel level continuously
Calculates distance to the fuel station
Returns to refuel before running out

Decision making

Remembers tiles and holes it has seen before
Assigns scores to each target (based on usefulness and distance)
Chooses the best target instead of reacting randomly

Movement

Uses A* pathfinding (AstarPathGenerator)
Finds efficient paths and avoids obstacles
 */
public class BalalaZonePatrolAgent extends TWAgent {

    // ---------------------------------------------------------------
    // Constants
    // ---------------------------------------------------------------
    private static final double FUEL_SAFETY_MARGIN = 25.0;
    private static final double FUEL_UNKNOWN_RATIO = 0.35;
    private static final int    CARRY_CAPACITY     = 3;
    private static final double W_RECENCY          = 0.35;
    private static final double W_PROXIMITY        = 0.65;
    private static final double HOLE_BIAS_MARGIN   = 8.0;

    // ---------------------------------------------------------------
    // Fields
    // ---------------------------------------------------------------
    private final String             name;
    private final AstarPathGenerator pathGenerator;
    private BalalaCustomTWAgentMemory customMemory;
    private final Phase1Strategy     phase1;

    private int fuelStationX = -1;
    private int fuelStationY = -1;

    // Boustrophedon sweep state
    private int     sweepCol;
    private int     sweepRow;
    private boolean sweepGoingDown;

    // ---------------------------------------------------------------
    // Constructor
    // ---------------------------------------------------------------
    public BalalaZonePatrolAgent(String name, int xpos, int ypos,
                                 TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.name = name;

        this.customMemory = new BalalaCustomTWAgentMemory(
                this, env.schedule,
                env.getxDimension(), env.getyDimension());
        this.memory = customMemory;

        this.pathGenerator = new AstarPathGenerator(
                env, this,
                env.getxDimension() * env.getyDimension());

        this.sweepCol       = 0;
        this.sweepRow       = 0;
        this.sweepGoingDown = true;
        this.phase1         = new Phase1Strategy(this);
    }

    // ---------------------------------------------------------------
    // communicate
    // ---------------------------------------------------------------
    @Override
    public void communicate() {
        phase1.communicate();
        if (!phase1.isComplete()) return;

        String payload = "POS:" + getX() + "," + getY()
                       + ";TILES:" + carriedTiles.size();
        Message msg = new Message(name, "*", payload);
        this.getEnvironment().receiveMessage(msg);
    }

    // ---------------------------------------------------------------
    // think
    // ---------------------------------------------------------------
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

        // 1. Fuel check
        if (needsRefuel(ax, ay)) {
            if (fuelStationX == -1) {
                // Station not yet found — head to map centre
                return new TWThought(TWAction.MOVE,
                        pathTo(getEnvironment().getxDimension() / 2,
                               getEnvironment().getyDimension() / 2));
            }
            if (ax == fuelStationX && ay == fuelStationY) {
                return new TWThought(TWAction.REFUEL, TWDirection.Z);
            }
            return new TWThought(TWAction.MOVE,
                    pathTo(fuelStationX, fuelStationY));
        }

        // 2. Opportunistic drop
        TWEntity onCell = (TWEntity) getEnvironment()
                .getObjectGrid().get(ax, ay);
        if (onCell instanceof TWHole && hasTile()) {
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }

        // 3. Opportunistic pickup
        if (onCell instanceof TWTile && carriedTiles.size() < CARRY_CAPACITY) {
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }

        // 4. Navigate to best memory target
        BalalaCustomTWAgentMemory.MemoryEntry target = selectTarget(ax, ay);
        if (target != null) {
            return new TWThought(TWAction.MOVE, pathTo(target.x, target.y));
        }

        // 5. Boustrophedon sweep
        return new TWThought(TWAction.MOVE, nextSweepDirection(ax, ay));
    }

    // ---------------------------------------------------------------
    // act
    // ---------------------------------------------------------------
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
                    if (tile != null) {
                        pickUpTile(tile);
                        customMemory.removeTile(getX(), getY());
                    }
                    break;

                case PUTDOWN:
                    TWHole hole = (TWHole) getEnvironment()
                            .getObjectGrid().get(getX(), getY());
                    if (hole != null) {
                        putTileInHole(hole);
                        customMemory.removeHole(getX(), getY());
                    }
                    break;

                case REFUEL:
                    refuel();
                    break;

                default:
                    break;
            }
        } catch (CellBlockedException e) {
            // A* replans next think() cycle
        }
    }

    // ---------------------------------------------------------------
    // Fuel management
    // ---------------------------------------------------------------
    private boolean needsRefuel(int ax, int ay) {
        if (fuelStationX == -1) {
            return fuelLevel <= Parameters.defaultFuelLevel * FUEL_UNKNOWN_RATIO;
        }
        double dist = manhattan(ax, ay, fuelStationX, fuelStationY);
        return fuelLevel <= dist + FUEL_SAFETY_MARGIN;
    }

    // ---------------------------------------------------------------
    // Target selection
    // ---------------------------------------------------------------
    private BalalaCustomTWAgentMemory.MemoryEntry selectTarget(int ax, int ay) {
        List<BalalaCustomTWAgentMemory.MemoryEntry> tiles = customMemory.getKnownTiles();
        List<BalalaCustomTWAgentMemory.MemoryEntry> holes = customMemory.getKnownHoles();

        reScore(tiles, ax, ay);
        reScore(holes, ax, ay);

        BalalaCustomTWAgentMemory.MemoryEntry bestTile = bestEntry(tiles);
        BalalaCustomTWAgentMemory.MemoryEntry bestHole = bestEntry(holes);

        int carried = carriedTiles.size();

        if (carried == CARRY_CAPACITY) return bestHole;

        if (carried == 2) {
            if (bestHole == null) return bestTile;
            if (bestTile == null) return bestHole;
            double dTile = manhattan(ax, ay, bestTile.x, bestTile.y);
            double dHole = manhattan(ax, ay, bestHole.x, bestHole.y);
            return (dTile + HOLE_BIAS_MARGIN < dHole) ? bestTile : bestHole;
        }

        if (carried == 1) {
            if (bestTile == null) return bestHole;
            if (bestHole == null) return bestTile;
            return (bestTile.utility >= bestHole.utility) ? bestTile : bestHole;
        }

        return bestTile; // carried == 0
    }

    private void reScore(List<BalalaCustomTWAgentMemory.MemoryEntry> list,
                         int ax, int ay) {
        double now  = getEnvironment().schedule.getTime();
        double maxD = getEnvironment().getxDimension()
                    + getEnvironment().getyDimension();
        for (int i = 0; i < list.size(); i++) {
            BalalaCustomTWAgentMemory.MemoryEntry e = list.get(i);
            double age       = now - e.timestamp;
            double recency   = 1.0 / (1.0 + age);
            double dist      = manhattan(ax, ay, e.x, e.y);
            double proximity = Math.max(0.0, 1.0 - dist / maxD);
            e.utility = W_RECENCY * recency + W_PROXIMITY * proximity;
        }
    }

    private BalalaCustomTWAgentMemory.MemoryEntry bestEntry(
            List<BalalaCustomTWAgentMemory.MemoryEntry> list) {
        BalalaCustomTWAgentMemory.MemoryEntry best = null;
        for (int i = 0; i < list.size(); i++) {
            BalalaCustomTWAgentMemory.MemoryEntry e = list.get(i);
            if (best == null || e.utility > best.utility) best = e;
        }
        return best;
    }

    // ---------------------------------------------------------------
    // Boustrophedon sweep
    // ---------------------------------------------------------------
    private TWDirection nextSweepDirection(int ax, int ay) {
        int maxX = getEnvironment().getxDimension() - 1;
        int maxY = getEnvironment().getyDimension() - 1;

        sweepCol = Math.max(0, Math.min(maxX, sweepCol));
        sweepRow = Math.max(0, Math.min(maxY, sweepRow));

        if (ax == sweepCol && ay == sweepRow) {
            advanceSweep(maxX, maxY);
        }

        TWDirection dir = pathTo(sweepCol, sweepRow);
        if (dir == TWDirection.Z) dir = randomUnblockedDirection();
        return dir;
    }

    private void advanceSweep(int maxX, int maxY) {
        if (sweepGoingDown) {
            sweepRow++;
            if (sweepRow > maxY) {
                sweepRow       = maxY;
                sweepCol++;
                sweepGoingDown = false;
                if (sweepCol > maxX) sweepCol = 0;
            }
        } else {
            sweepRow--;
            if (sweepRow < 0) {
                sweepRow       = 0;
                sweepCol++;
                sweepGoingDown = true;
                if (sweepCol > maxX) sweepCol = 0;
            }
        }
    }

    // ---------------------------------------------------------------
    // Pathfinding — uses real TWPath API: hasNext() / popNext()
    // ---------------------------------------------------------------
    private TWDirection pathTo(int tx, int ty) {
        if (getX() == tx && getY() == ty) return TWDirection.Z;
        try {
            TWPath path = pathGenerator.findPath(getX(), getY(), tx, ty);
            if (path != null && path.hasNext()) {
                TWDirection d = path.popNext().getDirection();
                if (d != null && d != TWDirection.Z) return d;
            }
        } catch (Exception e) {
            // path blocked or not found — fall through
        }
        return randomUnblockedDirection();
    }

    private TWDirection randomUnblockedDirection() {
        TWDirection[] dirs = {
            TWDirection.N, TWDirection.S,
            TWDirection.E, TWDirection.W
        };
        int start = getEnvironment().random.nextInt(4);
        for (int i = 0; i < 4; i++) {
            TWDirection d = dirs[(start + i) % 4];
            int nx = getX() + d.dx;
            int ny = getY() + d.dy;
            if (getEnvironment().isInBounds(nx, ny)
                    && !getEnvironment().isCellBlocked(nx, ny)) {
                return d;
            }
        }
        return TWDirection.Z;
    }

    // ---------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------
    private double manhattan(int x1, int y1, int x2, int y2) {
        return Math.abs(x1 - x2) + Math.abs(y1 - y2);
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

    @Override
    public String getName() { return name; }
}