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
import tileworld.planners.GreedyBFSPathGenerator;
import tileworld.planners.TWPath;
import tileworld.planners.TWPathStep;

public class GreedyBFSAgentWithMemoryMessage extends TWAgent {

    private static final String ENTITY_TILE        = "tile";
    private static final String ENTITY_HOLE        = "hole";
    private static final String ENTITY_FUEL        = "fuel";
    private static final String ENTITY_DELETE_TILE = "delete_tile";
    private static final String ENTITY_DELETE_HOLE = "delete_hole";

    private static final double FUEL_THRESHOLD_RATIO       = 0.20;
    private static final double FUEL_UNKNOWN_STATION_RATIO = 0.30;
    private static final int    CARRY_CAPACITY             = 3;
    private static final double CARRY_BIAS_DISTANCE        = 5.0;

    private final String name;
    private final GreedyBFSPathGenerator pathGenerator;
    private final CustomTWAgentMemory customMemory;

    private int fuelStationX = -1;
    private int fuelStationY = -1;

    private int[] pendingDeleteTile = null; // confirmed pickup last act()
    private int[] pendingDeleteHole = null; // confirmed putdown last act()
    private int[] pendingInfoTile   = null; // saw tile but couldn't pick up
    private int[] pendingInfoHole   = null; // saw hole but couldn't fill

    public GreedyBFSAgentWithMemoryMessage(String name, int xpos, int ypos,
                                            TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.name = name;

        this.customMemory = new CustomTWAgentMemory(
                this, env.schedule,
                env.getxDimension(), env.getyDimension());
        this.memory = customMemory;

        this.pathGenerator = new GreedyBFSPathGenerator(
                env, this, env.getxDimension() * env.getyDimension());
    }

    @Override
    public void communicate() {

        // --- STEP 1: Process incoming ArdaMessages from other agents ---
        for (Message raw : getEnvironment().getMessages()) {
            // Skip own messages
            if (getName().equals(raw.getFrom())) continue;
            // Only process ArdaMessage instances
            if (!(raw instanceof ArdaMessage)) continue;

            ArdaMessage msg = (ArdaMessage) raw;

            // We only handle INFO type - INTENTION is used by other team members
            if (msg.getType() != ArdaMessage.MessageType.INFO) continue;

            String entity = msg.getEntityType();
            int mx = msg.getX();
            int my = msg.getY();

            if (ENTITY_TILE.equals(entity)) {
                // Another agent saw a tile it couldn't pick up - add to memory
                addTileToMemory(mx, my);

            } else if (ENTITY_HOLE.equals(entity)) {
                // Another agent saw a hole it couldn't fill - add to memory
                addHoleToMemory(mx, my);

            } else if (ENTITY_FUEL.equals(entity)) {
                // Another agent found the fuel station - store it
                if (fuelStationX == -1) {
                    fuelStationX = mx;
                    fuelStationY = my;
                }

            } else if (ENTITY_DELETE_TILE.equals(entity)) {
                // Another agent picked up a tile - remove from memory
                customMemory.removeTrackedTile(mx, my);
                customMemory.removeAgentPercept(mx, my);

            } else if (ENTITY_DELETE_HOLE.equals(entity)) {
                // Another agent filled a hole - remove from memory
                customMemory.removeTrackedHole(mx, my);
                customMemory.removeAgentPercept(mx, my);
            }
        }

        // --- STEP 2: Broadcast confirmed DELETEs from previous act() ---
        if (pendingDeleteTile != null) {
            broadcast(ArdaMessage.info(
                    getName(),
                    ENTITY_DELETE_TILE,
                    pendingDeleteTile[0], pendingDeleteTile[1],
                    getX(), getY()));
            // Apply to own memory too
            customMemory.removeTrackedTile(pendingDeleteTile[0], pendingDeleteTile[1]);
            customMemory.removeAgentPercept(pendingDeleteTile[0], pendingDeleteTile[1]);
            pendingDeleteTile = null;
        }

        if (pendingDeleteHole != null) {
            broadcast(ArdaMessage.info(
                    getName(),
                    ENTITY_DELETE_HOLE,
                    pendingDeleteHole[0], pendingDeleteHole[1],
                    getX(), getY()));
            // Apply to own memory too
            customMemory.removeTrackedHole(pendingDeleteHole[0], pendingDeleteHole[1]);
            customMemory.removeAgentPercept(pendingDeleteHole[0], pendingDeleteHole[1]);
            pendingDeleteHole = null;
        }

        // --- STEP 3: Broadcast INFO messages flagged in previous think() ---
        if (pendingInfoTile != null) {
            broadcast(ArdaMessage.info(
                    getName(),
                    ENTITY_TILE,
                    pendingInfoTile[0], pendingInfoTile[1],
                    getX(), getY()));
            // Add to own memory too
            addTileToMemory(pendingInfoTile[0], pendingInfoTile[1]);
            pendingInfoTile = null;
        }

        if (pendingInfoHole != null) {
            broadcast(ArdaMessage.info(
                    getName(),
                    ENTITY_HOLE,
                    pendingInfoHole[0], pendingInfoHole[1],
                    getX(), getY()));
            // Add to own memory too
            addHoleToMemory(pendingInfoHole[0], pendingInfoHole[1]);
            pendingInfoHole = null;
        }

        // --- STEP 4: Broadcast fuel station whenever known ---
        if (fuelStationX != -1) {
            broadcast(ArdaMessage.info(
                    getName(),
                    ENTITY_FUEL,
                    fuelStationX, fuelStationY,
                    getX(), getY()));
        }
    }

    @Override
    protected TWThought think() {

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

        if (currentCell instanceof TWHole) {
            if (hasTile()) {
                return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
            } else {
                pendingInfoHole = new int[]{ax, ay};
            }
        }

        if (currentCell instanceof TWTile) {
            if (carriedTiles.size() < CARRY_CAPACITY) {
                return new TWThought(TWAction.PICKUP, TWDirection.Z);
            } else {
                pendingInfoTile = new int[]{ax, ay};
            }
        }

        CustomTWAgentMemory.MemoryEntry target = selectTarget();
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
                    if (tile != null) {
                        int tx = tile.getX();
                        int ty = tile.getY();
                        pickUpTile(tile);
                        pendingDeleteTile = new int[]{tx, ty};
                    }
                    break;

                case PUTDOWN:
                    TWHole hole = (TWHole) getEnvironment()
                            .getObjectGrid().get(getX(), getY());
                    if (hole != null) {
                        int hx = hole.getX();
                        int hy = hole.getY();
                        putTileInHole(hole);
                        pendingDeleteHole = new int[]{hx, hy};
                    }
                    break;

                case REFUEL:
                    refuel();
                    break;
            }
        } catch (CellBlockedException e) {
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


    private CustomTWAgentMemory.MemoryEntry selectTarget() {
        List<CustomTWAgentMemory.MemoryEntry> tiles = customMemory.getKnownTiles();
        List<CustomTWAgentMemory.MemoryEntry> holes = customMemory.getKnownHoles();

        CustomTWAgentMemory.MemoryEntry bestTile = bestEntry(tiles);
        CustomTWAgentMemory.MemoryEntry bestHole = bestEntry(holes);

        int carried = carriedTiles.size();

        if (carried == CARRY_CAPACITY) return bestHole;

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

    private CustomTWAgentMemory.MemoryEntry bestEntry(
            List<CustomTWAgentMemory.MemoryEntry> entries) {
        CustomTWAgentMemory.MemoryEntry best = null;
        for (CustomTWAgentMemory.MemoryEntry e : entries) {
            if (best == null || e.utility > best.utility) best = e;
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

    /** Sends a message to the environment broadcast channel. */
    private void broadcast(ArdaMessage msg) {
        getEnvironment().receiveMessage(msg);
    }

    /**
     * Adds a tile to memory only if it still exists on the live grid
     * and is not already tracked.
     */
    private void addTileToMemory(int x, int y) {
        Object obj = getEnvironment().getObjectGrid().get(x, y);
        if (obj instanceof TWTile) {
            customMemory.addTrackedTile(x, y, getEnvironment().schedule.getTime());
        }
    }

    /**
     * Adds a hole to memory only if it still exists on the live grid
     * and is not already tracked.
     */
    private void addHoleToMemory(int x, int y) {
        Object obj = getEnvironment().getObjectGrid().get(x, y);
        if (obj instanceof TWHole) {
            customMemory.addTrackedHole(x, y, getEnvironment().schedule.getTime());
        }
    }

    @Override
    public String getName() {
        return name;
    }
}