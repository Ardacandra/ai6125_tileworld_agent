package tileworld.agent;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import tileworld.Parameters;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import sim.util.Int2D;

/**
 * The skeleton class for all team agents.
 * Encapsulates Phase 1 exploration, base communication, and shared state.
 */
public abstract class TWAgentSkeleton extends TWAgent {

    // --- Shared Constants ---
    protected static final String ENTITY_TILE = "tile";
    protected static final String ENTITY_HOLE = "hole";
    protected static final String ENTITY_FUEL = "fuel";
    protected static final String ENTITY_DELETE_TILE = "delete_tile";
    protected static final String ENTITY_DELETE_HOLE = "delete_hole";

    protected static final int CARRY_CAPACITY = 3;

    // --- Shared State ---
    protected final String agentName;
    protected final Phase1Strategy phase1;

    private final Map<String, SharedLocation> knownSharedTiles = new HashMap<>();
    private final Map<String, SharedLocation> knownSharedHoles = new HashMap<>();
    private final List<IntentInfo> receivedIntentions = new ArrayList<>();

    private String intendedType = "";
    private int intendedX = -1;
    private int intendedY = -1;
    
    // Globally tracked fuel station coordinates
    protected int fuelStationX = -1;
    protected int fuelStationY = -1;

    private static final class SharedLocation {
        private final int x;
        private final int y;

        private SharedLocation(int x, int y) {
            this.x = x;
            this.y = y;
        }
    }

    private static final class IntentInfo {
        private final String agentName;
        private final String entityType;
        private final int targetX;
        private final int targetY;
        private final int senderX;
        private final int senderY;

        private IntentInfo(String agentName, String entityType, int targetX, int targetY, int senderX, int senderY) {
            this.agentName = agentName;
            this.entityType = entityType;
            this.targetX = targetX;
            this.targetY = targetY;
            this.senderX = senderX;
            this.senderY = senderY;
        }
    }

    public TWAgentSkeleton(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.agentName = name;
        this.phase1 = new Phase1Strategy(this);
    }

    // ---------------------------------------------------------
    // COMMUNICATION PIPELINE
    // ---------------------------------------------------------
    @Override
    public final void communicate() {
        // 1. Always let Phase 1 handle initial coordination and fuel broadcasting
        phase1.communicate();

        // 2. Once Phase 1 is complete, delegate to the specific agent's Phase 2 communication
        if (phase1.isComplete()) {
            broadcastVisibleEntityInfo();
            broadcastCurrentIntention();
            customCommunicate();
        }
    }

    /**
     * Reads all incoming messages. Automatically extracts fuel station data.
     * Delegates other message types to the subclass.
     */
    protected final void processSharedMessages() {
        receivedIntentions.clear();

        for (Message raw : getEnvironment().getMessages()) {
            // Ignore our own messages
            if (agentName.equals(raw.getFrom())) continue;
            if (!(raw instanceof ArdaMessage)) continue;

            ArdaMessage msg = (ArdaMessage) raw;

            if (msg.getType() == ArdaMessage.MessageType.INFO) {
                ingestSharedInfo(msg);
            } else if (msg.getType() == ArdaMessage.MessageType.INTENTION) {
                receivedIntentions.add(new IntentInfo(
                        msg.getFrom(),
                        msg.getEntityType(),
                        msg.getX(),
                        msg.getY(),
                        msg.getSenderX(),
                        msg.getSenderY()));
            }

            // Let the specific subclass handle tiles, holes, intents, or positions
            handleTeamMessage(msg);
        }

        pruneSharedKnowledge();
    }

    /**
     * Helper to broadcast standard ArdaMessages
     */
    protected void broadcastInfo(String entityType, int targetX, int targetY) {
        getEnvironment().receiveMessage(
            ArdaMessage.info(agentName, entityType, targetX, targetY, getX(), getY())
        );
    }

    protected final void setIntention(String entityType, int targetX, int targetY) {
        intendedType = entityType;
        intendedX = targetX;
        intendedY = targetY;
    }

    protected final void clearIntention() {
        intendedType = "";
        intendedX = -1;
        intendedY = -1;
    }

    protected final List<int[]> getSharedTileLocations() {
        return toCoordinateList(knownSharedTiles);
    }

    protected final List<int[]> getSharedHoleLocations() {
        return toCoordinateList(knownSharedHoles);
    }

    protected final void forgetSharedTile(int x, int y) {
        knownSharedTiles.remove(asKey(x, y));
    }

    protected final void forgetSharedHole(int x, int y) {
        knownSharedHoles.remove(asKey(x, y));
    }

    protected final boolean isClaimedByCloserAgent(String entityType, int targetX, int targetY) {
        double myDistance = manhattan(getX(), getY(), targetX, targetY);

        for (IntentInfo intent : receivedIntentions) {
            if (!entityType.equals(intent.entityType)) {
                continue;
            }
            if (intent.targetX != targetX || intent.targetY != targetY) {
                continue;
            }

            double otherDistance = manhattan(intent.senderX, intent.senderY, targetX, targetY);
            if (otherDistance < myDistance) {
                return true;
            }
            if (otherDistance == myDistance && intent.agentName.compareTo(agentName) < 0) {
                return true;
            }
        }

        return false;
    }

    private void broadcastCurrentIntention() {
        if ("".equals(intendedType) || intendedX < 0 || intendedY < 0) {
            return;
        }
        getEnvironment().receiveMessage(
                ArdaMessage.intention(agentName, intendedType, intendedX, intendedY, getX(), getY()));
    }

    private void ingestSharedInfo(ArdaMessage msg) {
        if (ENTITY_TILE.equals(msg.getEntityType())) {
            knownSharedTiles.put(asKey(msg.getX(), msg.getY()), new SharedLocation(msg.getX(), msg.getY()));
            return;
        }

        if (ENTITY_HOLE.equals(msg.getEntityType())) {
            knownSharedHoles.put(asKey(msg.getX(), msg.getY()), new SharedLocation(msg.getX(), msg.getY()));
            return;
        }

        // Universally track fuel station broadcasts.
        if (ENTITY_FUEL.equals(msg.getEntityType()) && fuelStationX == -1) {
            fuelStationX = msg.getX();
            fuelStationY = msg.getY();
        }
    }

    private void broadcastVisibleEntityInfo() {
        int sensorRange = Parameters.defaultSensorRange;
        int minX = Math.max(0, getX() - sensorRange);
        int maxX = Math.min(getEnvironment().getxDimension() - 1, getX() + sensorRange);
        int minY = Math.max(0, getY() - sensorRange);
        int maxY = Math.min(getEnvironment().getyDimension() - 1, getY() + sensorRange);

        for (int cellX = minX; cellX <= maxX; cellX++) {
            for (int cellY = minY; cellY <= maxY; cellY++) {
                TWEntity entity = (TWEntity) getEnvironment().getObjectGrid().get(cellX, cellY);
                if (entity instanceof TWTile) {
                    broadcastInfo(ENTITY_TILE, cellX, cellY);
                } else if (entity instanceof TWHole) {
                    broadcastInfo(ENTITY_HOLE, cellX, cellY);
                } else if (entity instanceof TWFuelStation) {
                    broadcastInfo(ENTITY_FUEL, cellX, cellY);
                }
            }
        }
    }

    private void pruneSharedKnowledge() {
        pruneByType(knownSharedTiles, ENTITY_TILE);
        pruneByType(knownSharedHoles, ENTITY_HOLE);
    }

    private void pruneByType(Map<String, SharedLocation> map, String type) {
        List<String> toRemove = new ArrayList<>();
        for (Map.Entry<String, SharedLocation> entry : map.entrySet()) {
            SharedLocation location = entry.getValue();
            if (!isPossiblyValidEntityAt(type, location.x, location.y)) {
                toRemove.add(entry.getKey());
            }
        }

        for (String key : toRemove) {
            map.remove(key);
        }
    }

    private boolean isPossiblyValidEntityAt(String type, int x, int y) {
        if (!isWithinSensorRange(x, y)) {
            return true;
        }

        Object obj = getEnvironment().getObjectGrid().get(x, y);
        if (ENTITY_TILE.equals(type)) {
            return obj instanceof TWTile;
        }
        if (ENTITY_HOLE.equals(type)) {
            return obj instanceof TWHole;
        }
        return true;
    }

    private boolean isWithinSensorRange(int x, int y) {
        int range = Parameters.defaultSensorRange;
        return Math.abs(x - getX()) <= range && Math.abs(y - getY()) <= range;
    }

    private List<int[]> toCoordinateList(Map<String, SharedLocation> source) {
        List<int[]> locations = new ArrayList<>();
        for (SharedLocation location : source.values()) {
            locations.add(new int[] { location.x, location.y });
        }
        return locations;
    }

    private String asKey(int x, int y) {
        return x + ":" + y;
    }

    // ---------------------------------------------------------
    // THINKING PIPELINE
    // ---------------------------------------------------------
    @Override
    protected final TWThought think() {
        // 1. Process all incoming network messages first
        processSharedMessages();

        // 2. Execute Phase 1 Strategy if not complete
        if (!phase1.isComplete()) {
            TWThought p1Thought = phase1.think();
            if (p1Thought != null) {
                return p1Thought; // Still exploring
            }
        }

        // 3. Phase 1 has finished. Sync the fuel station locally if Phase 1 found it.
        if (fuelStationX == -1 && phase1.getFuelStation() != null) {
            Int2D fs = phase1.getFuelStation();
            fuelStationX = fs.x;
            fuelStationY = fs.y;
        }

        // 4. Delegate to the subclass for Phase 2 target selection and movement
        return customThink();
    }

    @Override
    public String getName() {
        return agentName;
    }

    // ---------------------------------------------------------
    // ABSTRACT HOOKS FOR SUBCLASSES
    // ---------------------------------------------------------
    
    /** Implemented by subclass to handle Phase 2 communication */
    protected abstract void customCommunicate();

    /** Implemented by subclass to handle custom message types (tiles, holes, intents) */
    protected abstract void handleTeamMessage(ArdaMessage msg);

    /** Implemented by subclass to handle Phase 2 decision making */
    protected abstract TWThought customThink();

    /** Implemented by subclass to handle physical actions and specific path replanning */
    @Override
    protected abstract void act(TWThought thought);
    
    // --- Helper Methods ---
    protected double manhattan(int x1, int y1, int x2, int y2) {
        return Math.abs(x1 - x2) + Math.abs(y1 - y2);
    }
}