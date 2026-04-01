package tileworld.agent;

import tileworld.environment.TWEnvironment;
import sim.util.Int2D;

/**
 * The skeleton class for all team agents.
 * Encapsulates Phase 1 exploration, base communication, and shared state.
 */
public abstract class ArdaTWAgentSkeleton extends TWAgent {

    // --- Shared Constants ---
    protected static final String ENTITY_TILE = "tile";
    protected static final String ENTITY_HOLE = "hole";
    protected static final String ENTITY_FUEL = "fuel";
    protected static final int CARRY_CAPACITY = 3;

    // --- Shared State ---
    protected final String agentName;
    protected final Phase1Strategy phase1;
    
    // Globally tracked fuel station coordinates
    protected int fuelStationX = -1;
    protected int fuelStationY = -1;

    public ArdaTWAgentSkeleton(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
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
            customCommunicate();
        }
    }

    /**
     * Reads all incoming messages. Automatically extracts fuel station data.
     * Delegates other message types to the subclass.
     */
    protected final void processSharedMessages() {
        for (Message raw : getEnvironment().getMessages()) {
            // Ignore our own messages
            if (agentName.equals(raw.getFrom())) continue;
            if (!(raw instanceof ArdaMessage)) continue;

            ArdaMessage msg = (ArdaMessage) raw;

            // Universally track the fuel station if anyone broadcasts it
            if (ENTITY_FUEL.equals(msg.getEntityType()) && fuelStationX == -1) {
                fuelStationX = msg.getX();
                fuelStationY = msg.getY();
            }

            // Let the specific subclass handle tiles, holes, intents, or positions
            handleTeamMessage(msg);
        }
    }

    /**
     * Helper to broadcast standard ArdaMessages
     */
    protected void broadcastInfo(String entityType, int targetX, int targetY) {
        getEnvironment().receiveMessage(
            ArdaMessage.info(agentName, entityType, targetX, targetY, getX(), getY())
        );
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