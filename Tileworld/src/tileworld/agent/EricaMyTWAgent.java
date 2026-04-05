package tileworld.agent;

import tileworld.Parameters;
import tileworld.environment.*;
import sim.util.Int2D;
import tileworld.planners.AstarPathGenerator;
import tileworld.planners.TWPath;
import tileworld.exceptions.CellBlockedException;
import java.util.*;

public class EricaMyTWAgent extends TWAgentSkeleton {

    private static final double FUEL_THRESHOLD_RATIO = 0.28;
    private static final int CARRY_CAPACITY = 3;
    private static final int MEMORY_LIFESPAN = 100; 

    private final AstarPathGenerator astar;
    private MyCustomMemory myMemory; // Integrated Custom Memory
    
    private final int[][] corners;
    private int cornerIdx = 0;
    private final boolean isLargeMap;

    public EricaMyTWAgent(String name, int x, int y, TWEnvironment env, double fuel) {
        super(name, x, y, env, fuel);
        this.astar = new AstarPathGenerator(env, this, 2000); 
        this.isLargeMap = (env.getxDimension() >= 60 || env.getyDimension() >= 60);
        
        // Initialize our custom memory with the lifespan defined above
        this.myMemory = new MyCustomMemory(this, env.schedule, env.getxDimension(), env.getyDimension(), MEMORY_LIFESPAN);
        
        int maxX = env.getxDimension() - 1;
        int maxY = env.getyDimension() - 1;
        this.corners = new int[][]{{0, 0}, {maxX, 0}, {maxX, maxY}, {0, maxY}};
    }

    @Override
    protected void customCommunicate() {
        int r = Parameters.defaultSensorRange;
        for (int dx = -r; dx <= r; dx++) {
            for (int dy = -r; dy <= r; dy++) {
                int cx = getX() + dx, cy = getY() + dy;
                if (!getEnvironment().isValidLocation(cx, cy)) continue;
                
                Object o = getEnvironment().getObjectGrid().get(cx, cy);
                if (o instanceof TWFuelStation) {
                    this.fuelStationX = cx;
                    this.fuelStationY = cy;
                    broadcast(ArdaMessage.info(agentName, "fuel", cx, cy, getX(), getY()));
                } else if (o instanceof TWTile || o instanceof TWHole) {
                    String type = (o instanceof TWTile) ? "tile" : "hole";
                    // Inform teammates about what we see
                    broadcast(ArdaMessage.info(agentName, type, cx, cy, getX(), getY()));
                }
            }
        }
    }

    @Override
    protected void handleTeamMessage(ArdaMessage am) {
        // If a teammate picked it up, purge it from our local memory immediately
        if (am.getEntityType().startsWith("delete")) {
            myMemory.clearLocation(am.getX(), am.getY());
        } else if (am.getEntityType().equals("fuel")) {
            this.fuelStationX = am.getX(); 
            this.fuelStationY = am.getY();
        }
    }

    @Override
    protected TWThought customThink() {
        // 1. ACTIVE CLEANUP: Sync memory with current sensors
        updateLocalTimestamps();
        
        double fuelLevel = getFuelLevel();
        double fuelRatio = fuelLevel / Parameters.defaultFuelLevel;

        // 2. REFUEL LOGIC
        if (fuelStationX != -1) {
            double dist = manhattan(getX(), getY(), fuelStationX, fuelStationY);
            double safetyPadding = isLargeMap ? 15 : 5;
            if (fuelLevel < (dist * 2.0 + safetyPadding) || fuelRatio < FUEL_THRESHOLD_RATIO) {
                if (getX() == fuelStationX && getY() == fuelStationY) 
                    return new TWThought(TWAction.REFUEL, TWDirection.Z);
                return navigate(fuelStationX, fuelStationY);
            }
        }

        // 3. IMMEDIATE ACTIONS (Pick up/Put down if standing on target)
        Object here = getEnvironment().getObjectGrid().get(getX(), getY());
        if (here instanceof TWTile && carriedTiles.size() < CARRY_CAPACITY) 
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        if (here instanceof TWHole && !carriedTiles.isEmpty()) 
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);

        // 4. TARGETING (Using Freshness Factor)
        Int2D target = findBestTarget();
        if (target != null) return navigate(target.x, target.y);

        // 5. PATROL (If nothing to do)
        return patrol();
    }

    private Int2D findBestTarget() {
        Int2D best = null;
        double maxUtility = -1.0;
        double now = getEnvironment().schedule.getTime();
        int searchRange = isLargeMap ? 30 : 100; 

        for (int x = 0; x < getEnvironment().getxDimension(); x++) {
            for (int y = 0; y < getEnvironment().getyDimension(); y++) {
                double dist = manhattan(getX(), getY(), x, y);
                if (isLargeMap && dist > searchRange) continue;

                // Check our memory grid
                Object o = myMemory.getMemoryGrid().get(x, y);
                if (o == null) continue;

                // Calculate Freshness from our internal HashMap in MyCustomMemory
                double freshness = myMemory.getFreshness(x, y, now);
                if (freshness <= 0) continue;

                boolean isValidType = (o instanceof TWTile && carriedTiles.size() < CARRY_CAPACITY) ||
                                     (o instanceof TWHole && !carriedTiles.isEmpty());
                
                if (isValidType) {
                    // Utility Formula: Inverse Distance weighted by Information Freshness
                    double utility = (100.0 / (dist + 1.0)) * freshness;
                    
                    // Priority Bias: Scoring (Holes) is worth more than Collecting (Tiles)
                    if (o instanceof TWHole) utility *= 2.5; 

                    if (utility > maxUtility) {
                        maxUtility = utility;
                        best = new Int2D(x, y);
                    }
                }
            }
        }
        return best;
    }

    /**
     * Scans the current sensor range to perform Active Cleanup.
     * If a cell is empty in reality but has an object in memory, it is purged.
     */
    private void updateLocalTimestamps() {
        int r = Parameters.defaultSensorRange;
        for (int dx = -r; dx <= r; dx++) {
            for (int dy = -r; dy <= r; dy++) {
                int cx = getX() + dx, cy = getY() + dy;
                if (getEnvironment().isValidLocation(cx, cy)) {
                    // If the sensor sees nothing, but memory thinks there's something...
                    if (getEnvironment().getObjectGrid().get(cx, cy) == null) {
                        myMemory.clearLocation(cx, cy); // Purge ghost
                    }
                }
            }
        }
    }

    @Override
    protected void act(TWThought thought) {
        try {
            int ax = getX(), ay = getY();
            switch (thought.getAction()) {
                case MOVE: move(thought.getDirection()); break;
                case REFUEL: refuel(); break;
                case PICKUP:
                    pickUpTile((TWTile) getEnvironment().getObjectGrid().get(ax, ay));
                    myMemory.clearLocation(ax, ay); // Update local memory immediately
                    broadcast(ArdaMessage.info(agentName, "delete_tile", ax, ay, ax, ay));
                    break;
                case PUTDOWN:
                    putTileInHole((TWHole) getEnvironment().getObjectGrid().get(ax, ay));
                    myMemory.clearLocation(ax, ay); // Update local memory immediately
                    broadcast(ArdaMessage.info(agentName, "delete_hole", ax, ay, ax, ay));
                    break;
            }
        } catch (CellBlockedException e) {
            // Path is blocked, agent will recalculate next tick
        }
    }

    private void broadcast(ArdaMessage msg) {
        getEnvironment().receiveMessage(msg);
    }

    private TWThought navigate(int tx, int ty) {
        TWPath path = astar.findPath(getX(), getY(), tx, ty);
        if (path != null && path.hasNext()) return new TWThought(TWAction.MOVE, path.popNext().getDirection());
        
        // Simple Manhattan fallback if A* fails
        int dx = Integer.compare(tx, getX()), dy = Integer.compare(ty, getY());
        TWDirection d = (dx != 0) ? (dx > 0 ? TWDirection.E : TWDirection.W) : (dy > 0 ? TWDirection.S : TWDirection.N);
        return new TWThought(TWAction.MOVE, d);
    }

    private TWThought patrol() {
        int[] corner = corners[cornerIdx];
        if (getX() == corner[0] && getY() == corner[1]) cornerIdx = (cornerIdx + 1) % corners.length;
        return navigate(corners[cornerIdx][0], corners[cornerIdx][1]);
    }

    public double manhattan(int x1, int y1, int x2, int y2) { 
        return Math.abs(x1 - x2) + Math.abs(y1 - y2); 
    }

    @Override
    public TWAgentWorkingMemory getMemory() { 
        return this.myMemory; 
    }

    @Override
    public String getName() { 
        return this.agentName; 
    }
}
