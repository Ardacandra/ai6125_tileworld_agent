package tileworld.agent;

import tileworld.Parameters;
import tileworld.environment.*;
import sim.util.Int2D;
import tileworld.planners.AstarPathGenerator;
import tileworld.planners.TWPath;
import java.util.*;

public class MyTWAgent extends TWAgent {
    private String name;

  
    private static final double FUEL_THRESHOLD_RATIO = 0.28; \
    private static final int CARRY_CAPACITY = 3;
    private static final int MEMORY_LIFESPAN = 100; 

   
    private int fuelStationX = -1, fuelStationY = -1;
    private final AstarPathGenerator astar;
    private final Map<Int2D, Long> lastSeenTime = new HashMap<>();
    
    // Phase 1 Integration
    private final Phase1Strategy phase1;
    private final int[][] corners;
    private int cornerIdx = 0;
    private boolean isLargeMap;

    public MyTWAgent(String name, int x, int y, TWEnvironment env, double fuel) {
        super(x, y, env, fuel);
        this.name = name;
        this.astar = new AstarPathGenerator(env, this, 2000); 
        this.phase1 = new Phase1Strategy(this);
        
        // Dynamic Check: Is this a big map?
        this.isLargeMap = (env.getxDimension() >= 60 || env.getyDimension() >= 60);
        
        int maxX = env.getxDimension() - 1;
        int maxY = env.getyDimension() - 1;
        this.corners = new int[][]{{0, 0}, {maxX, 0}, {maxX, maxY}, {0, maxY}};
    }

    @Override
    public void communicate() {
        // 1. Dynamic Discovery (Only if large map and station not found)
        if (isLargeMap && !phase1.isComplete()) {
            phase1.communicate(); 
        }

        // 2. Standard Team Communication (Your High-Utility Logic)
        int r = Parameters.defaultSensorRange;
        for (int dx = -r; dx <= r; dx++) {
            for (int dy = -r; dy <= r; dy++) {
                int cx = getX() + dx, cy = getY() + dy;
                if (!getEnvironment().isValidLocation(cx, cy)) continue;
                
                Object o = getEnvironment().getObjectGrid().get(cx, cy);
                if (o instanceof TWFuelStation) {
                    this.fuelStationX = cx;
                    this.fuelStationY = cy;
                    getEnvironment().receiveMessage(ArdaMessage.info(name, "fuel", cx, cy, getX(), getY()));
                } else if (o instanceof TWTile) 
                    getEnvironment().receiveMessage(ArdaMessage.info(name, "tile", cx, cy, getX(), getY()));
                else if (o instanceof TWHole)
                    getEnvironment().receiveMessage(ArdaMessage.info(name, "hole", cx, cy, getX(), getY()));
            }
        }
    }

    @Override
    protected TWThought think() {
        processMessages();
        updateLocalTimestamps();
        
        double fuelLevel = getFuelLevel();
        double fuelRatio = fuelLevel / Parameters.defaultFuelLevel;

        // --- STEP 1: DYNAMIC PHASE 1 (Only for Large Maps) ---
        if (isLargeMap && !phase1.isComplete()) {
            TWThought p1Thought = phase1.think();
            if (p1Thought != null) return p1Thought;
            
            // Sync fuel station if Phase 1 found it
            if (phase1.getFuelStation() != null) {
                this.fuelStationX = phase1.getFuelStation().x;
                this.fuelStationY = phase1.getFuelStation().y;
            }
        }

        // --- STEP 2: REFUELING (Priority) ---
        if (fuelStationX != -1) {
            double dist = manhattan(getX(), getY(), fuelStationX, fuelStationY);
            // Dynamic Safety: Larger maps need more padding
            double safetyPadding = isLargeMap ? 15 : 5;
            if (fuelLevel < (dist * 2.0 + safetyPadding) || fuelRatio < FUEL_THRESHOLD_RATIO) {
                if (getX() == fuelStationX && getY() == fuelStationY) 
                    return new TWThought(TWAction.REFUEL, TWDirection.Z);
                return navigate(fuelStationX, fuelStationY);
            }
        }

        // --- STEP 3: IMMEDIATE ACTIONS 
        Object here = getEnvironment().getObjectGrid().get(getX(), getY());
        if (here instanceof TWTile && carriedTiles.size() < CARRY_CAPACITY) 
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        if (here instanceof TWHole && !carriedTiles.isEmpty()) 
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);

        // --- STEP 4: TARGETING  ---
        Int2D target = findBestTarget();
        if (target != null) return navigate(target.x, target.y);

        // --- STEP 5: PATROL / WANDER ---
        return patrol();
    }

    private Int2D findBestTarget() {
        Int2D best = null;
        double minScore = Double.MAX_VALUE;
        long now = getEnvironment().schedule.getSteps();
        
        // Range Limit: Prevents "Death Marches" in 80x80
        int searchRange = isLargeMap ? 30 : 100; 

        for (int x = 0; x < getEnvironment().getxDimension(); x++) {
            for (int y = 0; y < getEnvironment().getyDimension(); y++) {
                // Optimization: skip far away targets in large maps
                if (isLargeMap && manhattan(getX(), getY(), x, y) > searchRange) continue;

                Object o = getMemory().getMemoryGrid().get(x, y);
                if (o == null) continue;

                Long seenAt = lastSeenTime.get(new Int2D(x, y));
                if (seenAt != null && (now - seenAt) > MEMORY_LIFESPAN) continue;

                boolean isTarget = (o instanceof TWTile && carriedTiles.size() < CARRY_CAPACITY) ||
                                   (o instanceof TWHole && !carriedTiles.isEmpty());
                if (isTarget) {
                    double dist = manhattan(getX(), getY(), x, y);
                    // YOUR WINNING BIAS: Prioritize holes
                    double score = (o instanceof TWHole) ? dist - 5 : dist; 
                    
                    if (score < minScore) {
                        minScore = score;
                        best = new Int2D(x, y);
                    }
                }
            }
        }
        return best;
    }

    private void updateLocalTimestamps() {
        int r = Parameters.defaultSensorRange;
        long now = getEnvironment().schedule.getSteps();
        for (int dx = -r; dx <= r; dx++) {
            for (int dy = -r; dy <= r; dy++) {
                int cx = getX() + dx, cy = getY() + dy;
                if (getEnvironment().isValidLocation(cx, cy)) {
                    lastSeenTime.put(new Int2D(cx, cy), now);
                    if (getEnvironment().getObjectGrid().get(cx, cy) == null) {
                        getMemory().getMemoryGrid().set(cx, cy, null);
                    }
                }
            }
        }
    }

    private void processMessages() {
        for (Object mObj : getEnvironment().getMessages()) {
            if (!(mObj instanceof ArdaMessage)) continue;
            ArdaMessage am = (ArdaMessage) mObj;
            if (am.getEntityType().startsWith("delete")) {
                getMemory().getMemoryGrid().set(am.getX(), am.getY(), null);
                lastSeenTime.remove(new Int2D(am.getX(), am.getY()));
            } else if (am.getEntityType().equals("fuel")) {
                fuelStationX = am.getX(); fuelStationY = am.getY();
            }
        }
    }

    private TWThought navigate(int tx, int ty) {
        TWPath path = astar.findPath(getX(), getY(), tx, ty);
        if (path != null && path.hasNext()) return new TWThought(TWAction.MOVE, path.popNext().getDirection());
        
        int dx = Integer.compare(tx, getX()), dy = Integer.compare(ty, getY());
        TWDirection d = (dx != 0) ? (dx > 0 ? TWDirection.E : TWDirection.W) : (dy > 0 ? TWDirection.S : TWDirection.N);
        return new TWThought(TWAction.MOVE, d);
    }

    private TWThought patrol() {
        int[] corner = corners[cornerIdx];
        if (getX() == corner[0] && getY() == corner[1]) cornerIdx = (cornerIdx + 1) % corners.length;
        return navigate(corners[cornerIdx][0], corners[cornerIdx][1]);
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
                    getMemory().getMemoryGrid().set(ax, ay, null);
                    getEnvironment().receiveMessage(ArdaMessage.info(name, "delete_tile", ax, ay, ax, ay));
                    break;
                case PUTDOWN:
                    putTileInHole((TWHole) getEnvironment().getObjectGrid().get(ax, ay));
                    getMemory().getMemoryGrid().set(ax, ay, null);
                    getEnvironment().receiveMessage(ArdaMessage.info(name, "delete_hole", ax, ay, ax, ay));
                    break;
            }
        } catch (Exception e) {}
    }

    private double manhattan(int x1, int y1, int x2, int y2) { return Math.abs(x1 - x2) + Math.abs(y1 - y2); }
    @Override public String getName() { return name; }
    public int getFuelStationX() { return fuelStationX; }
    public int getFuelStationY() { return fuelStationY; }
}
