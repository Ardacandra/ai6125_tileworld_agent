package tileworld.agent;

import tileworld.Parameters;
import tileworld.environment.*;
import sim.util.Int2D;
import tileworld.planners.AstarPathGenerator;
import tileworld.planners.TWPath;
import java.util.*;

public class MyTWAgent extends TWAgent {
    private String name;

    // --- TUNED PARAMETERS ---
    private static final double FUEL_THRESHOLD_RATIO = 0.25;
    private static final double EMERGENCY_FUEL_RATIO = 0.35; // Higher if station unknown
    private static final int CARRY_CAPACITY = 3;
    private static final int MEMORY_LIFESPAN = 100;

    // --- STATE ---
    private int fuelStationX = -1, fuelStationY = -1;
    private final AstarPathGenerator astar;
    private final Map<Int2D, Long> lastSeenTime = new HashMap<>();
    
    // Patrol logic (Phase 1 & Phase 2 Fallback)
    private final int[][] corners;
    private int cornerIdx = 0;

    public MyTWAgent(String name, int x, int y, TWEnvironment env, double fuel) {
        super(x, y, env, fuel);
        this.name = name;
        this.astar = new AstarPathGenerator(env, this, 2000); 
        
        int maxX = env.getxDimension() - 1;
        int maxY = env.getyDimension() - 1;
        this.corners = new int[][]{{0, 0}, {maxX, 0}, {maxX, maxY}, {0, maxY}};
    }

    @Override
    public void communicate() {
        // Broadcast local sensor data using ArdaMessage format
        int r = Parameters.defaultSensorRange;
        for (int dx = -r; dx <= r; dx++) {
            for (int dy = -r; dy <= r; dy++) {
                int cx = getX() + dx, cy = getY() + dy;
                if (!getEnvironment().isValidLocation(cx, cy)) continue;
                
                Object o = getEnvironment().getObjectGrid().get(cx, cy);
                if (o instanceof TWTile) 
                    getEnvironment().receiveMessage(ArdaMessage.info(name, "tile", cx, cy, getX(), getY()));
                else if (o instanceof TWHole)
                    getEnvironment().receiveMessage(ArdaMessage.info(name, "hole", cx, cy, getX(), getY()));
                else if (o instanceof TWFuelStation)
                    getEnvironment().receiveMessage(ArdaMessage.info(name, "fuel", cx, cy, getX(), getY()));
            }
        }
    }

    @Override
    protected TWThought think() {
        updatePerception(); // Realistic Sensor Check
        processMessages();  // Listens for Teammate Phase 1 Discoveries

        double fuelRatio = getFuelLevel() / Parameters.defaultFuelLevel;
        // Use a higher safety margin if we still haven't found the station
        double threshold = (fuelStationX == -1) ? EMERGENCY_FUEL_RATIO : FUEL_THRESHOLD_RATIO;

        // 1. REFUELING (Priority 1)
        if (fuelStationX != -1 && fuelRatio < threshold) {
            if (getX() == fuelStationX && getY() == fuelStationY) 
                return new TWThought(TWAction.REFUEL, TWDirection.Z);
            return navigate(fuelStationX, fuelStationY);
        }

        // 2. IMMEDIATE ACTIONS
        Object here = getEnvironment().getObjectGrid().get(getX(), getY());
        if (here instanceof TWTile && carriedTiles.size() < CARRY_CAPACITY) 
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        if (here instanceof TWHole && !carriedTiles.isEmpty()) 
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);

        // 3. TARGETING (Phase 2 Logic)
        Int2D target = findBestTarget();
        if (target != null) return navigate(target.x, target.y);

        // 4. PATROL/SEARCH (Phase 1 Logic)
        // If fuel station is unknown, this patrol acts as your search sweep
        return patrol();
    }

    private void updatePerception() {
        int r = Parameters.defaultSensorRange;
        long now = getEnvironment().schedule.getSteps();
        for (int dx = -r; dx <= r; dx++) {
            for (int dy = -r; dy <= r; dy++) {
                int cx = getX() + dx, cy = getY() + dy;
                if (getEnvironment().isValidLocation(cx, cy)) {
                    Object o = getEnvironment().getObjectGrid().get(cx, cy);
                    Int2D pos = new Int2D(cx, cy);
                    
                    if (o instanceof TWFuelStation) {
                        this.fuelStationX = cx;
                        this.fuelStationY = cy;
                    }
                    
                    if (o != null) {
                        lastSeenTime.put(pos, now);
                    } else {
                        // Sensor confirms cell is empty: clear memory
                        getMemory().getMemoryGrid().set(cx, cy, null);
                        lastSeenTime.remove(pos);
                    }
                }
            }
        }
    }

    private void processMessages() {
        List<Message> msgs = getEnvironment().getMessages();
        for (Message m : msgs) {
            if (!(m instanceof ArdaMessage)) continue;
            ArdaMessage am = (ArdaMessage) m;
            Int2D pos = new Int2D(am.getX(), am.getY());
            
            String type = am.getEntityType();
            if ("fuel".equals(type)) {
                this.fuelStationX = am.getX();
                this.fuelStationY = am.getY();
            } else if (type.startsWith("delete")) {
                getMemory().getMemoryGrid().set(pos.x, pos.y, null);
                lastSeenTime.remove(pos);
            }
        }
    }

    private Int2D findBestTarget() {
        Int2D best = null;
        double minScore = Double.MAX_VALUE;
        long now = getEnvironment().schedule.getSteps();

        for (int x = 0; x < getEnvironment().getxDimension(); x++) {
            for (int y = 0; y < getEnvironment().getyDimension(); y++) {
                Object o = getMemory().getMemoryGrid().get(x, y);
                if (o == null || o instanceof TWFuelStation) continue;

                Long seenAt = lastSeenTime.get(new Int2D(x, y));
                if (seenAt != null && (now - seenAt) > MEMORY_LIFESPAN) continue;

                boolean isTarget = (o instanceof TWTile && carriedTiles.size() < CARRY_CAPACITY) ||
                                   (o instanceof TWHole && !carriedTiles.isEmpty());

                if (isTarget) {
                    double dist = manhattan(getX(), getY(), x, y);
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

    private TWThought navigate(int tx, int ty) {
        TWPath path = astar.findPath(getX(), getY(), tx, ty);
        if (path != null && path.hasNext()) return new TWThought(TWAction.MOVE, path.popNext().getDirection());
        
        int dx = Integer.compare(tx, getX()), dy = Integer.compare(ty, getY());
        TWDirection d = (dx != 0) ? (dx > 0 ? TWDirection.E : TWDirection.W) : (dy > 0 ? TWDirection.S : TWDirection.N);
        return new TWThought(TWAction.MOVE, getEnvironment().isCellBlocked(getX()+d.dx, getY()+d.dy) ? TWDirection.Z : d);
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
}