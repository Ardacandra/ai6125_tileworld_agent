package tileworld.agent;

import tileworld.Parameters;
import tileworld.environment.*;
import tileworld.planners.AstarPathGenerator;
import tileworld.planners.TWPath;
import tileworld.planners.TWPathStep;
import sim.util.Int2D;
import java.util.concurrent.ConcurrentHashMap;
import java.util.Map;

/**
 * MyTWAgent - Turbo Performance Version
 * Integrated with Team Blackboard and High-Reliability Pathfinding.
 */
public class EricaMyTWAgent extends TWAgent {
    private String name;
    private TWPath currentPath = null;
    
    // --- CONSISTENCY & STRATEGY PARAMETERS ---
    private static final double FUEL_THRESHOLD_RATIO       = 0.22; 
    private static final double FUEL_UNKNOWN_STATION_RATIO = 0.35; 
    private static final int    CARRY_CAPACITY             = 3;
    private static final double CARRY_BIAS_DISTANCE        = 5.0;

    // --- TEAM BLACKBOARD (Shared across ALL MyTWAgent instances) ---
    public static int fuelStationX = -1;
    public static int fuelStationY = -1;
    public static final Map<Int2D, Long> teamMemory = new ConcurrentHashMap<>();

    public EricaMyTWAgent(String name, int x, int y, TWEnvironment env, double fuel) {
        super(x, y, env, fuel);
        this.name = name;
    }

    @Override public String getName() { return this.name; }

    /**
     * Resets shared data. MUST be called at the start of every run in TileworldMain.
     */
    public static void resetTeamScore() {
        fuelStationX = -1; 
        fuelStationY = -1;
        teamMemory.clear();
    }

    private void updatePerception() {
        int r = Parameters.defaultSensorRange;
        long step = getEnvironment().schedule.getSteps();
        
        for (int i = -r; i <= r; i++) {
            for (int j = -r; j <= r; j++) {
                int tx = (int)getX() + i, ty = (int)getY() + j;
                if (!getEnvironment().isInBounds(tx, ty)) continue;
                
                Object o = getEnvironment().getObjectGrid().get(tx, ty);
                Int2D pos = new Int2D(tx, ty);
                
                if (o instanceof TWFuelStation) {
                    fuelStationX = tx; 
                    fuelStationY = ty;
                } else if (o instanceof TWHole || o instanceof TWTile) {
                    teamMemory.put(pos, step);
                } else if (o == null) {
                    teamMemory.remove(pos);
                }
            }
        }
        // Remove "ghost" tiles that have timed out
        teamMemory.entrySet().removeIf(e -> (step - e.getValue()) > Parameters.lifeTime);
    }

    @Override
    protected TWThought think() {
        updatePerception();

        // 1. SMART REFUEL LOGIC
        double fuelRatio = getFuelLevel() / Parameters.defaultFuelLevel;
        boolean knowsFuel = (fuelStationX != -1);
        
        // Stay on the station until topped up (95%)
        if (knowsFuel && (int)getX() == fuelStationX && (int)getY() == fuelStationY) {
            if (fuelRatio < 0.95) return new TWThought(TWAction.REFUEL, TWDirection.Z);
        }

        double threshold = knowsFuel ? FUEL_THRESHOLD_RATIO : FUEL_UNKNOWN_STATION_RATIO;
        if (fuelRatio < threshold) {
            if (knowsFuel) return navigateTo(fuelStationX, fuelStationY);
            // Search center if fuel is unknown
            return navigateTo(Parameters.xDimension/2, Parameters.yDimension/2);
        }

        // 2. IMMEDIATE INTERACTION
        Object here = getEnvironment().getObjectGrid().get((int)getX(), (int)getY());
        if (here instanceof TWTile && carriedTiles.size() < CARRY_CAPACITY) 
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        if (here instanceof TWHole && !carriedTiles.isEmpty()) 
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);

        // 3. TARGETING & FALLBACK
        if (currentPath == null || !currentPath.hasNext()) {
            Int2D best = findClusterTarget();
            if (best != null) {
                currentPath = getPathTo(best.x, best.y);
            } else {
                // GREEDY SEARCH FALLBACK: Scout a random corner if memory is empty
                int[][] corners = {{0,0}, {0, 49}, {49, 0}, {49, 49}, {25, 25}};
                int[] target = corners[getEnvironment().random.nextInt(5)];
                currentPath = getPathTo(target[0], target[1]);
            }
        }

        return followPath();
    }

    private Int2D findClusterTarget() {
        Int2D best = null; double maxScore = -Double.MAX_VALUE;
        Class<?> goalType = (carriedTiles.size() >= CARRY_CAPACITY) ? TWHole.class : TWTile.class;

        for (Int2D pos : teamMemory.keySet()) {
            Object obj = getEnvironment().getObjectGrid().get(pos.x, pos.y);
            if (obj == null || !obj.getClass().equals(goalType)) continue;

            double dist = Math.abs(pos.x - getX()) + Math.abs(pos.y - getY());
            double density = 0;
            for (Int2D other : teamMemory.keySet()) {
                if (pos.distance(other) < CARRY_BIAS_DISTANCE) density += 7.0; // Strong cluster preference
            }
            
            double score = density - (dist * 1.3); 
            if (score > maxScore) { maxScore = score; best = pos; }
        }
        return best;
    }

    private TWThought navigateTo(int x, int y) {
        currentPath = getPathTo(x, y);
        return followPath();
    }

    private TWThought followPath() {
        if (currentPath == null || !currentPath.hasNext()) 
            return new TWThought(TWAction.MOVE, TWDirection.values()[getEnvironment().random.nextInt(4)]);
        
        TWPathStep next = currentPath.popNext();
        // Reliability check: recalculate if blocked
        if (getEnvironment().isCellBlocked(next.getX(), next.getY())) {
            currentPath = null;
            return new TWThought(TWAction.MOVE, TWDirection.Z);
        }
        return new TWThought(TWAction.MOVE, next.getDirection());
    }

    private TWPath getPathTo(int x, int y) {
        // High 2000-node limit for robust A* navigation
        return new AstarPathGenerator(getEnvironment(), this, 2000)
               .findPath((int)getX(), (int)getY(), x, y);
    }

    @Override
    protected void act(TWThought t) {
        try {
            switch(t.getAction()){
                case MOVE: move(t.getDirection()); break;
                case REFUEL: refuel(); break;
                case PICKUP: pickUpTile((TWTile)getEnvironment().getObjectGrid().get((int)getX(),(int)getY())); break;
                case PUTDOWN: putTileInHole((TWHole)getEnvironment().getObjectGrid().get((int)getX(),(int)getY())); break;
            }
        } catch (Exception e) { currentPath = null; }
    }
}