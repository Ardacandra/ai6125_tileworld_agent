package tileworld.agent;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ThreadLocalRandom;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.AstarPathGenerator;
import tileworld.planners.TWPath;
import tileworld.planners.TWPathStep;

public class ArdaTWAgent_v2 extends TWAgentSkeleton {

    private static final int MANHATTAN_OBSTACLE_PENALTY = 10;
    private static final double SAFETY_MARGIN_RATIO = 0.20; // Adjust this to tune refuel behavior
    private static final double MIN_FUEL_PERCENTAGE = 0.20; // Minimum fuel threshold as safety floor
    private static final double MIN_MEMORY_TARGET_AGE = 8.0;
    private static final double MAX_MEMORY_TARGET_AGE = 80.0;
    private static final double MISSING_TARGET_BLACKLIST_STEPS = 15.0;
    private static final double DELIVERY_CHURN_THRESHOLD = 0.25;
    private static final int LOW_CHURN_HOLE_ADVANTAGE = 2;

    private final AstarPathGenerator pathGenerator;
    private final ArdaCustomTWAgentMemory customMemory;
    private final int safetyMargin;
    private String pendingTargetType = "";
    private int pendingTargetX = -1;
    private int pendingTargetY = -1;
    private int resolvedMemoryTargets = 0;
    private int missedMemoryTargets = 0;
    private double localStep = 0.0;
    private final Map<String, Double> blacklistedTargets = new HashMap<>();

    public ArdaTWAgent_v2(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(name, xpos, ypos, env, fuelLevel);
        this.customMemory = new ArdaCustomTWAgentMemory(
            this, env.schedule,
                env.getxDimension(), env.getyDimension());
        this.memory = customMemory;
        this.pathGenerator = new AstarPathGenerator(env, this, env.getxDimension() * env.getyDimension());
        // Calculate safety margin as a ratio of average grid dimension
        this.safetyMargin = (int) ((env.getxDimension() + env.getyDimension()) / 2 * SAFETY_MARGIN_RATIO);
    }

    @Override
    protected void customCommunicate() {
        // No Phase 2 coordination for the simple baseline agent.
    }

    @Override
    protected void handleTeamMessage(ArdaMessage msg) {
        // No-op: this baseline agent only uses shared fuel handling from the skeleton.
    }

    @Override
    protected TWThought customThink() {
        localStep++;

        TWTile tileHere = getTileAtCurrentPosition();
        TWHole holeHere = getHoleAtCurrentPosition();
        resolvePendingTargetIfReached(tileHere, holeHere);

        if (fuelStationX >= 0 && fuelStationY >= 0 && shouldRefuel()) {
            setIntention(ENTITY_FUEL, fuelStationX, fuelStationY);
            rememberPendingTarget(ENTITY_FUEL, fuelStationX, fuelStationY);
            if (this.getX() == fuelStationX && this.getY() == fuelStationY) {
                return new TWThought(TWAction.REFUEL, TWDirection.Z);
            }
            return new TWThought(TWAction.MOVE, nextStepAlongPath());
        }

        if (tileHere != null && carriedTiles.size() < CARRY_CAPACITY) {
            setIntention(ENTITY_TILE, this.getX(), this.getY());
            rememberPendingTarget(ENTITY_TILE, this.getX(), this.getY());
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }

        if (holeHere != null && hasTile()) {
            setIntention(ENTITY_HOLE, this.getX(), this.getY());
            rememberPendingTarget(ENTITY_HOLE, this.getX(), this.getY());
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }

        int[] closestTile = getClosestTileTarget();
        int[] closestHole = getClosestHoleTarget();

        if (!hasTile()) {
            if (closestTile != null) {
                setIntention(ENTITY_TILE, closestTile[0], closestTile[1]);
                rememberPendingTarget(ENTITY_TILE, closestTile[0], closestTile[1]);
                return new TWThought(TWAction.MOVE, nextStepTowardTarget(closestTile[0], closestTile[1]));
            }
            clearIntention();
            clearPendingTarget();
            return new TWThought(TWAction.MOVE, getRandomDirection());
        }

        if (shouldDeliverNow(closestTile, closestHole)) {
            setIntention(ENTITY_HOLE, closestHole[0], closestHole[1]);
            rememberPendingTarget(ENTITY_HOLE, closestHole[0], closestHole[1]);
            return new TWThought(TWAction.MOVE, nextStepTowardTarget(closestHole[0], closestHole[1]));
        }

        if (carriedTiles.size() < CARRY_CAPACITY && closestTile != null) {
            setIntention(ENTITY_TILE, closestTile[0], closestTile[1]);
            rememberPendingTarget(ENTITY_TILE, closestTile[0], closestTile[1]);
            return new TWThought(TWAction.MOVE, nextStepTowardTarget(closestTile[0], closestTile[1]));
        }

        if (hasTile() && closestHole != null) {
            setIntention(ENTITY_HOLE, closestHole[0], closestHole[1]);
            rememberPendingTarget(ENTITY_HOLE, closestHole[0], closestHole[1]);
            return new TWThought(TWAction.MOVE, nextStepTowardTarget(closestHole[0], closestHole[1]));
        }

        clearIntention();
        clearPendingTarget();
        return new TWThought(TWAction.MOVE, getRandomDirection());
    }

    @Override
    protected void act(TWThought thought) {
        try {
            switch (thought.getAction()) {
                case REFUEL:
                    this.refuel();
                    break;
                case PICKUP:
                    TWTile tile = getTileAtCurrentPosition();
                    if (tile != null && carriedTiles.size() < CARRY_CAPACITY) {
                        this.pickUpTile(tile);
                        customMemory.removeTile(this.getX(), this.getY());
                        forgetSharedTile(this.getX(), this.getY());
                    }
                    break;
                case PUTDOWN:
                    TWHole hole = getHoleAtCurrentPosition();
                    if (hole != null && hasTile()) {
                        this.putTileInHole(hole);
                        customMemory.removeHole(this.getX(), this.getY());
                        forgetSharedHole(this.getX(), this.getY());
                    }
                    break;
                case MOVE:
                default:
                    this.move(thought.getDirection());
                    break;
            }
        } catch (CellBlockedException ex) {
            // Cell is blocked; try again next step.
        }
    }

    private boolean shouldRefuel() {
        if (fuelStationX < 0 || fuelStationY < 0) {
            return false;
        }

        // Floor: always refuel if fuel drops below minimum percentage
        boolean belowMinPercentage = this.getFuelLevel() <= tileworld.Parameters.defaultFuelLevel * MIN_FUEL_PERCENTAGE;
        if (belowMinPercentage) {
            System.out.println(this.getName() + " [RefuelV2] Below minimum fuel percentage (" 
                    + MIN_FUEL_PERCENTAGE * 100 + "%). Fuel=" + this.getFuelLevel());
            return true;
        }

        TWPath path = pathGenerator.findPath(this.getX(), this.getY(), fuelStationX, fuelStationY);

        if (path == null) {
            // No path in memory; use Manhattan as pessimistic estimate with obstacle penalty
            int manhattanDist = Math.abs(fuelStationX - this.getX()) + Math.abs(fuelStationY - this.getY());
            int estimatedCost = manhattanDist + MANHATTAN_OBSTACLE_PENALTY;
            boolean shouldRefuel = this.getFuelLevel() <= estimatedCost + safetyMargin;
            if (shouldRefuel) {
                System.out.println(this.getName() + " [RefuelV2] Path blocked or not found. Manhattan=" + manhattanDist
                        + " + penalty=" + MANHATTAN_OBSTACLE_PENALTY + " + margin=" + safetyMargin + ". Fuel=" + this.getFuelLevel());
            }
            return shouldRefuel;
        }

        // Path exists; count actual steps
        int pathLength = countPathSteps(path);
        boolean shouldRefuel = this.getFuelLevel() <= pathLength + safetyMargin;

        if (shouldRefuel) {
            System.out.println(this.getName() + " [RefuelV2] A* pathLength=" + pathLength
                    + " + margin=" + safetyMargin + ". Fuel=" + this.getFuelLevel());
        }

        return shouldRefuel;
    }

    private int countPathSteps(TWPath path) {
        int count = 0;
        while (path.hasNext()) {
            path.popNext();
            count++;
        }
        return count;
    }

    private TWDirection nextStepAlongPath() {
        TWPath path = pathGenerator.findPath(this.getX(), this.getY(), fuelStationX, fuelStationY);

        if (path != null) {
            while (path.hasNext()) {
                TWPathStep step = path.popNext();
                if (step.getDirection() != TWDirection.Z) {
                    return step.getDirection();
                }
            }
        }

        // Fallback to greedy if no valid path
        return nextStepTowardFuel();
    }

    private TWDirection nextStepTowardTarget(int targetX, int targetY) {
        TWPath path = pathGenerator.findPath(this.getX(), this.getY(), targetX, targetY);

        if (path != null) {
            while (path.hasNext()) {
                TWPathStep step = path.popNext();
                if (step.getDirection() != TWDirection.Z) {
                    return step.getDirection();
                }
            }
        }

        return nextStepTowardCoordinates(targetX, targetY);
    }

    private TWDirection nextStepTowardFuel() {
        return nextStepTowardCoordinates(fuelStationX, fuelStationY);
    }

    private TWDirection nextStepTowardCoordinates(int targetX, int targetY) {
        int dx = targetX - this.getX();
        int dy = targetY - this.getY();

        TWDirection primary = Math.abs(dx) >= Math.abs(dy)
                ? (dx > 0 ? TWDirection.E : TWDirection.W)
                : (dy > 0 ? TWDirection.S : TWDirection.N);
        TWDirection secondary = Math.abs(dx) >= Math.abs(dy)
                ? (dy > 0 ? TWDirection.S : TWDirection.N)
                : (dx > 0 ? TWDirection.E : TWDirection.W);

        if ((dx != 0 || dy != 0) && canMove(primary)) {
            return primary;
        }
        if ((dx != 0 || dy != 0) && canMove(secondary)) {
            return secondary;
        }

        return getRandomDirection();
    }

    private TWTile getTileAtCurrentPosition() {
        TWTile observed = (TWTile) this.getMemory().getClosestObjectInSensorRange(TWTile.class);
        if (observed != null && observed.getX() == this.getX() && observed.getY() == this.getY()) {
            return observed;
        }
        return null;
    }

    private TWHole getHoleAtCurrentPosition() {
        TWHole observed = (TWHole) this.getMemory().getClosestObjectInSensorRange(TWHole.class);
        if (observed != null && observed.getX() == this.getX() && observed.getY() == this.getY()) {
            return observed;
        }
        return null;
    }

    private int[] getClosestTileTarget() {
        double bestUtility = -1.0;
        int[] best = null;

        TWTile observed = (TWTile) this.getMemory().getClosestObjectInSensorRange(TWTile.class);
        if (observed != null) {
            if (!isBlacklisted(ENTITY_TILE, observed.getX(), observed.getY())
                    && !isClaimedByCloserAgent(ENTITY_TILE, observed.getX(), observed.getY())) {
                // Observed tiles get max priority (freshly in sensor range)
                bestUtility = Double.MAX_VALUE;
                best = new int[] { observed.getX(), observed.getY() };
            }
        }

        // Prefer memory entries by utility (freshness / distance), not just distance
        for (ArdaCustomTWAgentMemory.MemoryEntry entry : customMemory.getKnownTiles()) {
            if (!isMemoryEntryFresh(entry)) {
                continue;
            }
            if (isBlacklisted(ENTITY_TILE, entry.x, entry.y)) {
                continue;
            }
            if (isClaimedByCloserAgent(ENTITY_TILE, entry.x, entry.y)) {
                continue;
            }
            if (entry.utility > bestUtility) {
                bestUtility = entry.utility;
                best = new int[] { entry.x, entry.y };
            }
        }

        for (int[] shared : getSharedTileLocations()) {
            if (isBlacklisted(ENTITY_TILE, shared[0], shared[1])) {
                continue;
            }
            if (isClaimedByCloserAgent(ENTITY_TILE, shared[0], shared[1])) {
                continue;
            }
            // For shared targets, use utility weight similar to memory entries
            double sharedUtility = 1.0 / (manhattanDistanceTo(shared[0], shared[1]) + 1.0);
            if (sharedUtility > bestUtility) {
                bestUtility = sharedUtility;
                best = new int[] { shared[0], shared[1] };
            }
        }

        return best;
    }

    private int[] getClosestHoleTarget() {
        double bestUtility = -1.0;
        int[] best = null;

        TWHole observed = (TWHole) this.getMemory().getClosestObjectInSensorRange(TWHole.class);
        if (observed != null) {
            if (!isBlacklisted(ENTITY_HOLE, observed.getX(), observed.getY())
                    && !isClaimedByCloserAgent(ENTITY_HOLE, observed.getX(), observed.getY())) {
                // Observed holes get max priority (freshly in sensor range)
                bestUtility = Double.MAX_VALUE;
                best = new int[] { observed.getX(), observed.getY() };
            }
        }

        // Prefer memory entries by utility (freshness / distance), not just distance
        for (ArdaCustomTWAgentMemory.MemoryEntry entry : customMemory.getKnownHoles()) {
            if (!isMemoryEntryFresh(entry)) {
                continue;
            }
            if (isBlacklisted(ENTITY_HOLE, entry.x, entry.y)) {
                continue;
            }
            if (isClaimedByCloserAgent(ENTITY_HOLE, entry.x, entry.y)) {
                continue;
            }
            if (entry.utility > bestUtility) {
                bestUtility = entry.utility;
                best = new int[] { entry.x, entry.y };
            }
        }

        for (int[] shared : getSharedHoleLocations()) {
            if (isBlacklisted(ENTITY_HOLE, shared[0], shared[1])) {
                continue;
            }
            if (isClaimedByCloserAgent(ENTITY_HOLE, shared[0], shared[1])) {
                continue;
            }
            // For shared targets, use utility weight similar to memory entries
            double sharedUtility = 1.0 / (manhattanDistanceTo(shared[0], shared[1]) + 1.0);
            if (sharedUtility > bestUtility) {
                bestUtility = sharedUtility;
                best = new int[] { shared[0], shared[1] };
            }
        }

        return best;
    }

    private int manhattanDistanceTo(int targetX, int targetY) {
        return Math.abs(targetX - this.getX()) + Math.abs(targetY - this.getY());
    }

    private boolean shouldDeliverNow(int[] closestTile, int[] closestHole) {
        if (!hasTile() || closestHole == null) {
            return false;
        }

        // Always deliver when inventory is full, or when no tile target exists.
        if (carriedTiles.size() >= CARRY_CAPACITY || closestTile == null) {
            return true;
        }

        int tileDistance = manhattanDistanceTo(closestTile[0], closestTile[1]);
        int holeDistance = manhattanDistanceTo(closestHole[0], closestHole[1]);
        double missRate = getObservedMissRate();

        // Low churn: preserve Config 1 behavior by favoring batching/collection.
        if (missRate < DELIVERY_CHURN_THRESHOLD) {
            if (carriedTiles.size() <= 1) {
                return false;
            }
            return holeDistance + LOW_CHURN_HOLE_ADVANTAGE < tileDistance;
        }

        double holeUrgency = targetUrgencyScore(closestHole[0], closestHole[1]);
        double tileUrgency = targetUrgencyScore(closestTile[0], closestTile[1]);

        // More carried tiles and higher miss-rate (high churn) should bias toward delivery-first.
        double carryBias = 1.0 + 0.20 * carriedTiles.size();
        double churnBias = 1.0 + missRate;

        return holeUrgency * carryBias * churnBias >= tileUrgency;
    }

    private double targetUrgencyScore(int targetX, int targetY) {
        return 1.0 / (manhattanDistanceTo(targetX, targetY) + 1.0);
    }

    private double getObservedMissRate() {
        if (resolvedMemoryTargets <= 0) {
            return 0.0;
        }
        double missRate = missedMemoryTargets / (double) resolvedMemoryTargets;
        if (missRate < 0.0) {
            return 0.0;
        }
        if (missRate > 1.0) {
            return 1.0;
        }
        return missRate;
    }

    private void rememberPendingTarget(String targetType, int targetX, int targetY) {
        pendingTargetType = targetType;
        pendingTargetX = targetX;
        pendingTargetY = targetY;
    }

    private void clearPendingTarget() {
        pendingTargetType = "";
        pendingTargetX = -1;
        pendingTargetY = -1;
    }

    private void resolvePendingTargetIfReached(TWTile tileHere, TWHole holeHere) {
        if ("".equals(pendingTargetType)) {
            return;
        }
        if (this.getX() != pendingTargetX || this.getY() != pendingTargetY) {
            return;
        }

        if (ENTITY_TILE.equals(pendingTargetType)) {
            resolvedMemoryTargets++;
            if (tileHere == null) {
                missedMemoryTargets++;
                customMemory.removeTile(pendingTargetX, pendingTargetY);
                forgetSharedTile(pendingTargetX, pendingTargetY);
                addToBlacklist(ENTITY_TILE, pendingTargetX, pendingTargetY);
            }
        } else if (ENTITY_HOLE.equals(pendingTargetType)) {
            resolvedMemoryTargets++;
            if (holeHere == null) {
                missedMemoryTargets++;
                customMemory.removeHole(pendingTargetX, pendingTargetY);
                forgetSharedHole(pendingTargetX, pendingTargetY);
                addToBlacklist(ENTITY_HOLE, pendingTargetX, pendingTargetY);
            }
        }

        clearPendingTarget();
    }

    private boolean isMemoryEntryFresh(ArdaCustomTWAgentMemory.MemoryEntry entry) {
        double age = localStep - entry.observedAt;
        return age <= getAdaptiveMemoryAgeLimit();
    }

    private double getAdaptiveMemoryAgeLimit() {
        if (resolvedMemoryTargets <= 0) {
            return MAX_MEMORY_TARGET_AGE;
        }

        double missRate = getObservedMissRate();

        return MAX_MEMORY_TARGET_AGE - (MAX_MEMORY_TARGET_AGE - MIN_MEMORY_TARGET_AGE) * missRate;
    }

    private void addToBlacklist(String targetType, int targetX, int targetY) {
        blacklistedTargets.put(blacklistKey(targetType, targetX, targetY), localStep + MISSING_TARGET_BLACKLIST_STEPS);
    }

    private boolean isBlacklisted(String targetType, int targetX, int targetY) {
        String key = blacklistKey(targetType, targetX, targetY);
        Double expiresAt = blacklistedTargets.get(key);
        if (expiresAt == null) {
            return false;
        }
        if (expiresAt <= localStep) {
            blacklistedTargets.remove(key);
            return false;
        }
        return true;
    }

    private String blacklistKey(String targetType, int targetX, int targetY) {
        return targetType + ":" + targetX + ":" + targetY;
    }

    private boolean canMove(TWDirection direction) {
        if (direction == TWDirection.Z) {
            return true;
        }

        int nextX = this.getX() + direction.dx;
        int nextY = this.getY() + direction.dy;
        return this.getEnvironment().isInBounds(nextX, nextY)
                && !this.getMemory().isCellBlocked(nextX, nextY);
    }

    private TWDirection getRandomDirection() {
        TWDirection randomDir = TWDirection.values()[ThreadLocalRandom.current().nextInt(5)];

        if (this.getX() >= this.getEnvironment().getxDimension()) {
            randomDir = TWDirection.W;
        } else if (this.getX() <= 1) {
            randomDir = TWDirection.E;
        } else if (this.getY() <= 1) {
            randomDir = TWDirection.S;
        } else if (this.getY() >= this.getEnvironment().getyDimension()) {
            randomDir = TWDirection.N;
        }

        return randomDir;
    }
}