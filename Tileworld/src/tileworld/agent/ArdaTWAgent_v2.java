package tileworld.agent;

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

    private final AstarPathGenerator pathGenerator;
    private final ArdaCustomTWAgentMemory customMemory;
    private final int safetyMargin;

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
        if (fuelStationX >= 0 && fuelStationY >= 0 && shouldRefuel()) {
            setIntention(ENTITY_FUEL, fuelStationX, fuelStationY);
            if (this.getX() == fuelStationX && this.getY() == fuelStationY) {
                return new TWThought(TWAction.REFUEL, TWDirection.Z);
            }
            return new TWThought(TWAction.MOVE, nextStepAlongPath());
        }

        TWTile tileHere = getTileAtCurrentPosition();
        if (tileHere != null && carriedTiles.size() < CARRY_CAPACITY) {
            setIntention(ENTITY_TILE, this.getX(), this.getY());
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }

        TWHole holeHere = getHoleAtCurrentPosition();
        if (holeHere != null && hasTile()) {
            setIntention(ENTITY_HOLE, this.getX(), this.getY());
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }

        int[] closestTile = getClosestTileTarget();
        int[] closestHole = getClosestHoleTarget();
        int tileDistance = (closestTile == null) ? Integer.MAX_VALUE : manhattanDistanceTo(closestTile[0], closestTile[1]);
        int holeDistance = (closestHole == null) ? Integer.MAX_VALUE : manhattanDistanceTo(closestHole[0], closestHole[1]);

        if (hasTile() && closestHole != null && holeDistance < tileDistance) {
            setIntention(ENTITY_HOLE, closestHole[0], closestHole[1]);
            return new TWThought(TWAction.MOVE, nextStepTowardTarget(closestHole[0], closestHole[1]));
        }

        if (carriedTiles.size() < CARRY_CAPACITY && closestTile != null && tileDistance <= holeDistance) {
            setIntention(ENTITY_TILE, closestTile[0], closestTile[1]);
            return new TWThought(TWAction.MOVE, nextStepTowardTarget(closestTile[0], closestTile[1]));
        }

        if (hasTile() && closestHole != null) {
            setIntention(ENTITY_HOLE, closestHole[0], closestHole[1]);
            return new TWThought(TWAction.MOVE, nextStepTowardTarget(closestHole[0], closestHole[1]));
        }

        clearIntention();
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

        if (path != null && path.hasNext()) {
            TWPathStep step = path.popNext();
            if (step.getDirection() != TWDirection.Z) {
                return step.getDirection();
            }
        }

        // Fallback to greedy if no valid path
        return nextStepTowardFuel();
    }

    private TWDirection nextStepTowardTarget(int targetX, int targetY) {
        TWPath path = pathGenerator.findPath(this.getX(), this.getY(), targetX, targetY);

        if (path != null && path.hasNext()) {
            TWPathStep step = path.popNext();
            if (step.getDirection() != TWDirection.Z) {
                return step.getDirection();
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
        int bestDistance = Integer.MAX_VALUE;
        int[] best = null;

        TWTile observed = (TWTile) this.getMemory().getClosestObjectInSensorRange(TWTile.class);
        if (observed != null) {
            int observedDistance = manhattanDistanceTo(observed.getX(), observed.getY());
            if (!isClaimedByCloserAgent(ENTITY_TILE, observed.getX(), observed.getY())) {
                bestDistance = observedDistance;
                best = new int[] { observed.getX(), observed.getY() };
            }
        }

        for (ArdaCustomTWAgentMemory.MemoryEntry entry : customMemory.getKnownTiles()) {
            if (isClaimedByCloserAgent(ENTITY_TILE, entry.x, entry.y)) {
                continue;
            }
            int d = manhattanDistanceTo(entry.x, entry.y);
            if (d < bestDistance) {
                bestDistance = d;
                best = new int[] { entry.x, entry.y };
            }
        }

        for (int[] shared : getSharedTileLocations()) {
            if (isClaimedByCloserAgent(ENTITY_TILE, shared[0], shared[1])) {
                continue;
            }
            int d = manhattanDistanceTo(shared[0], shared[1]);
            if (d < bestDistance) {
                bestDistance = d;
                best = new int[] { shared[0], shared[1] };
            }
        }

        return best;
    }

    private int[] getClosestHoleTarget() {
        int bestDistance = Integer.MAX_VALUE;
        int[] best = null;

        TWHole observed = (TWHole) this.getMemory().getClosestObjectInSensorRange(TWHole.class);
        if (observed != null) {
            int observedDistance = manhattanDistanceTo(observed.getX(), observed.getY());
            if (!isClaimedByCloserAgent(ENTITY_HOLE, observed.getX(), observed.getY())) {
                bestDistance = observedDistance;
                best = new int[] { observed.getX(), observed.getY() };
            }
        }

        for (ArdaCustomTWAgentMemory.MemoryEntry entry : customMemory.getKnownHoles()) {
            if (isClaimedByCloserAgent(ENTITY_HOLE, entry.x, entry.y)) {
                continue;
            }
            int d = manhattanDistanceTo(entry.x, entry.y);
            if (d < bestDistance) {
                bestDistance = d;
                best = new int[] { entry.x, entry.y };
            }
        }

        for (int[] shared : getSharedHoleLocations()) {
            if (isClaimedByCloserAgent(ENTITY_HOLE, shared[0], shared[1])) {
                continue;
            }
            int d = manhattanDistanceTo(shared[0], shared[1]);
            if (d < bestDistance) {
                bestDistance = d;
                best = new int[] { shared[0], shared[1] };
            }
        }

        return best;
    }

    private int manhattanDistanceTo(int targetX, int targetY) {
        return Math.abs(targetX - this.getX()) + Math.abs(targetY - this.getY());
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