package tileworld.agent;

import java.util.concurrent.ThreadLocalRandom;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEnvironment;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.AstarPathGenerator;
import tileworld.planners.TWPath;
import tileworld.planners.TWPathStep;

public class ArdaTWAgent_v2 extends ArdaTWAgentSkeleton {

    private static final int MANHATTAN_OBSTACLE_PENALTY = 10;
    private static final double SAFETY_MARGIN_RATIO = 0.20; // Adjust this to tune refuel behavior
    private static final double MIN_FUEL_PERCENTAGE = 0.20; // Minimum fuel threshold as safety floor

    private final AstarPathGenerator pathGenerator;
    private final int safetyMargin;

    public ArdaTWAgent_v2(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(name, xpos, ypos, env, fuelLevel);
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
            if (this.getX() == fuelStationX && this.getY() == fuelStationY) {
                return new TWThought(TWAction.REFUEL, TWDirection.Z);
            }
            return new TWThought(TWAction.MOVE, nextStepAlongPath());
        }

        return new TWThought(TWAction.MOVE, getRandomDirection());
    }

    @Override
    protected void act(TWThought thought) {
        try {
            if (thought.getAction() == TWAction.REFUEL) {
                this.refuel();
            } else {
                this.move(thought.getDirection());
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

    private TWDirection nextStepTowardFuel() {
        int dx = fuelStationX - this.getX();
        int dy = fuelStationY - this.getY();

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

    private boolean canMove(TWDirection direction) {
        if (direction == TWDirection.Z) {
            return true;
        }

        int nextX = this.getX() + direction.dx;
        int nextY = this.getY() + direction.dy;
        return this.getEnvironment().isInBounds(nextX, nextY)
                && !this.getEnvironment().isCellBlocked(nextX, nextY);
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