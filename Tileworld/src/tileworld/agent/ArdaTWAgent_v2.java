package tileworld.agent;

import java.util.concurrent.ThreadLocalRandom;
import tileworld.Parameters;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEnvironment;
import tileworld.exceptions.CellBlockedException;

public class ArdaTWAgent_v2 extends ArdaTWAgentSkeleton {

    private static final double REFUEL_THRESHOLD_RATIO = 0.20;

    public ArdaTWAgent_v2(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(name, xpos, ypos, env, fuelLevel);
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
            return new TWThought(TWAction.MOVE, nextStepTowardFuel());
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
        return this.getFuelLevel() <= Parameters.defaultFuelLevel * REFUEL_THRESHOLD_RATIO;
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