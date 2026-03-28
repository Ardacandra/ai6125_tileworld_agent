package tileworld.agent;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ThreadLocalRandom;
import tileworld.Parameters;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.AstarPathGenerator;
import tileworld.planners.TWPath;
import tileworld.planners.TWPathStep;

public class ArdaTWAgent_v1 extends TWAgent {
    private static final double FUEL_THRESHOLD_RATIO = 0.20;
    private static final String TYPE_TILE = "tile";
    private static final String TYPE_HOLE = "hole";
    private static final String TYPE_FUEL = "fuel";

    private final String name;

    private String intendedType = "";
    private int intendedX = -1;
    private int intendedY = -1;

    private int fuelX = -1;
    private int fuelY = -1;

    private final AstarPathGenerator pathGenerator;

    private final Map<String, SharedLocation> knownTiles = new HashMap<String, SharedLocation>();
    private final Map<String, SharedLocation> knownHoles = new HashMap<String, SharedLocation>();
    private final List<IntentInfo> receivedIntentions = new ArrayList<IntentInfo>();

    private static class SharedLocation {
        private final int x;
        private final int y;

        private SharedLocation(int x, int y) {
            this.x = x;
            this.y = y;
        }
    }

    private static class IntentInfo {
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

    public ArdaTWAgent_v1(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.name = name;
        this.pathGenerator = new AstarPathGenerator(env, this, env.getxDimension() * env.getyDimension());
    }

    @Override
    protected TWThought think() {
        ingestMessages();
        refreshKnowledge();

        if (shouldRefuel() && fuelX >= 0 && fuelY >= 0) {
            rememberIntention(TYPE_FUEL, fuelX, fuelY);
            if (this.getX() == fuelX && this.getY() == fuelY) {
                return new TWThought(TWAction.REFUEL, TWDirection.Z);
            }
            return new TWThought(TWAction.MOVE, nextStepToward(fuelX, fuelY));
        }

        TWEntity currentCell = (TWEntity) this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());
        if (currentCell instanceof TWHole && this.hasTile()) {
            rememberIntention(TYPE_HOLE, this.getX(), this.getY());
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }
        if (currentCell instanceof TWTile && this.carriedTiles.size() < 3) {
            rememberIntention(TYPE_TILE, this.getX(), this.getY());
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }

        if (!this.hasTile()) {
            SharedLocation tileTarget = selectClosestTarget(TYPE_TILE, knownTiles);
            if (tileTarget != null) {
                rememberIntention(TYPE_TILE, tileTarget.x, tileTarget.y);
                return new TWThought(TWAction.MOVE, nextStepToward(tileTarget.x, tileTarget.y));
            }
            clearIntention();
            return new TWThought(TWAction.MOVE, getRandomDirection());
        }

        if (this.carriedTiles.size() >= 3) {
            SharedLocation holeTarget = selectClosestTarget(TYPE_HOLE, knownHoles);
            if (holeTarget != null) {
                rememberIntention(TYPE_HOLE, holeTarget.x, holeTarget.y);
                return new TWThought(TWAction.MOVE, nextStepToward(holeTarget.x, holeTarget.y));
            }
            clearIntention();
            return new TWThought(TWAction.MOVE, getRandomDirection());
        }

        SharedLocation holeTarget = selectClosestTarget(TYPE_HOLE, knownHoles);
        SharedLocation tileTarget = selectClosestTarget(TYPE_TILE, knownTiles);
        if (holeTarget != null && tileTarget != null) {
            double holeDistance = this.getDistanceTo(holeTarget.x, holeTarget.y);
            double tileDistance = this.getDistanceTo(tileTarget.x, tileTarget.y);
            if (holeDistance <= tileDistance) {
                rememberIntention(TYPE_HOLE, holeTarget.x, holeTarget.y);
                return new TWThought(TWAction.MOVE, nextStepToward(holeTarget.x, holeTarget.y));
            }
            rememberIntention(TYPE_TILE, tileTarget.x, tileTarget.y);
            return new TWThought(TWAction.MOVE, nextStepToward(tileTarget.x, tileTarget.y));
        }
        if (holeTarget != null) {
            rememberIntention(TYPE_HOLE, holeTarget.x, holeTarget.y);
            return new TWThought(TWAction.MOVE, nextStepToward(holeTarget.x, holeTarget.y));
        }
        if (tileTarget != null) {
            rememberIntention(TYPE_TILE, tileTarget.x, tileTarget.y);
            return new TWThought(TWAction.MOVE, nextStepToward(tileTarget.x, tileTarget.y));
        }

        clearIntention();
        return new TWThought(TWAction.MOVE, getRandomDirection());
    }

    @Override
    protected void act(TWThought thought) {
        try {
            if (thought.getAction() == TWAction.MOVE) {
                this.move(thought.getDirection());
            } else if (thought.getAction() == TWAction.PICKUP) {
                TWEntity e = (TWEntity) this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());
                if (e instanceof TWTile && this.carriedTiles.size() < 3) {
                    this.pickUpTile((TWTile) e);
                    this.getMemory().removeObject(e);
                    knownTiles.remove(asKey(e.getX(), e.getY()));
                }
            } else if (thought.getAction() == TWAction.PUTDOWN) {
                TWEntity e = (TWEntity) this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());
                if (e instanceof TWHole && this.hasTile()) {
                    this.putTileInHole((TWHole) e);
                    this.getMemory().removeObject(e);
                    knownHoles.remove(asKey(e.getX(), e.getY()));
                }
            } else if (thought.getAction() == TWAction.REFUEL) {
                this.refuel();
            }
        } catch (CellBlockedException ex) {
            // blocked cell, next think step will replan
        }
    }

    @Override
    public void communicate() {
        TWEntity seenTile = this.getMemory().getClosestObjectInSensorRange(TWTile.class);
        if (seenTile != null) {
            this.getEnvironment().receiveMessage(
                    ArdaMessage.info(this.getName(), TYPE_TILE, seenTile.getX(), seenTile.getY(), this.getX(), this.getY()));
        }

        TWEntity seenHole = this.getMemory().getClosestObjectInSensorRange(TWHole.class);
        if (seenHole != null) {
            this.getEnvironment().receiveMessage(
                    ArdaMessage.info(this.getName(), TYPE_HOLE, seenHole.getX(), seenHole.getY(), this.getX(), this.getY()));
        }

        locateFuelStation();
        if (fuelX >= 0 && fuelY >= 0) {
            this.getEnvironment().receiveMessage(
                    ArdaMessage.info(this.getName(), TYPE_FUEL, fuelX, fuelY, this.getX(), this.getY()));
        }

        if (!"".equals(intendedType) && intendedX >= 0 && intendedY >= 0) {
            this.getEnvironment().receiveMessage(
                    ArdaMessage.intention(this.getName(), intendedType, intendedX, intendedY, this.getX(), this.getY()));
        }
    }

    private void ingestMessages() {
        receivedIntentions.clear();
        for (Message rawMessage : this.getEnvironment().getMessages()) {
            if (!(rawMessage instanceof ArdaMessage)) {
                continue;
            }
            ArdaMessage message = (ArdaMessage) rawMessage;
            if (this.getName().equals(message.getFrom())) {
                continue;
            }

            if (message.getType() == ArdaMessage.MessageType.INFO) {
                updateKnowledge(message.getEntityType(), message.getX(), message.getY());
            } else if (message.getType() == ArdaMessage.MessageType.INTENTION) {
                receivedIntentions.add(new IntentInfo(
                        message.getFrom(),
                        message.getEntityType(),
                        message.getX(),
                        message.getY(),
                        message.getSenderX(),
                        message.getSenderY()));
            }
        }
    }

    private void refreshKnowledge() {
        TWTile tileFromMemory = this.getMemory().getNearbyTile(this.getX(), this.getY(), Double.MAX_VALUE);
        if (tileFromMemory != null) {
            updateKnowledge(TYPE_TILE, tileFromMemory.getX(), tileFromMemory.getY());
        }

        TWHole holeFromMemory = this.getMemory().getNearbyHole(this.getX(), this.getY(), Double.MAX_VALUE);
        if (holeFromMemory != null) {
            updateKnowledge(TYPE_HOLE, holeFromMemory.getX(), holeFromMemory.getY());
        }

        locateFuelStation();
        pruneInvalidKnowledge();
    }

    private void locateFuelStation() {
        if (fuelX >= 0 && fuelY >= 0) {
            return;
        }

        int maxX = this.getEnvironment().getxDimension();
        int maxY = this.getEnvironment().getyDimension();
        for (int x = 0; x < maxX; x++) {
            for (int y = 0; y < maxY; y++) {
                Object obj = this.getEnvironment().getObjectGrid().get(x, y);
                if (obj instanceof TWFuelStation) {
                    fuelX = x;
                    fuelY = y;
                    return;
                }
            }
        }
    }

    private void pruneInvalidKnowledge() {
        pruneByType(knownTiles, TYPE_TILE);
        pruneByType(knownHoles, TYPE_HOLE);
    }

    private void pruneByType(Map<String, SharedLocation> map, String type) {
        Iterator<Map.Entry<String, SharedLocation>> iterator = map.entrySet().iterator();
        while (iterator.hasNext()) {
            Map.Entry<String, SharedLocation> entry = iterator.next();
            if (!isValidEntityAt(type, entry.getValue().x, entry.getValue().y)) {
                iterator.remove();
            }
        }
    }

    private boolean isValidEntityAt(String type, int x, int y) {
        Object obj = this.getEnvironment().getObjectGrid().get(x, y);
        if (TYPE_TILE.equals(type)) {
            return obj instanceof TWTile;
        }
        if (TYPE_HOLE.equals(type)) {
            return obj instanceof TWHole;
        }
        if (TYPE_FUEL.equals(type)) {
            return obj instanceof TWFuelStation;
        }
        return false;
    }

    private SharedLocation selectClosestTarget(String type, Map<String, SharedLocation> map) {
        SharedLocation best = null;
        double bestDistance = Double.MAX_VALUE;

        for (SharedLocation candidate : map.values()) {
            if (!isValidEntityAt(type, candidate.x, candidate.y)) {
                continue;
            }
            if (isClaimedByCloserAgent(type, candidate.x, candidate.y)) {
                continue;
            }

            double distance = this.getDistanceTo(candidate.x, candidate.y);
            if (distance < bestDistance) {
                bestDistance = distance;
                best = candidate;
            }
        }

        return best;
    }

    private boolean isClaimedByCloserAgent(String type, int x, int y) {
        double myDistance = this.getDistanceTo(x, y);
        for (IntentInfo intent : receivedIntentions) {
            if (!type.equals(intent.entityType)) {
                continue;
            }
            if (intent.targetX != x || intent.targetY != y) {
                continue;
            }

            double otherDistance = Math.abs(intent.targetX - intent.senderX) + Math.abs(intent.targetY - intent.senderY);
            if (otherDistance < myDistance) {
                return true;
            }
            if (otherDistance == myDistance && intent.agentName.compareTo(this.getName()) < 0) {
                return true;
            }
        }
        return false;
    }

    private void updateKnowledge(String entityType, int x, int y) {
        if (x < 0 || y < 0) {
            return;
        }

        SharedLocation location = new SharedLocation(x, y);
        if (TYPE_TILE.equals(entityType)) {
            knownTiles.put(asKey(x, y), location);
        } else if (TYPE_HOLE.equals(entityType)) {
            knownHoles.put(asKey(x, y), location);
        } else if (TYPE_FUEL.equals(entityType)) {
            fuelX = x;
            fuelY = y;
        }
    }

    private String asKey(int x, int y) {
        return x + ":" + y;
    }

    private boolean shouldRefuel() {
        return this.getFuelLevel() <= (Parameters.defaultFuelLevel * FUEL_THRESHOLD_RATIO);
    }

    private void rememberIntention(String type, int x, int y) {
        intendedType = type;
        intendedX = x;
        intendedY = y;
    }

    private void clearIntention() {
        intendedType = "";
        intendedX = -1;
        intendedY = -1;
    }

    private TWDirection nextStepToward(int tx, int ty) {
        if (this.getX() == tx && this.getY() == ty) {
            return TWDirection.Z;
        }

        TWPath path = pathGenerator.findPath(this.getX(), this.getY(), tx, ty);
        if (path != null) {
            while (path.hasNext()) {
                TWPathStep step = path.popNext();
                if (step.getDirection() != TWDirection.Z) {
                    return step.getDirection();
                }
            }
            return TWDirection.Z;
        }

        List<TWDirection> candidates = prioritizedDirections(tx, ty);
        for (TWDirection direction : candidates) {
            int nx = this.getX() + direction.dx;
            int ny = this.getY() + direction.dy;
            if (!this.getEnvironment().isCellBlocked(nx, ny)) {
                return direction;
            }
        }

        return getRandomDirection();
    }

    private List<TWDirection> prioritizedDirections(int tx, int ty) {
        List<TWDirection> ordered = new ArrayList<TWDirection>(4);
        int dx = tx - this.getX();
        int dy = ty - this.getY();

        if (Math.abs(dx) >= Math.abs(dy)) {
            if (dx > 0) {
                ordered.add(TWDirection.E);
            } else if (dx < 0) {
                ordered.add(TWDirection.W);
            }
            if (dy > 0) {
                ordered.add(TWDirection.S);
            } else if (dy < 0) {
                ordered.add(TWDirection.N);
            }
        } else {
            if (dy > 0) {
                ordered.add(TWDirection.S);
            } else if (dy < 0) {
                ordered.add(TWDirection.N);
            }
            if (dx > 0) {
                ordered.add(TWDirection.E);
            } else if (dx < 0) {
                ordered.add(TWDirection.W);
            }
        }

        for (TWDirection direction : new TWDirection[] {TWDirection.E, TWDirection.W, TWDirection.N, TWDirection.S}) {
            if (!ordered.contains(direction)) {
                ordered.add(direction);
            }
        }

        return ordered;
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

    @Override
    public String getName() {
        return name;
    }
}
