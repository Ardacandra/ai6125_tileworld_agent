package tileworld.agent;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import sim.engine.Schedule;
import sim.util.Bag;
import sim.util.IntBag;
import tileworld.Parameters;
import tileworld.environment.TWHole;
import tileworld.environment.TWObject;
import tileworld.environment.TWObstacle;
import tileworld.environment.TWTile;

/**
 * ArdaCustomTWAgentMemory
 *
 * Extends TWAgentWorkingMemory with structured memory sets for tiles, holes,
 * and obstacles, plus light utility scoring for target selection.
 */
public class ArdaCustomTWAgentMemory extends TWAgentWorkingMemory {

    public static class MemoryEntry {
        public final int x;
        public final int y;
        public final double observedAt;
        public final double utility;

        public MemoryEntry(int x, int y, double observedAt, double utility) {
            this.x = x;
            this.y = y;
            this.observedAt = observedAt;
            this.utility = utility;
        }
    }

    private static final double MEMORY_HORIZON = Parameters.lifeTime * 0.8;

    // Value layout: [observedAt, x, y]
    private final Map<Integer, double[]> trackedTiles = new HashMap<Integer, double[]>();
    private final Map<Integer, double[]> trackedHoles = new HashMap<Integer, double[]>();
    private final Map<Integer, double[]> knownObstacles = new HashMap<Integer, double[]>();

    private final TWAgent owner;
    private final Schedule schedule;
    private final int width;

    public ArdaCustomTWAgentMemory(TWAgent owner, Schedule schedule, int xDim, int yDim) {
        super(owner, schedule, xDim, yDim);
        this.owner = owner;
        this.schedule = schedule;
        this.width = xDim;
    }

    @Override
    public void updateMemory(Bag sensedObjects,
                             IntBag objectXCoords,
                             IntBag objectYCoords,
                             Bag sensedAgents,
                             IntBag agentXCoords,
                             IntBag agentYCoords) {

        super.updateMemory(sensedObjects, objectXCoords, objectYCoords,
                sensedAgents, agentXCoords, agentYCoords);

        double now = schedule.getTime();

        for (int i = 0; i < sensedObjects.size(); i++) {
            Object sensed = sensedObjects.get(i);
            if (!(sensed instanceof TWObject)) {
                continue;
            }

            TWObject object = (TWObject) sensed;
            int key = pack(object.getX(), object.getY());

            if (object instanceof TWObstacle) {
                knownObstacles.put(key, new double[] { now, object.getX(), object.getY() });
            } else if (object instanceof TWTile) {
                trackedTiles.put(key, new double[] { now, object.getX(), object.getY() });
            } else if (object instanceof TWHole) {
                trackedHoles.put(key, new double[] { now, object.getX(), object.getY() });
            }
        }

        decayMemory();
    }

    @Override
    public void decayMemory() {
        double now = schedule.getTime();

        Iterator<Map.Entry<Integer, double[]>> tileIterator = trackedTiles.entrySet().iterator();
        while (tileIterator.hasNext()) {
            Map.Entry<Integer, double[]> entry = tileIterator.next();
            double[] value = entry.getValue();
            int x = (int) value[1];
            int y = (int) value[2];

            boolean stale = (now - value[0]) > MEMORY_HORIZON;

            if (stale) {
                removeAgentPercept(x, y);
                tileIterator.remove();
            }
        }

        Iterator<Map.Entry<Integer, double[]>> holeIterator = trackedHoles.entrySet().iterator();
        while (holeIterator.hasNext()) {
            Map.Entry<Integer, double[]> entry = holeIterator.next();
            double[] value = entry.getValue();
            int x = (int) value[1];
            int y = (int) value[2];

            boolean stale = (now - value[0]) > MEMORY_HORIZON;

            if (stale) {
                removeAgentPercept(x, y);
                holeIterator.remove();
            }
        }

        Iterator<Map.Entry<Integer, double[]>> obstacleIterator = knownObstacles.entrySet().iterator();
        while (obstacleIterator.hasNext()) {
            Map.Entry<Integer, double[]> entry = obstacleIterator.next();
            double[] obstacle = entry.getValue();
            int x = (int) obstacle[1];
            int y = (int) obstacle[2];
            boolean stale = (now - obstacle[0]) > MEMORY_HORIZON;
            if (stale) {
                removeAgentPercept(x, y);
                obstacleIterator.remove();
            }
        }
    }

    public List<MemoryEntry> getKnownTiles() {
        return buildEntries(trackedTiles);
    }

    public List<MemoryEntry> getKnownHoles() {
        return buildEntries(trackedHoles);
    }

    public void removeTile(int x, int y) {
        trackedTiles.remove(pack(x, y));
        removeAgentPercept(x, y);
    }

    public void removeHole(int x, int y) {
        trackedHoles.remove(pack(x, y));
        removeAgentPercept(x, y);
    }

    @Override
    public boolean isCellBlocked(int tx, int ty) {
        if (super.isCellBlocked(tx, ty)) {
            return true;
        }
        return knownObstacles.containsKey(pack(tx, ty));
    }

    private List<MemoryEntry> buildEntries(Map<Integer, double[]> source) {
        List<MemoryEntry> entries = new ArrayList<MemoryEntry>();

        int ax = owner.getX();
        int ay = owner.getY();
        double now = schedule.getTime();

        for (double[] value : source.values()) {
            int x = (int) value[1];
            int y = (int) value[2];
            double observedAt = value[0];

            double age = now - observedAt;
            double freshness = Math.max(0.0, 1.0 - age / MEMORY_HORIZON);
            double distance = Math.abs(ax - x) + Math.abs(ay - y);
            double utility = freshness / (distance + 1.0);

            entries.add(new MemoryEntry(x, y, observedAt, utility));
        }

        return entries;
    }

    private int pack(int x, int y) {
        return y * width + x;
    }
}
