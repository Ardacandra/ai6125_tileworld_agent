package tileworld.agent;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import sim.engine.Schedule;
import sim.field.grid.ObjectGrid2D;
import tileworld.Parameters;
import tileworld.environment.TWEntity;
import tileworld.environment.TWHole;
import tileworld.environment.TWObject;
import tileworld.environment.TWObstacle;
import tileworld.environment.TWTile;

public class AmeyaCustomTWAgentMemory extends TWAgentWorkingMemory {


    private static final double TILE_HOLE_MEMORY_STEPS = Parameters.lifeTime * 0.8;


    public static class MemoryEntry {
        public final int x;
        public final int y;
        public final double observedAt;  // simulation time last seen
        public final double utility;     // higher = better target

        public MemoryEntry(int x, int y, double observedAt, double utility) {
            this.x = x;
            this.y = y;
            this.observedAt = observedAt;
            this.utility = utility;
        }
    }


    protected final Map<Integer, double[]> trackedTiles  = new HashMap<>();
    protected final Map<Integer, double[]> trackedHoles  = new HashMap<>();

 
    private final Map<Integer, int[]> knownObstacles = new HashMap<>();

    protected final int width;
    protected final int height;
    protected final Schedule schedule;
    protected final TWAgent me;


    public AmeyaCustomTWAgentMemory(TWAgent agent, Schedule schedule, int x, int y) {
        super(agent, schedule, x, y);
        this.me       = agent;
        this.schedule = schedule;
        this.width    = x;
        this.height   = y;
    }


    @Override
    public void updateMemory(sim.util.Bag sensedObjects,
                              sim.util.IntBag objectXCoords,
                              sim.util.IntBag objectYCoords,
                              sim.util.Bag sensedAgents,
                              sim.util.IntBag agentXCoords,
                              sim.util.IntBag agentYCoords) {

        super.updateMemory(sensedObjects, objectXCoords, objectYCoords,
                sensedAgents, agentXCoords, agentYCoords);

        double now = schedule.getTime();

        for (int i = 0; i < sensedObjects.size(); i++) {
            Object obj = sensedObjects.get(i);
            if (!(obj instanceof TWObject)) continue;

            TWObject o = (TWObject) obj;
            int key = pack(o.getX(), o.getY());

            if (o instanceof TWObstacle) {
                knownObstacles.put(key, new int[]{o.getX(), o.getY()});
            } else if (o instanceof TWTile) {
                // [observedTime, x, y]
                trackedTiles.put(key, new double[]{now, o.getX(), o.getY()});
            } else if (o instanceof TWHole) {
                trackedHoles.put(key, new double[]{now, o.getX(), o.getY()});
            }
        }

        decayMemory();
    }


    @Override
    public void decayMemory() {
        double now = schedule.getTime();
        ObjectGrid2D grid = me.getEnvironment().getObjectGrid();

        Iterator<Map.Entry<Integer, double[]>> tit = trackedTiles.entrySet().iterator();
        while (tit.hasNext()) {
            Map.Entry<Integer, double[]> entry = tit.next();
            double[] val = entry.getValue();
            int x = (int) val[1];
            int y = (int) val[2];

            boolean tooOld  = (now - val[0]) > TILE_HOLE_MEMORY_STEPS;
            boolean gone    = !(grid.get(x, y) instanceof TWTile);

            if (tooOld || gone) {
                removeAgentPercept(x, y);
                tit.remove();
            }
        }

        Iterator<Map.Entry<Integer, double[]>> hit = trackedHoles.entrySet().iterator();
        while (hit.hasNext()) {
            Map.Entry<Integer, double[]> entry = hit.next();
            double[] val = entry.getValue();
            int x = (int) val[1];
            int y = (int) val[2];

            boolean tooOld = (now - val[0]) > TILE_HOLE_MEMORY_STEPS;
            boolean gone   = !(grid.get(x, y) instanceof TWHole);

            if (tooOld || gone) {
                removeAgentPercept(x, y);
                hit.remove();
            }
        }

        Iterator<Map.Entry<Integer, int[]>> oit = knownObstacles.entrySet().iterator();
        while (oit.hasNext()) {
            Map.Entry<Integer, int[]> entry = oit.next();
            int ox = entry.getValue()[0];
            int oy = entry.getValue()[1];
            if (!me.getEnvironment().isCellBlocked(ox, oy)) {
                removeAgentPercept(ox, oy);
                oit.remove();
            }
        }
    }

    public List<MemoryEntry> getKnownTiles() {
        return buildEntries(trackedTiles);
    }

    public List<MemoryEntry> getKnownHoles() {
        return buildEntries(trackedHoles);
    }

    private List<MemoryEntry> buildEntries(Map<Integer, double[]> map) {
        List<MemoryEntry> entries = new ArrayList<>();
        double now = schedule.getTime();
        int ax = me.getX();
        int ay = me.getY();

        for (double[] val : map.values()) {
            int x = (int) val[1];
            int y = (int) val[2];
            double observedAt = val[0];

            double age = now - observedAt;
            double freshnessFactor = Math.max(0.0,
                    1.0 - (age / TILE_HOLE_MEMORY_STEPS));

            double distance = Math.abs(ax - x) + Math.abs(ay - y);
            double utility  = (1.0 / (distance + 1.0)) * freshnessFactor;

            entries.add(new MemoryEntry(x, y, observedAt, utility));
        }

        return entries;
    }


    @Override
    public boolean isCellBlocked(int tx, int ty) {
        if (super.isCellBlocked(tx, ty)) return true;
        return knownObstacles.containsKey(pack(tx, ty));
    }

    protected void addTrackedTile(int x, int y, double timestamp) {
        trackedTiles.put(pack(x, y), new double[]{timestamp, x, y});
    }

    protected void addTrackedHole(int x, int y, double timestamp) {
        trackedHoles.put(pack(x, y), new double[]{timestamp, x, y});
    }

    protected void removeTrackedTile(int x, int y) {
        trackedTiles.remove(pack(x, y));
    }

    protected void removeTrackedHole(int x, int y) {
        trackedHoles.remove(pack(x, y));
    }

    protected TWAgent getAgent() {
        return me;
    }

    protected int pack(int x, int y)  { return y * width + x; }
    protected int unpackX(int packed) { return packed % width; }
    protected int unpackY(int packed) { return packed / width; }
}
