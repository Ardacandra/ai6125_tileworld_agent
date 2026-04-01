package tileworld.agent;

import java.util.ArrayList;
import java.util.List;
import sim.engine.Schedule;
import sim.util.Bag;
import sim.util.IntBag;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;

/**
 * CustomTWAgentMemory
 *
 * Extends TWAgentWorkingMemory with scored lists of known tiles and
 * holes. Utility = recency * 0.4 + proximity * 0.6.
 */
public class BalalaCustomTWAgentMemory extends TWAgentWorkingMemory {

    public static class MemoryEntry {
        public final int    x;
        public final int    y;
        public final double timestamp;
        public double       utility; // mutable

        public MemoryEntry(int x, int y, double timestamp, double utility) {
            this.x         = x;
            this.y         = y;
            this.timestamp = timestamp;
            this.utility   = utility;
        }
    }

    private static final double W_RECENCY   = 0.4;
    private static final double W_PROXIMITY = 0.6;
    private static final double MAX_DIST    = 150.0;

    private final TWAgent  owner;
    private final Schedule schedule;

    private final List<MemoryEntry> knownTiles = new ArrayList<MemoryEntry>();
    private final List<MemoryEntry> knownHoles = new ArrayList<MemoryEntry>();

    public BalalaCustomTWAgentMemory(TWAgent owner, Schedule schedule,
                               int xDim, int yDim) {
        super(owner, schedule, xDim, yDim);
        this.owner    = owner;
        this.schedule = schedule;
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
            Object obj = sensedObjects.get(i);
            if (obj instanceof TWTile) {
                upsert(knownTiles, objectXCoords.get(i),
                                   objectYCoords.get(i), now);
            } else if (obj instanceof TWHole) {
                upsert(knownHoles, objectXCoords.get(i),
                                   objectYCoords.get(i), now);
            }
        }

        recomputeUtilities(knownTiles, now);
        recomputeUtilities(knownHoles, now);
    }

    public List<MemoryEntry> getKnownTiles() { return knownTiles; }
    public List<MemoryEntry> getKnownHoles() { return knownHoles; }

    public void removeTile(int x, int y) {
        removeByPosition(knownTiles, x, y);
        removeAgentPercept(x, y);
    }

    public void removeHole(int x, int y) {
        removeByPosition(knownHoles, x, y);
        removeAgentPercept(x, y);
    }

    private void upsert(List<MemoryEntry> list, int x, int y, double now) {
        for (int i = 0; i < list.size(); i++) {
            MemoryEntry e = list.get(i);
            if (e.x == x && e.y == y) {
                list.set(i, new MemoryEntry(x, y, now, e.utility));
                return;
            }
        }
        list.add(new MemoryEntry(x, y, now, 0.0));
    }

    private void recomputeUtilities(List<MemoryEntry> list, double now) {
        int ax = owner.getX();
        int ay = owner.getY();
        for (int i = 0; i < list.size(); i++) {
            MemoryEntry e   = list.get(i);
            double age      = now - e.timestamp;
            double recency  = 1.0 / (1.0 + age);
            double dist     = Math.abs(ax - e.x) + Math.abs(ay - e.y);
            double proximity = Math.max(0.0, 1.0 - dist / MAX_DIST);
            e.utility = W_RECENCY * recency + W_PROXIMITY * proximity;
        }
    }

    private void removeByPosition(List<MemoryEntry> list, int x, int y) {
        for (int i = list.size() - 1; i >= 0; i--) {
            MemoryEntry e = list.get(i);
            if (e.x == x && e.y == y) {
                list.remove(i);
            }
        }
    }
}