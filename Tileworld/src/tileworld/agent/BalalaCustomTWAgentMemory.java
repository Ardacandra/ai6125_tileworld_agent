package tileworld.agent;

import java.util.ArrayList;
import java.util.List;
import sim.engine.Schedule;
import sim.util.Bag;
import sim.util.IntBag;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;

/**
 * BalalaCustomTWAgentMemory
 *
 * Maintains two sets of lists:
 *
 *   PERSONAL lists — objects this agent directly sensed within its
 *   own sensor range. These are broadcast to teammates each step.
 *   Broadcasting only personal observations breaks the circular
 *   re-broadcast loop where agents re-share injected data.
 *
 *   SHARED lists — objects injected from teammate broadcasts.
 *   These are merged into the combined getKnownTiles()/getKnownHoles()
 *   lists which drive targeting. Only fresh injections (within
 *   INJECT_MAX_AGE steps) are accepted to avoid stale phantom targets.
 *
 * Combined lists = personal + shared, deduplicated by position.
 */
public class BalalaCustomTWAgentMemory extends TWAgentWorkingMemory {

    public static class MemoryEntry {
        public final int    x;
        public final int    y;
        public final double timestamp;
        public double       utility;

        public MemoryEntry(int x, int y, double timestamp, double utility) {
            this.x         = x;
            this.y         = y;
            this.timestamp = timestamp;
            this.utility   = utility;
        }
    }

    private static final double W_RECENCY      = 0.4;
    private static final double W_PROXIMITY    = 0.6;
    private static final double MAX_DIST       = 150.0;

    /**
     * Only accept teammate broadcasts this many steps old or newer.
     * Config 2 lifetime=30, so 10 is safe. Config 1 lifetime=100,
     * also fine. Prevents stale phantom objects clogging memory.
     */
    private static final double INJECT_MAX_AGE = 10.0;

    private final TWAgent  owner;
    private final Schedule schedule;

    // Objects this agent personally sensed — broadcast to teammates
    private final List<MemoryEntry> personalTiles = new ArrayList<MemoryEntry>();
    private final List<MemoryEntry> personalHoles = new ArrayList<MemoryEntry>();

    // Objects injected from teammate broadcasts — NOT re-broadcast
    private final List<MemoryEntry> sharedTiles = new ArrayList<MemoryEntry>();
    private final List<MemoryEntry> sharedHoles = new ArrayList<MemoryEntry>();

    public BalalaCustomTWAgentMemory(TWAgent owner, Schedule schedule,
                                     int xDim, int yDim) {
        super(owner, schedule, xDim, yDim);
        this.owner    = owner;
        this.schedule = schedule;
    }

    // ---------------------------------------------------------------
    // Direct sensing — populates personal lists only
    // ---------------------------------------------------------------
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
                upsert(personalTiles, objectXCoords.get(i),
                                      objectYCoords.get(i), now);
            } else if (obj instanceof TWHole) {
                upsert(personalHoles, objectXCoords.get(i),
                                      objectYCoords.get(i), now);
            }
        }

        recomputeUtilities(personalTiles, now);
        recomputeUtilities(personalHoles, now);
        recomputeUtilities(sharedTiles,   now);
        recomputeUtilities(sharedHoles,   now);
    }

    // ---------------------------------------------------------------
    // Teammate injection — only into shared lists, staleness filtered
    // ---------------------------------------------------------------

    /**
     * Inject a tile seen by a teammate. Only accepted if the
     * timestamp is within INJECT_MAX_AGE steps of now.
     * Goes into sharedTiles — NOT re-broadcast to avoid loops.
     */
    public void injectTile(int x, int y, double timestamp) {
        double now = schedule.getTime();
        if (now - timestamp > INJECT_MAX_AGE) return; // stale — ignore
        upsert(sharedTiles, x, y, timestamp);
    }

    /**
     * Inject a hole seen by a teammate. Only accepted if the
     * timestamp is within INJECT_MAX_AGE steps of now.
     */
    public void injectHole(int x, int y, double timestamp) {
        double now = schedule.getTime();
        if (now - timestamp > INJECT_MAX_AGE) return; // stale — ignore
        upsert(sharedHoles, x, y, timestamp);
    }

    // ---------------------------------------------------------------
    // Combined accessors — personal + shared merged
    // ---------------------------------------------------------------

    /**
     * Returns all known tiles (personal + shared, deduplicated).
     * Personal entries take priority over shared for same position.
     */
    public List<MemoryEntry> getKnownTiles() {
        return merged(personalTiles, sharedTiles);
    }

    /**
     * Returns all known holes (personal + shared, deduplicated).
     */
    public List<MemoryEntry> getKnownHoles() {
        return merged(personalHoles, sharedHoles);
    }

    /**
     * Returns ONLY personally sensed tiles — for broadcasting.
     * Agents must only broadcast what they personally saw to
     * prevent circular re-broadcasting of injected data.
     */
    public List<MemoryEntry> getPersonalTiles() { return personalTiles; }

    /**
     * Returns ONLY personally sensed holes — for broadcasting.
     */
    public List<MemoryEntry> getPersonalHoles() { return personalHoles; }

    // ---------------------------------------------------------------
    // Remove — cleans both personal and shared lists
    // ---------------------------------------------------------------
    public void removeTile(int x, int y) {
        removeByPosition(personalTiles, x, y);
        removeByPosition(sharedTiles,   x, y);
        removeAgentPercept(x, y);
    }

    public void removeHole(int x, int y) {
        removeByPosition(personalHoles, x, y);
        removeByPosition(sharedHoles,   x, y);
        removeAgentPercept(x, y);
    }

    // ---------------------------------------------------------------
    // Private helpers
    // ---------------------------------------------------------------

    private List<MemoryEntry> merged(List<MemoryEntry> personal,
                                     List<MemoryEntry> shared) {
        List<MemoryEntry> result = new ArrayList<MemoryEntry>(personal);
        for (int i = 0; i < shared.size(); i++) {
            MemoryEntry s = shared.get(i);
            boolean found = false;
            for (int j = 0; j < personal.size(); j++) {
                if (personal.get(j).x == s.x && personal.get(j).y == s.y) {
                    found = true;
                    break;
                }
            }
            if (!found) result.add(s);
        }
        return result;
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
            MemoryEntry e    = list.get(i);
            double age       = now - e.timestamp;
            double recency   = 1.0 / (1.0 + age);
            double dist      = Math.abs(ax - e.x) + Math.abs(ay - e.y);
            double proximity = Math.max(0.0, 1.0 - dist / MAX_DIST);
            e.utility = W_RECENCY * recency + W_PROXIMITY * proximity;
        }
    }

    private void removeByPosition(List<MemoryEntry> list, int x, int y) {
        for (int i = list.size() - 1; i >= 0; i--) {
            MemoryEntry e = list.get(i);
            if (e.x == x && e.y == y) list.remove(i);
        }
    }
}
