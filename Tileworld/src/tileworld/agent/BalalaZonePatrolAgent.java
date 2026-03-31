package tileworld.agent;

import java.util.ArrayList;
import java.util.List;
import sim.util.Int2D;
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

/**
 * ZonePatrolAgent — Final merged version
 *
 * ── PHASE 1 (via Phase1Strategy) ─────────────────────────────────
 * - Map divided into 6 zones (2 cols x 3 rows)
 * - Each agent claims the zone nearest its spawn — no overlap
 * - Boustrophedon sweep per zone with sensor-range waypoints
 * - First agent to sense fuel station broadcasts via ArdaMessage
 * - All agents switch to Phase 2 the moment station is known
 * - Opportunistic pickup/putdown during sweep — no reward wasted
 *
 * ── PHASE 1 TIMEOUT ──────────────────────────────────────────────
 * If Phase 1 has not found the fuel station within PHASE1_TIMEOUT
 * steps, it is force-completed and Phase 2 begins immediately.
 * This handles the solo case (1 agent covering only 1/6 of map)
 * and any case where the station is not in the assigned zone.
 * Phase 2's full-map sweep will eventually find the station.
 *
 * ── PHASE 2 (ZonePatrolAgent logic) ──────────────────────────────
 * - Full map boustrophedon sweep, SWEEP_STEP jumps, starts at spawn
 * - Scored memory targeting: 0.25*recency + 0.75*proximity
 * - Stale hole validation — removes expired holes from memory
 * - Time-aware fuel margin: 40 steps early → 15 steps late
 * - Position broadcast via ArdaMessage ("pos" entity type)
 * - Proximity yield: skip targets a closer capable teammate will reach
 *
 * ── FUEL EMERGENCY ───────────────────────────────────────────────
 * Global emergency check runs BEFORE all phase logic.
 * Uses EMERGENCY_BUFFER (60 steps) above distance to station.
 * When station unknown, triggers at EMERGENCY_UNKNOWN (45%) of fuel.
 *
 * ── RULES RESPECTED ──────────────────────────────────────────────
 * - Extends TWAgent only
 * - Overrides only communicate(), think(), act()
 * - Does NOT call increaseReward() directly
 * - Does NOT modify the environment package
 * - Fuel station discovered only within sensor range (spec compliant)
 */
public class BalalaZonePatrolAgent extends TWAgent {

    // ---------------------------------------------------------------
    // Constants
    // ---------------------------------------------------------------

    /**
     * Maximum steps to spend in Phase 1 before forcing Phase 2.
     * Set to 300 — enough for a full zone sweep on a 50x50 map,
     * but short enough that Phase 2 gets most of the 5000 steps.
     * Tune lower (e.g. 200) if solo performance matters more.
     */
    private static final int    PHASE1_TIMEOUT     = 150;

    private static final double EMERGENCY_BUFFER   = 60.0;
    private static final double EMERGENCY_UNKNOWN  = 0.45;
    private static final double FUEL_SAFETY_EARLY  = 40.0;
    private static final double FUEL_SAFETY_LATE   = 15.0;
    private static final double FUEL_UNKNOWN_RATIO = 0.35;
    private static final int    CARRY_CAPACITY     = 3;
    private static final double W_RECENCY          = 0.25;
    private static final double W_PROXIMITY        = 0.75;
    private static final double HOLE_BIAS_MARGIN   = 4.0;
    private static final int    SWEEP_STEP         = Parameters.defaultSensorRange;
    private static final double EARLY_PHASE        = 0.20;
    private static final String ENTITY_POS         = "pos";

    // ---------------------------------------------------------------
    // Fields
    // ---------------------------------------------------------------
    private final String             name;
    private final AstarPathGenerator pathGenerator;
    private BalalaCustomTWAgentMemory      customMemory;
    private final Phase1Strategy     phase1;

    /** True when Phase 1 is done (found station OR timed out). */
    private boolean phase1Done = false;

    private int fuelStationX = -1;
    private int fuelStationY = -1;

    private final List<int[]> teammateSnapshots = new ArrayList<int[]>();

    // Phase 2 sweep — starts at spawn position
    private int     sweepCol;
    private int     sweepRow;
    private boolean sweepGoingDown;

    // ---------------------------------------------------------------
    // Constructor
    // ---------------------------------------------------------------
    public BalalaZonePatrolAgent(String name, int xpos, int ypos,
                           TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.name = name;

        this.customMemory = new BalalaCustomTWAgentMemory(
                this, env.schedule,
                env.getxDimension(), env.getyDimension());
        this.memory = customMemory;

        this.pathGenerator = new AstarPathGenerator(
                env, this,
                env.getxDimension() * env.getyDimension());

        this.phase1 = new Phase1Strategy(this);

        // Start Phase 2 sweep from spawn — no wasted travel to (0,0)
        this.sweepCol       = xpos;
        this.sweepRow       = ypos;
        this.sweepGoingDown = true;
    }

    // ---------------------------------------------------------------
    // communicate
    // ---------------------------------------------------------------
    @Override
    public void communicate() {
        // Always let Phase1 communicate (handles "init" + "fuel" types)
        phase1.communicate();

        // Only broadcast Phase 2 position once Phase 1 is done
        if (!phase1Done) return;

        getEnvironment().receiveMessage(
                ArdaMessage.info(name, ENTITY_POS,
                        getX(), getY(),
                        carriedTiles.size(), 0));
    }

    // ---------------------------------------------------------------
    // think
    // ---------------------------------------------------------------
    @Override
    protected TWThought think() {

        // Always scan sensor range for fuel station
        locateFuelStation();

        // Sync fuel station from Phase1 if it found it
        if (fuelStationX == -1 && phase1.getFuelStation() != null) {
            Int2D fs = phase1.getFuelStation();
            fuelStationX = fs.x;
            fuelStationY = fs.y;
        }

        // Check Phase1 timeout — force Phase 2 if time exceeded
        checkPhase1Timeout();

        int ax = getX();
        int ay = getY();

        // ── GLOBAL FUEL EMERGENCY ─────────────────────────────────
        // Runs BEFORE all phase logic — agent can never run dry
        if (isFuelEmergency(ax, ay)) {
            if (fuelStationX == -1) {
                // Unknown — head to map centre while searching
                return new TWThought(TWAction.MOVE,
                        pathTo(getEnvironment().getxDimension() / 2,
                               getEnvironment().getyDimension() / 2));
            }
            if (ax == fuelStationX && ay == fuelStationY) {
                return new TWThought(TWAction.REFUEL, TWDirection.Z);
            }
            return new TWThought(TWAction.MOVE,
                    pathTo(fuelStationX, fuelStationY));
        }

        // ── PHASE 1 ───────────────────────────────────────────────
        if (!phase1Done) {
            TWThought t = phase1.think();
            if (t != null) return t;
            // Phase 1 just completed naturally
            phase1Done = true;
            System.out.println(name + " Phase 1 complete — switching to Phase 2");
        }

        // ── PHASE 2 ───────────────────────────────────────────────
        return phase2Think(ax, ay);
    }

    // ---------------------------------------------------------------
    // Phase 1 timeout check
    // ---------------------------------------------------------------
    /**
     * Forces Phase 1 to complete if PHASE1_TIMEOUT steps have passed
     * without finding the fuel station. Phase 2's full-map sweep
     * will find it eventually.
     */
    private void checkPhase1Timeout() {
        if (phase1Done) return;
        double now = getEnvironment().schedule.getTime();
        if (now >= PHASE1_TIMEOUT) {
            phase1Done = true;
            System.out.println(name + " Phase 1 TIMEOUT at step " + (int) now
                    + " — switching to Phase 2 full-map sweep");
        }
        // Also mark done if Phase1Strategy itself completed
        if (phase1.isComplete()) {
            phase1Done = true;
        }
    }

    // ---------------------------------------------------------------
    // Phase 2 decision logic
    // ---------------------------------------------------------------
    private TWThought phase2Think(int ax, int ay) {

        readTeammateSnapshots();

        double  now        = getEnvironment().schedule.getTime();
        boolean earlyPhase = isEarlyPhase(now);

        // 1. Normal fuel check
        if (needsRefuel(ax, ay, now)) {
            if (fuelStationX == -1) {
                return new TWThought(TWAction.MOVE,
                        pathTo(getEnvironment().getxDimension() / 2,
                               getEnvironment().getyDimension() / 2));
            }
            if (ax == fuelStationX && ay == fuelStationY) {
                return new TWThought(TWAction.REFUEL, TWDirection.Z);
            }
            return new TWThought(TWAction.MOVE,
                    pathTo(fuelStationX, fuelStationY));
        }

        // 2. Opportunistic drop
        TWEntity onCell = (TWEntity) getEnvironment()
                .getObjectGrid().get(ax, ay);
        if (onCell instanceof TWHole && hasTile()) {
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }

        // 3. Opportunistic pickup
        if (onCell instanceof TWTile && carriedTiles.size() < CARRY_CAPACITY) {
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }

        // 4. Full carry → nearest valid unclaimed hole
        if (carriedTiles.size() == CARRY_CAPACITY) {
            BalalaCustomTWAgentMemory.MemoryEntry hole =
                    nearestValidYieldedHole(ax, ay);
            if (hole != null) {
                return new TWThought(TWAction.MOVE, pathTo(hole.x, hole.y));
            }
            return new TWThought(TWAction.MOVE, nextSweepDirection(ax, ay));
        }

        // 5. Scored target with yield check
        BalalaCustomTWAgentMemory.MemoryEntry target =
                selectTarget(ax, ay, earlyPhase);
        if (target != null) {
            return new TWThought(TWAction.MOVE, pathTo(target.x, target.y));
        }

        // 6. Sweep
        return new TWThought(TWAction.MOVE, nextSweepDirection(ax, ay));
    }

    // ---------------------------------------------------------------
    // act
    // ---------------------------------------------------------------
    @Override
    protected void act(TWThought thought) {
        try {
            switch (thought.getAction()) {
                case MOVE:
                    move(thought.getDirection());
                    break;
                case PICKUP:
                    TWTile tile = (TWTile) getEnvironment()
                            .getObjectGrid().get(getX(), getY());
                    if (tile != null) {
                        pickUpTile(tile);
                        customMemory.removeTile(getX(), getY());
                    }
                    break;
                case PUTDOWN:
                    TWHole hole = (TWHole) getEnvironment()
                            .getObjectGrid().get(getX(), getY());
                    if (hole != null) {
                        putTileInHole(hole);
                        customMemory.removeHole(getX(), getY());
                    }
                    break;
                case REFUEL:
                    refuel();
                    break;
                default:
                    break;
            }
        } catch (CellBlockedException e) {
            // A* replans next cycle
        }
    }

    // ---------------------------------------------------------------
    // FUEL STATION — sensor-range only (spec compliant)
    // ---------------------------------------------------------------
    private void locateFuelStation() {
        if (fuelStationX != -1) return;
        int range = Parameters.defaultSensorRange;
        for (int dx = -range; dx <= range; dx++) {
            for (int dy = -range; dy <= range; dy++) {
                int nx = getX() + dx;
                int ny = getY() + dy;
                if (!getEnvironment().isInBounds(nx, ny)) continue;
                TWEntity obj = (TWEntity) getEnvironment()
                        .getObjectGrid().get(nx, ny);
                if (obj instanceof TWFuelStation) {
                    fuelStationX = nx;
                    fuelStationY = ny;
                    return;
                }
            }
        }
    }

    // ---------------------------------------------------------------
    // FUEL MANAGEMENT
    // ---------------------------------------------------------------
    private boolean isFuelEmergency(int ax, int ay) {
        if (fuelStationX == -1) {
            return fuelLevel <= Parameters.defaultFuelLevel * EMERGENCY_UNKNOWN;
        }
        double dist = manhattan(ax, ay, fuelStationX, fuelStationY);
        return fuelLevel <= dist + EMERGENCY_BUFFER;
    }

    private boolean needsRefuel(int ax, int ay, double now) {
        if (fuelStationX == -1) {
            return fuelLevel <= Parameters.defaultFuelLevel * FUEL_UNKNOWN_RATIO;
        }
        double dist   = manhattan(ax, ay, fuelStationX, fuelStationY);
        double margin = computeFuelMargin(now);
        return fuelLevel <= dist + margin;
    }

    private double computeFuelMargin(double now) {
        double maxT    = Parameters.defaultFuelLevel * 10.0;
        double elapsed = Math.min(now / maxT, 1.0);
        return FUEL_SAFETY_EARLY
               - elapsed * (FUEL_SAFETY_EARLY - FUEL_SAFETY_LATE);
    }

    private boolean isEarlyPhase(double now) {
        double maxT = Parameters.defaultFuelLevel * 10.0;
        return (now / maxT) < EARLY_PHASE;
    }

    // ---------------------------------------------------------------
    // COMMUNICATION — ArdaMessage "pos" entity type
    // ---------------------------------------------------------------
    private void readTeammateSnapshots() {
        teammateSnapshots.clear();
        ArrayList<Message> messages = getEnvironment().getMessages();
        for (int i = 0; i < messages.size(); i++) {
            Message m = messages.get(i);
            if (m.getFrom().equals(name)) continue;
            if (!(m instanceof ArdaMessage)) continue;
            ArdaMessage am = (ArdaMessage) m;
            if (!ENTITY_POS.equals(am.getEntityType())) continue;
            teammateSnapshots.add(new int[]{
                am.getX(), am.getY(), am.getSenderX()
            });
        }
    }

    private boolean teammateCloserForTile(int tx, int ty, int myDist) {
        for (int i = 0; i < teammateSnapshots.size(); i++) {
            int[] s = teammateSnapshots.get(i);
            if (s[2] >= CARRY_CAPACITY) continue;
            if ((int) manhattan(s[0], s[1], tx, ty) < myDist) return true;
        }
        return false;
    }

    private boolean teammateCloserForHole(int tx, int ty, int myDist) {
        for (int i = 0; i < teammateSnapshots.size(); i++) {
            int[] s = teammateSnapshots.get(i);
            if (s[2] == 0) continue;
            if ((int) manhattan(s[0], s[1], tx, ty) < myDist) return true;
        }
        return false;
    }

    // ---------------------------------------------------------------
    // TARGET SELECTION
    // ---------------------------------------------------------------
    private BalalaCustomTWAgentMemory.MemoryEntry selectTarget(
            int ax, int ay, boolean earlyPhase) {

        List<BalalaCustomTWAgentMemory.MemoryEntry> tiles = customMemory.getKnownTiles();
        List<BalalaCustomTWAgentMemory.MemoryEntry> holes = customMemory.getKnownHoles();

        reScore(tiles, ax, ay);
        reScore(holes, ax, ay);

        BalalaCustomTWAgentMemory.MemoryEntry bestTile =
                bestYieldedEntry(tiles, true, ax, ay);
        BalalaCustomTWAgentMemory.MemoryEntry bestHole =
                bestYieldedValidEntry(holes, ax, ay);

        int carried = carriedTiles.size();

        if (earlyPhase && carried < 2 && bestTile != null) return bestTile;

        if (carried == 2) {
            if (bestHole == null) return bestTile;
            if (bestTile == null) return bestHole;
            double dTile = manhattan(ax, ay, bestTile.x, bestTile.y);
            double dHole = manhattan(ax, ay, bestHole.x, bestHole.y);
            return (dTile + HOLE_BIAS_MARGIN < dHole) ? bestTile : bestHole;
        }
        if (carried == 1) {
            if (bestTile == null) return bestHole;
            if (bestHole == null) return bestTile;
            return (bestTile.utility >= bestHole.utility) ? bestTile : bestHole;
        }
        return bestTile;
    }

    private BalalaCustomTWAgentMemory.MemoryEntry bestYieldedEntry(
            List<BalalaCustomTWAgentMemory.MemoryEntry> list,
            boolean isTile, int ax, int ay) {
        BalalaCustomTWAgentMemory.MemoryEntry best = null;
        for (int i = 0; i < list.size(); i++) {
            BalalaCustomTWAgentMemory.MemoryEntry e = list.get(i);
            int myDist = (int) manhattan(ax, ay, e.x, e.y);
            if (isTile  && teammateCloserForTile(e.x, e.y, myDist)) continue;
            if (!isTile && teammateCloserForHole(e.x, e.y, myDist)) continue;
            if (best == null || e.utility > best.utility) best = e;
        }
        return best;
    }

    private BalalaCustomTWAgentMemory.MemoryEntry bestYieldedValidEntry(
            List<BalalaCustomTWAgentMemory.MemoryEntry> list, int ax, int ay) {
        BalalaCustomTWAgentMemory.MemoryEntry best = null;
        List<BalalaCustomTWAgentMemory.MemoryEntry> stale =
                new ArrayList<BalalaCustomTWAgentMemory.MemoryEntry>();
        for (int i = 0; i < list.size(); i++) {
            BalalaCustomTWAgentMemory.MemoryEntry e = list.get(i);
            TWEntity obj = (TWEntity) getEnvironment()
                    .getObjectGrid().get(e.x, e.y);
            if (!(obj instanceof TWHole)) { stale.add(e); continue; }
            int myDist = (int) manhattan(ax, ay, e.x, e.y);
            if (teammateCloserForHole(e.x, e.y, myDist)) continue;
            if (best == null || e.utility > best.utility) best = e;
        }
        for (int i = 0; i < stale.size(); i++) {
            customMemory.removeHole(stale.get(i).x, stale.get(i).y);
        }
        return best;
    }

    private BalalaCustomTWAgentMemory.MemoryEntry nearestValidYieldedHole(
            int ax, int ay) {
        List<BalalaCustomTWAgentMemory.MemoryEntry> holes = customMemory.getKnownHoles();
        BalalaCustomTWAgentMemory.MemoryEntry nearest = null;
        double minDist = Double.MAX_VALUE;
        List<BalalaCustomTWAgentMemory.MemoryEntry> stale =
                new ArrayList<BalalaCustomTWAgentMemory.MemoryEntry>();
        for (int i = 0; i < holes.size(); i++) {
            BalalaCustomTWAgentMemory.MemoryEntry e = holes.get(i);
            TWEntity obj = (TWEntity) getEnvironment()
                    .getObjectGrid().get(e.x, e.y);
            if (!(obj instanceof TWHole)) { stale.add(e); continue; }
            int myDist = (int) manhattan(ax, ay, e.x, e.y);
            if (teammateCloserForHole(e.x, e.y, myDist)) continue;
            if (myDist < minDist) { minDist = myDist; nearest = e; }
        }
        for (int i = 0; i < stale.size(); i++) {
            customMemory.removeHole(stale.get(i).x, stale.get(i).y);
        }
        return nearest;
    }

    private void reScore(List<BalalaCustomTWAgentMemory.MemoryEntry> list,
                         int ax, int ay) {
        double now  = getEnvironment().schedule.getTime();
        double maxD = getEnvironment().getxDimension()
                    + getEnvironment().getyDimension();
        for (int i = 0; i < list.size(); i++) {
            BalalaCustomTWAgentMemory.MemoryEntry e = list.get(i);
            double age       = now - e.timestamp;
            double recency   = 1.0 / (1.0 + age);
            double dist      = manhattan(ax, ay, e.x, e.y);
            double proximity = Math.max(0.0, 1.0 - dist / maxD);
            e.utility = W_RECENCY * recency + W_PROXIMITY * proximity;
        }
    }

    // ---------------------------------------------------------------
    // BOUSTROPHEDON SWEEP (Phase 2)
    // ---------------------------------------------------------------
    private TWDirection nextSweepDirection(int ax, int ay) {
        int maxX = getEnvironment().getxDimension() - 1;
        int maxY = getEnvironment().getyDimension() - 1;
        sweepCol = Math.max(0, Math.min(maxX, sweepCol));
        sweepRow = Math.max(0, Math.min(maxY, sweepRow));
        if (manhattan(ax, ay, sweepCol, sweepRow) <= SWEEP_STEP) {
            advanceSweep(maxX, maxY);
        }
        TWDirection dir = pathTo(sweepCol, sweepRow);
        if (dir == TWDirection.Z) dir = randomUnblockedDirection();
        return dir;
    }

    private void advanceSweep(int maxX, int maxY) {
        if (sweepGoingDown) {
            sweepRow += SWEEP_STEP;
            if (sweepRow > maxY) {
                sweepRow = maxY; sweepCol += SWEEP_STEP; sweepGoingDown = false;
            }
        } else {
            sweepRow -= SWEEP_STEP;
            if (sweepRow < 0) {
                sweepRow = 0; sweepCol += SWEEP_STEP; sweepGoingDown = true;
            }
        }
        if (sweepCol > maxX) sweepCol = 0;
    }

    // ---------------------------------------------------------------
    // PATHFINDING
    // ---------------------------------------------------------------
    private TWDirection pathTo(int tx, int ty) {
        if (getX() == tx && getY() == ty) return TWDirection.Z;
        try {
            TWPath path = pathGenerator.findPath(getX(), getY(), tx, ty);
            if (path != null && path.hasNext()) {
                TWDirection d = path.popNext().getDirection();
                if (d != null && d != TWDirection.Z) return d;
            }
        } catch (Exception e) {
            // fall through
        }
        return randomUnblockedDirection();
    }

    private TWDirection randomUnblockedDirection() {
        TWDirection[] dirs = {
            TWDirection.N, TWDirection.S,
            TWDirection.E, TWDirection.W
        };
        int start = getEnvironment().random.nextInt(4);
        for (int i = 0; i < 4; i++) {
            TWDirection d = dirs[(start + i) % 4];
            int nx = getX() + d.dx;
            int ny = getY() + d.dy;
            if (getEnvironment().isInBounds(nx, ny)
                    && !getEnvironment().isCellBlocked(nx, ny)) {
                return d;
            }
        }
        return TWDirection.Z;
    }

    // ---------------------------------------------------------------
    // HELPERS
    // ---------------------------------------------------------------
    private double manhattan(int x1, int y1, int x2, int y2) {
        return Math.abs(x1 - x2) + Math.abs(y1 - y2);
    }

    @Override
    public String getName() { return name; }
}
