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
 * ZonePatrolAgent — Final balanced version
 *
 * Constants tuned to balance Config 1 and Config 2:
 *
 *   Config 1: sparse (mu=0.2), lifetime=100, 50x50
 *   Config 2: dense  (mu=2.0), lifetime=30,  80x80
 *   Config 3: unknown — balanced values are safest bet
 *
 * Constant derivations:
 *
 *     Config 1 wants ~150 (switch to full sweep sooner)
 *     Config 2 wants ~300 (but station found before timeout anyway)
 *     200 is safe for both — frees Phase 2 sooner in Config 1
 *     without affecting Config 2 where Phase 1 finishes naturally.
 *
 *   W_RECENCY = 0.35, W_PROXIMITY = 0.65
 *     Lifetime ratio Config1/Config2 = 100/30 = 3.3x
 *     Config 2 optimal: proximity=0.75 (objects expire fast, go nearest)
 *     Config 1 optimal: proximity=0.60 (old memories still valid)
 *     Balanced midpoint: (0.75+0.60)/2 = 0.675 → rounded to 0.65
 *     Small nudge toward recency helps Config 1 without hurting Config 2.
 *
 *   HOLE_BIAS_MARGIN = 6.0
 *     Config 2 wants 4.0  (holes expire at 30 steps — deposit fast)
 *     Config 1 wants 12.0 (holes last 100 steps — fill up first)
 *     Weighted by lifetime: 4 + (12-4)*(30/100) = 6.0
 *     Agent is willing to pick up a 3rd tile if it's within 6 steps
 *     closer than the nearest hole, balancing both configs.
 *
 * ── PHASE 1 (via Phase1Strategy) ─────────────────────────────────
 * - Map divided into 6 zones (2 cols x 3 rows)
 * - Each agent claims the zone nearest its spawn — no overlap
 * - Boustrophedon sweep per zone with sensor-range waypoints
 * - First agent to sense fuel station broadcasts via ArdaMessage
 * - All agents switch to Phase 2 the moment station is known
 *
 * ── PHASE 2 ───────────────────────────────────────────────────────
 * - Full map boustrophedon sweep, SWEEP_STEP jumps, starts at spawn
 * - Scored memory targeting with balanced recency/proximity weights
 * - Stale hole validation — removes expired holes from memory
 * - Time-aware fuel margin: 40 steps early → 15 steps late
 * - Position broadcast via ArdaMessage ("pos" entity type)
 * - Proximity yield: skip targets a closer capable teammate will reach
 *
 * ── FUEL EMERGENCY ───────────────────────────────────────────────
 * Global emergency check runs BEFORE all phase logic.
 * Threshold = dist_to_station + 60. Station unknown = 45% of max fuel.
 *
 * ── RULES RESPECTED ──────────────────────────────────────────────
 * - Extends TWAgent only
 * - Overrides only communicate(), think(), act()
 * - Does NOT call increaseReward() directly
 * - Does NOT modify the environment package
 * - Fuel station discovered only within sensor range (spec compliant)
 */
public class BalalaZonePatrolAgent_NoTimeout extends TWAgent {

    // ---------------------------------------------------------------
    // Tuned constants — balanced for Config 1, 2 and unknown Config 3
    // ---------------------------------------------------------------

    /** Steps before Phase 1 is force-abandoned. */

    /** Recency weight — slight increase from 0.25 helps Config 1. */
    private static final double W_RECENCY          = 0.40;

    /** Proximity weight — slight decrease from 0.75, still dominant. */
    private static final double W_PROXIMITY        = 0.60;

    /**
     * Hole bias margin — balanced between Config 1 (12) and Config 2 (4).
     * When carrying 2 tiles, pick up a 3rd only if the tile is this many
     * steps closer than the nearest hole.
     */
    private static final double HOLE_BIAS_MARGIN   = 3.0;

    /** Only yield a target to a teammate if they are at least this many
     *  steps closer. Prevents over-yielding in sparse maps (Config 1)
     *  where the agent would idle instead of going for a distant object. */
    private static final int    YIELD_THRESHOLD    = 5;

    // Fixed constants
    private static final double EMERGENCY_BUFFER   = 30.0;
    private static final double EMERGENCY_UNKNOWN  = 0.35;
    private static final double FUEL_SAFETY_EARLY  = 30.0;
    private static final double FUEL_SAFETY_LATE   = 10.0;
    private static final double FUEL_UNKNOWN_RATIO = 0.30;
    private static final int    CARRY_CAPACITY     = 3;
    private static final int    SWEEP_STEP         = Parameters.defaultSensorRange;
    private static final double EARLY_PHASE        = 0.20;
    private static final String ENTITY_POS         = "pos";
    private static final String ENTITY_TILE        = "tile";
    private static final String ENTITY_HOLE        = "hole";

    // ---------------------------------------------------------------
    // Fields
    // ---------------------------------------------------------------
    private final String             name;
    private final AstarPathGenerator pathGenerator;
    private BalalaCustomTWAgentMemory      customMemory;
    private final Phase1Strategy     phase1;


    private int fuelStationX = -1;
    private int fuelStationY = -1;

    private final List<int[]> teammateSnapshots = new ArrayList<int[]>();

    private int     sweepCol;
    private int     sweepRow;
    private boolean sweepGoingDown;

    // ---------------------------------------------------------------
    // Constructor
    // ---------------------------------------------------------------
    public BalalaZonePatrolAgent_NoTimeout(String name, int xpos, int ypos,
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

        this.sweepCol       = xpos;
        this.sweepRow       = ypos;
        this.sweepGoingDown = true;
    }

    // ---------------------------------------------------------------
    // communicate
    // ---------------------------------------------------------------
    @Override
    public void communicate() {
        phase1.communicate();
        if (!phase1.isComplete()) return;

        getEnvironment().receiveMessage(
                ArdaMessage.info(name, ENTITY_POS,
                        getX(), getY(),
                        carriedTiles.size(), 0));

        // Broadcast all known tiles and holes so teammates
        // can navigate to objects outside their own sensor range
        java.util.List<BalalaCustomTWAgentMemory.MemoryEntry> bTiles =
                customMemory.getPersonalTiles();
        for (int _i = 0; _i < bTiles.size(); _i++) {
            BalalaCustomTWAgentMemory.MemoryEntry _e = bTiles.get(_i);
            getEnvironment().receiveMessage(
                    ArdaMessage.info(name, ENTITY_TILE,
                            _e.x, _e.y, getX(), getY()));
        }
        java.util.List<BalalaCustomTWAgentMemory.MemoryEntry> bHoles =
                customMemory.getPersonalHoles();
        for (int _i = 0; _i < bHoles.size(); _i++) {
            BalalaCustomTWAgentMemory.MemoryEntry _e = bHoles.get(_i);
            getEnvironment().receiveMessage(
                    ArdaMessage.info(name, ENTITY_HOLE,
                            _e.x, _e.y, getX(), getY()));
        }
    }

    // ---------------------------------------------------------------
    // think
    // ---------------------------------------------------------------
    @Override
    protected TWThought think() {

        locateFuelStation();

        if (fuelStationX == -1 && phase1.getFuelStation() != null) {
            Int2D fs = phase1.getFuelStation();
            fuelStationX = fs.x;
            fuelStationY = fs.y;
        }


        int ax = getX();
        int ay = getY();

        // ── GLOBAL FUEL EMERGENCY ─────────────────────────────────
        if (isFuelEmergency(ax, ay)) {
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

        // ── PHASE 1 ───────────────────────────────────────────────
        if (!phase1.isComplete()) {
            TWThought t = phase1.think();
            if (t != null) return t;
        }

        // ── PHASE 2 ───────────────────────────────────────────────
        return phase2Think(ax, ay);
    }

    // ---------------------------------------------------------------
    // Phase 1 timeout

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
        double _now = getEnvironment().schedule.getTime();
        for (int i = 0; i < messages.size(); i++) {
            Message m = messages.get(i);
            if (m.getFrom().equals(name)) continue;
            if (!(m instanceof ArdaMessage)) continue;
            ArdaMessage am = (ArdaMessage) m;
            if (ENTITY_POS.equals(am.getEntityType())) {
                teammateSnapshots.add(new int[]{
                    am.getX(), am.getY(), am.getSenderX()
                });
            } else if (ENTITY_TILE.equals(am.getEntityType())) {
                // Inject teammate-seen tile into our memory
                customMemory.injectTile(am.getX(), am.getY(), _now);
            } else if (ENTITY_HOLE.equals(am.getEntityType())) {
                // Inject teammate-seen hole into our memory
                customMemory.injectHole(am.getX(), am.getY(), _now);
            }
        }
    }

    private boolean teammateCloserForTile(int tx, int ty, int myDist) {
        for (int i = 0; i < teammateSnapshots.size(); i++) {
            int[] s = teammateSnapshots.get(i);
            if (s[2] >= CARRY_CAPACITY) continue;
            if ((int) manhattan(s[0], s[1], tx, ty) < myDist - YIELD_THRESHOLD) return true;
        }
        return false;
    }

    private boolean teammateCloserForHole(int tx, int ty, int myDist) {
        for (int i = 0; i < teammateSnapshots.size(); i++) {
            int[] s = teammateSnapshots.get(i);
            if (s[2] == 0) continue;
            if ((int) manhattan(s[0], s[1], tx, ty) < myDist - YIELD_THRESHOLD) return true;
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
        List<BalalaCustomTWAgentMemory.MemoryEntry> stale =
                new ArrayList<BalalaCustomTWAgentMemory.MemoryEntry>();
        for (int i = 0; i < list.size(); i++) {
            BalalaCustomTWAgentMemory.MemoryEntry e = list.get(i);
            // Validate object still exists on grid
            TWEntity _obj = (TWEntity) getEnvironment()
                    .getObjectGrid().get(e.x, e.y);
            if (isTile && !(_obj instanceof tileworld.environment.TWTile)) {
                stale.add(e); continue;
            }
            if (!isTile && !(_obj instanceof tileworld.environment.TWHole)) {
                stale.add(e); continue;
            }
            int myDist = (int) manhattan(ax, ay, e.x, e.y);
            if (isTile  && teammateCloserForTile(e.x, e.y, myDist)) continue;
            if (!isTile && teammateCloserForHole(e.x, e.y, myDist)) continue;
            if (best == null || e.utility > best.utility) best = e;
        }
        for (int i = 0; i < stale.size(); i++) {
            if (isTile) customMemory.removeTile(stale.get(i).x, stale.get(i).y);
            else customMemory.removeHole(stale.get(i).x, stale.get(i).y);
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
    // BOUSTROPHEDON SWEEP
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
