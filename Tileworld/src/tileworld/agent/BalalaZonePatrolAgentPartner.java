package tileworld.agent;

import java.util.ArrayList;
import java.util.List;
import tileworld.Parameters;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.AstarPathGenerator;
import tileworld.planners.TWPath;

/**
 * BalalaZonePatrolAgentPartner
 *
 * Extends TWAgentSkeleton which provides:
 *   - Phase 1 zone sweep + fuel station discovery (Phase1Strategy)
 *   - communicate() pipeline (final) — automatically broadcasts all
 *     visible tiles, holes and fuel station every step
 *   - think() pipeline (final) — processSharedMessages() then Phase1
 *     then customThink()
 *   - getSharedTileLocations() / getSharedHoleLocations() — coordinates
 *     seen by teammates, pruned automatically when stale
 *   - isClaimedByCloserAgent() — intent-based yield system
 *   - setIntention() / clearIntention() — broadcasts current target
 *   - fuelStationX / fuelStationY — auto-populated from broadcasts
 *   - manhattan() helper
 *
 * This class implements the four abstract hooks:
 *   - customCommunicate() — Phase 2 extra broadcasts (pos message)
 *   - handleTeamMessage() — reads pos snapshots for carry-state yield
 *   - customThink()       — Phase 2 decision logic
 *   - act()               — executes chosen action
 *
 * ── PHASE 1 TIMEOUT ──────────────────────────────────────────────
 * PHASE1_TIMEOUT = 30 steps. In solo/Config1 Phase 1 is abandoned
 * quickly so Phase 2 full-map sweep starts immediately. In 6-agent
 * team Phase 1 completes naturally before timeout fires.
 *
 * ── FUEL EMERGENCY ───────────────────────────────────────────────
 * Global check at top of customThink() before all other logic.
 * EMERGENCY_BUFFER = 45 (used to be 30), EMERGENCY_UNKNOWN = 0.35.
 *
 * ── SHARED MEMORY TARGETING ──────────────────────────────────────
 * In addition to personal memory (BalalaCustomTWAgentMemory), the
 * agent uses getSharedTileLocations() and getSharedHoleLocations()
 * from the skeleton to navigate to teammate-seen objects. This
 * effectively gives the agent a full-map view constructed from all
 * 6 agents' sensor ranges combined — fixing the idle problem.
 *
 * ── INTENT YIELD ─────────────────────────────────────────────────
 * Uses skeleton's isClaimedByCloserAgent() to skip targets already
 * claimed by a closer teammate. Calls setIntention() before moving
 * to any target so teammates can yield to us too.
 *
 * ── CONSTANTS — balanced for Config 1, 2, unknown Config 3 ───────
 *   W_RECENCY = 0.40, W_PROXIMITY = 0.60
 *   HOLE_BIAS_MARGIN = 3.0
 *   YIELD_THRESHOLD = 5
 *
 * ── RULES RESPECTED ──────────────────────────────────────────────
 * - Extends TWAgentSkeleton (which extends TWAgent)
 * - communicate() and think() NOT overridden (final in skeleton)
 * - Does NOT call increaseReward() directly
 * - Does NOT modify the environment package
 * - Fuel station found only within sensor range (spec compliant)
 */
public class BalalaZonePatrolAgentPartner extends TWAgentSkeleton {

    // ---------------------------------------------------------------
    // Constants
    // ---------------------------------------------------------------
    private static final int    PHASE1_TIMEOUT     = 30;
    private static final double EMERGENCY_BUFFER   = 45.0;
    private static final double EMERGENCY_UNKNOWN  = 0.35;
    private static final double FUEL_SAFETY_EARLY  = 30.0;
    private static final double FUEL_SAFETY_LATE   = 10.0;
    private static final double FUEL_UNKNOWN_RATIO = 0.30;
    private static final double W_RECENCY          = 0.40;
    private static final double W_PROXIMITY        = 0.60;
    private static final double HOLE_BIAS_MARGIN   = 3.0;

    /**
     * Only yield to a teammate if they are at least this many steps
     * closer — prevents over-yielding and excessive idle in sparse maps.
     */
    private static final int    YIELD_THRESHOLD    = 5;
    private static final int    SWEEP_STEP         = Parameters.defaultSensorRange;
    private static final double EARLY_PHASE        = 0.20;
    private static final String ENTITY_POS         = "pos";

    // ---------------------------------------------------------------
    // Fields
    // ---------------------------------------------------------------
    private final AstarPathGenerator  pathGenerator;
    private BalalaCustomTWAgentMemory customMemory;

    private boolean phase1Done = false;

    /**
     * Teammate carry-state snapshots from "pos" ArdaMessage broadcasts.
     * Each entry: [x, y, tilesCarried]
     * Used for carry-state-aware yield (can teammate fill a hole?).
     */
    private final List<int[]> teammateSnapshots = new ArrayList<int[]>();

    // Phase 2 boustrophedon sweep — starts at spawn
    private int     sweepCol;
    private int     sweepRow;
    private boolean sweepGoingDown;

    // ---------------------------------------------------------------
    // Constructor
    // ---------------------------------------------------------------
    public BalalaZonePatrolAgentPartner(String name, int xpos, int ypos,
                                        TWEnvironment env, double fuelLevel) {
        super(name, xpos, ypos, env, fuelLevel);

        this.customMemory = new BalalaCustomTWAgentMemory(
                this, env.schedule,
                env.getxDimension(), env.getyDimension());
        this.memory = customMemory;

        this.pathGenerator = new AstarPathGenerator(
                env, this,
                env.getxDimension() * env.getyDimension());

        this.sweepCol       = xpos;
        this.sweepRow       = ypos;
        this.sweepGoingDown = true;
    }

    // ---------------------------------------------------------------
    // HOOK 1: customCommunicate
    // Skeleton already broadcasts visible tiles/holes/fuel via
    // broadcastVisibleEntityInfo(). We just add the pos message
    // for carry-state-aware yield logic.
    // ---------------------------------------------------------------
    @Override
    protected void customCommunicate() {
        getEnvironment().receiveMessage(
                ArdaMessage.info(agentName, ENTITY_POS,
                        getX(), getY(),
                        carriedTiles.size(), 0));
    }

    // ---------------------------------------------------------------
    // HOOK 2: handleTeamMessage
    // Skeleton handles tile/hole/fuel ingestion automatically.
    // We just collect pos snapshots for carry-state yield.
    // ---------------------------------------------------------------
    @Override
    protected void handleTeamMessage(ArdaMessage msg) {
        if (ENTITY_POS.equals(msg.getEntityType())) {
            teammateSnapshots.add(new int[]{
                msg.getX(), msg.getY(), msg.getSenderX()
            });
        }
    }

    // ---------------------------------------------------------------
    // HOOK 3: customThink — Phase 2 decision logic
    // ---------------------------------------------------------------
    @Override
    protected TWThought customThink() {

        // Rebuild pos snapshots fresh each step
        teammateSnapshots.clear();
        for (Message raw : getEnvironment().getMessages()) {
            if (agentName.equals(raw.getFrom())) continue;
            if (!(raw instanceof ArdaMessage)) continue;
            ArdaMessage am = (ArdaMessage) raw;
            if (ENTITY_POS.equals(am.getEntityType())) {
                teammateSnapshots.add(new int[]{
                    am.getX(), am.getY(), am.getSenderX()
                });
            }
        }

        checkPhase1Timeout();

        int    ax  = getX();
        int    ay  = getY();
        double now = getEnvironment().schedule.getTime();

        // ── GLOBAL FUEL EMERGENCY ─────────────────────────────────
        if (isFuelEmergency(ax, ay)) {
            clearIntention();
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

        boolean earlyPhase = isEarlyPhase(now);

        // 1. Normal fuel check
        if (needsRefuel(ax, ay, now)) {
            clearIntention();
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
            clearIntention();
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }

        // 3. Opportunistic pickup
        if (onCell instanceof TWTile && carriedTiles.size() < CARRY_CAPACITY) {
            clearIntention();
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }

        // 4. Full carry → nearest valid unclaimed hole
        if (carriedTiles.size() == CARRY_CAPACITY) {
            int[] hole = nearestValidHole(ax, ay);
            if (hole != null) {
                setIntention(ENTITY_HOLE, hole[0], hole[1]);
                return new TWThought(TWAction.MOVE, pathTo(hole[0], hole[1]));
            }
            clearIntention();
            return new TWThought(TWAction.MOVE, nextSweepDirection(ax, ay));
        }

        // 5. Scored target — personal memory + skeleton shared knowledge
        int[] target = selectTarget(ax, ay, earlyPhase);
        if (target != null) {
            setIntention(target[2] == 0 ? ENTITY_TILE : ENTITY_HOLE,
                         target[0], target[1]);
            return new TWThought(TWAction.MOVE, pathTo(target[0], target[1]));
        }

        // 6. Sweep
        clearIntention();
        return new TWThought(TWAction.MOVE, nextSweepDirection(ax, ay));
    }

    // ---------------------------------------------------------------
    // HOOK 4: act
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
                        forgetSharedTile(getX(), getY());
                    }
                    break;
                case PUTDOWN:
                    TWHole hole = (TWHole) getEnvironment()
                            .getObjectGrid().get(getX(), getY());
                    if (hole != null) {
                        putTileInHole(hole);
                        customMemory.removeHole(getX(), getY());
                        forgetSharedHole(getX(), getY());
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
    // PHASE 1 TIMEOUT
    // ---------------------------------------------------------------
    private void checkPhase1Timeout() {
        if (phase1Done) return;
        if (phase1.isComplete()) { phase1Done = true; return; }
        double now = getEnvironment().schedule.getTime();
        if (now >= PHASE1_TIMEOUT) {
            phase1Done = true;
            System.out.println(agentName + " Phase1 TIMEOUT step "
                    + (int) now + " → Phase 2");
        }
    }

    // ---------------------------------------------------------------
    // FUEL MANAGEMENT
    // ---------------------------------------------------------------
    private boolean isFuelEmergency(int ax, int ay) {
        if (fuelStationX == -1) {
            return fuelLevel <= Parameters.defaultFuelLevel * EMERGENCY_UNKNOWN;
        }
        return fuelLevel <= manhattan(ax, ay, fuelStationX, fuelStationY)
                          + EMERGENCY_BUFFER;
    }

    private boolean needsRefuel(int ax, int ay, double now) {
        if (fuelStationX == -1) {
            return fuelLevel <= Parameters.defaultFuelLevel * FUEL_UNKNOWN_RATIO;
        }
        return fuelLevel <= manhattan(ax, ay, fuelStationX, fuelStationY)
                          + computeFuelMargin(now);
    }

    private double computeFuelMargin(double now) {
        double maxT    = Parameters.defaultFuelLevel * 10.0;
        double elapsed = Math.min(now / maxT, 1.0);
        return FUEL_SAFETY_EARLY
               - elapsed * (FUEL_SAFETY_EARLY - FUEL_SAFETY_LATE);
    }

    private boolean isEarlyPhase(double now) {
        return (now / (Parameters.defaultFuelLevel * 10.0)) < EARLY_PHASE;
    }

    // ---------------------------------------------------------------
    // TARGET SELECTION — personal memory + skeleton shared knowledge
    // ---------------------------------------------------------------

    /**
     * Selects the best target by combining:
     *   1. Personal memory (BalalaCustomTWAgentMemory)
     *   2. Skeleton shared tile/hole locations (getSharedTileLocations/Holes)
     * Uses skeleton's isClaimedByCloserAgent() for intent yield,
     * and carry-state yield (YIELD_THRESHOLD) for pos-based yield.
     *
     * Returns int[]{x, y, type} where type=0=tile, type=1=hole, or null.
     */
    private int[] selectTarget(int ax, int ay, boolean earlyPhase) {
        int[] bestTile = bestTileTarget(ax, ay);
        int[] bestHole = bestHoleTarget(ax, ay);

        int carried = carriedTiles.size();

        if (earlyPhase && carried < 2 && bestTile != null) return bestTile;

        if (carried == 2) {
            if (bestHole == null) return bestTile;
            if (bestTile == null) return bestHole;
            double dTile = manhattan(ax, ay, bestTile[0], bestTile[1]);
            double dHole = manhattan(ax, ay, bestHole[0], bestHole[1]);
            return (dTile + HOLE_BIAS_MARGIN < dHole) ? bestTile : bestHole;
        }
        if (carried == 1) {
            if (bestTile == null) return bestHole;
            if (bestHole == null) return bestTile;
            double dTile = manhattan(ax, ay, bestTile[0], bestTile[1]);
            double dHole = manhattan(ax, ay, bestHole[0], bestHole[1]);
            return (dTile <= dHole) ? bestTile : bestHole;
        }
        return bestTile; // carried == 0
    }

    /**
     * Best tile from personal memory + shared knowledge.
     * Validates against live grid. Yields if claimed by closer agent
     * OR if a capable teammate is YIELD_THRESHOLD+ steps closer.
     */
    private int[] bestTileTarget(int ax, int ay) {
        int[] best = null;
        double bestScore = Double.NEGATIVE_INFINITY;

        // Personal memory tiles
        List<BalalaCustomTWAgentMemory.MemoryEntry> personal =
                customMemory.getKnownTiles();
        for (int i = 0; i < personal.size(); i++) {
            BalalaCustomTWAgentMemory.MemoryEntry e = personal.get(i);
            TWEntity obj = (TWEntity) getEnvironment()
                    .getObjectGrid().get(e.x, e.y);
            if (!(obj instanceof TWTile)) {
                customMemory.removeTile(e.x, e.y); continue;
            }
            if (isClaimedByCloserAgent(ENTITY_TILE, e.x, e.y)) continue;
            if (teammateCloserForTile(e.x, e.y, (int) manhattan(ax, ay, e.x, e.y))) continue;
            double score = score(ax, ay, e.x, e.y,
                    getEnvironment().schedule.getTime(), e.timestamp);
            if (score > bestScore) { bestScore = score; best = new int[]{e.x, e.y, 0}; }
        }

        // Skeleton shared tile locations
        List<int[]> shared = getSharedTileLocations();
        for (int i = 0; i < shared.size(); i++) {
            int tx = shared.get(i)[0], ty = shared.get(i)[1];
            TWEntity obj = (TWEntity) getEnvironment()
                    .getObjectGrid().get(tx, ty);
            if (!(obj instanceof TWTile)) {
                forgetSharedTile(tx, ty); continue;
            }
            if (isClaimedByCloserAgent(ENTITY_TILE, tx, ty)) continue;
            if (teammateCloserForTile(tx, ty, (int) manhattan(ax, ay, tx, ty))) continue;
            double score = score(ax, ay, tx, ty,
                    getEnvironment().schedule.getTime(),
                    getEnvironment().schedule.getTime()); // shared = fresh
            if (score > bestScore) { bestScore = score; best = new int[]{tx, ty, 0}; }
        }

        return best;
    }

    /**
     * Best hole from personal memory + shared knowledge.
     */
    private int[] bestHoleTarget(int ax, int ay) {
        int[] best = null;
        double bestScore = Double.NEGATIVE_INFINITY;

        // Personal memory holes
        List<BalalaCustomTWAgentMemory.MemoryEntry> personal =
                customMemory.getKnownHoles();
        for (int i = 0; i < personal.size(); i++) {
            BalalaCustomTWAgentMemory.MemoryEntry e = personal.get(i);
            TWEntity obj = (TWEntity) getEnvironment()
                    .getObjectGrid().get(e.x, e.y);
            if (!(obj instanceof TWHole)) {
                customMemory.removeHole(e.x, e.y); continue;
            }
            if (isClaimedByCloserAgent(ENTITY_HOLE, e.x, e.y)) continue;
            if (teammateCloserForHole(e.x, e.y, (int) manhattan(ax, ay, e.x, e.y))) continue;
            double score = score(ax, ay, e.x, e.y,
                    getEnvironment().schedule.getTime(), e.timestamp);
            if (score > bestScore) { bestScore = score; best = new int[]{e.x, e.y, 1}; }
        }

        // Skeleton shared hole locations
        List<int[]> shared = getSharedHoleLocations();
        for (int i = 0; i < shared.size(); i++) {
            int hx = shared.get(i)[0], hy = shared.get(i)[1];
            TWEntity obj = (TWEntity) getEnvironment()
                    .getObjectGrid().get(hx, hy);
            if (!(obj instanceof TWHole)) {
                forgetSharedHole(hx, hy); continue;
            }
            if (isClaimedByCloserAgent(ENTITY_HOLE, hx, hy)) continue;
            if (teammateCloserForHole(hx, hy, (int) manhattan(ax, ay, hx, hy))) continue;
            double score = score(ax, ay, hx, hy,
                    getEnvironment().schedule.getTime(),
                    getEnvironment().schedule.getTime());
            if (score > bestScore) { bestScore = score; best = new int[]{hx, hy, 1}; }
        }

        return best;
    }

    /**
     * Nearest valid hole — used when carrying 3 tiles.
     * Checks both personal memory and shared knowledge.
     */
    private int[] nearestValidHole(int ax, int ay) {
        int[] nearest = null;
        double minDist = Double.MAX_VALUE;

        List<BalalaCustomTWAgentMemory.MemoryEntry> personal =
                customMemory.getKnownHoles();
        for (int i = 0; i < personal.size(); i++) {
            BalalaCustomTWAgentMemory.MemoryEntry e = personal.get(i);
            TWEntity obj = (TWEntity) getEnvironment()
                    .getObjectGrid().get(e.x, e.y);
            if (!(obj instanceof TWHole)) { customMemory.removeHole(e.x, e.y); continue; }
            if (isClaimedByCloserAgent(ENTITY_HOLE, e.x, e.y)) continue;
            double d = manhattan(ax, ay, e.x, e.y);
            if (d < minDist) { minDist = d; nearest = new int[]{e.x, e.y}; }
        }

        List<int[]> shared = getSharedHoleLocations();
        for (int i = 0; i < shared.size(); i++) {
            int hx = shared.get(i)[0], hy = shared.get(i)[1];
            TWEntity obj = (TWEntity) getEnvironment()
                    .getObjectGrid().get(hx, hy);
            if (!(obj instanceof TWHole)) { forgetSharedHole(hx, hy); continue; }
            if (isClaimedByCloserAgent(ENTITY_HOLE, hx, hy)) continue;
            double d = manhattan(ax, ay, hx, hy);
            if (d < minDist) { minDist = d; nearest = new int[]{hx, hy}; }
        }

        return nearest;
    }

    private double score(int ax, int ay, int tx, int ty,
                         double now, double timestamp) {
        double maxD    = getEnvironment().getxDimension()
                       + getEnvironment().getyDimension();
        double age     = now - timestamp;
        double recency = 1.0 / (1.0 + age);
        double dist    = manhattan(ax, ay, tx, ty);
        double prox    = Math.max(0.0, 1.0 - dist / maxD);
        return W_RECENCY * recency + W_PROXIMITY * prox;
    }

    // ---------------------------------------------------------------
    // CARRY-STATE YIELD (pos-based, complements intent yield)
    // ---------------------------------------------------------------
    private boolean teammateCloserForTile(int tx, int ty, int myDist) {
        for (int i = 0; i < teammateSnapshots.size(); i++) {
            int[] s = teammateSnapshots.get(i);
            if (s[2] >= CARRY_CAPACITY) continue;
            if ((int) manhattan(s[0], s[1], tx, ty) < myDist - YIELD_THRESHOLD)
                return true;
        }
        return false;
    }

    private boolean teammateCloserForHole(int tx, int ty, int myDist) {
        for (int i = 0; i < teammateSnapshots.size(); i++) {
            int[] s = teammateSnapshots.get(i);
            if (s[2] == 0) continue;
            if ((int) manhattan(s[0], s[1], tx, ty) < myDist - YIELD_THRESHOLD)
                return true;
        }
        return false;
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
}
