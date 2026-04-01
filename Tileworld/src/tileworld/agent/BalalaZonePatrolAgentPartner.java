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
 * Extends TWAgentSkeleton which handles:
 *   - Phase 1 zone sweep + fuel station discovery (via Phase1Strategy)
 *   - communicate() pipeline (final in skeleton)
 *   - think() pipeline (final in skeleton)
 *   - processSharedMessages() — auto-extracts fuel station from broadcasts
 *   - fuelStationX / fuelStationY shared state
 *   - manhattan() helper
 *
 * This class implements the four abstract hooks:
 *   - customCommunicate() — Phase 2 position broadcast via ArdaMessage
 *   - handleTeamMessage() — reads teammate positions for proximity yield
 *   - customThink()       — Phase 2 decision logic
 *   - act()               — executes chosen action
 *
 * ── PHASE 1 TIMEOUT ──────────────────────────────────────────────
 * If Phase 1 hasn't finished by PHASE1_TIMEOUT steps, customThink()
 * takes over even without a known fuel station. Phase 2's full-map
 * sweep will find the station eventually.
 *
 * ── FUEL EMERGENCY ───────────────────────────────────────────────
 * customThink() starts with a global fuel emergency check that runs
 * before all other logic, preventing the agent from running dry.
 *
 * ── CONSTANTS — balanced for Config 1, 2, and unknown Config 3 ───
 *   PHASE1_TIMEOUT  = 200  (Config 1: needs early Phase 2 switch)
 *   W_RECENCY       = 0.35 (Config 1: old memories last 100 steps)
 *   W_PROXIMITY     = 0.65 (Config 2: still proximity-dominant)
 *   HOLE_BIAS_MARGIN= 6.0  (midpoint between Config1=12, Config2=4)
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
    private static final int    PHASE1_TIMEOUT     = 150;
    private static final double EMERGENCY_BUFFER   = 15.0;
    private static final double EMERGENCY_UNKNOWN  = 0.25;
    private static final double FUEL_SAFETY_EARLY  = 20.0;
    private static final double FUEL_SAFETY_LATE   = 5.0;
    private static final double FUEL_UNKNOWN_RATIO = 0.20;
    private static final double W_RECENCY          = 0.35;
    private static final double W_PROXIMITY        = 0.65;
    private static final double HOLE_BIAS_MARGIN   = 6.0;
    private static final int    SWEEP_STEP         = Parameters.defaultSensorRange;
    private static final double EARLY_PHASE        = 0.20;
    private static final String ENTITY_POS         = "pos";

    // ---------------------------------------------------------------
    // Fields
    // ---------------------------------------------------------------
    private final AstarPathGenerator  pathGenerator;
    private BalalaCustomTWAgentMemory customMemory;

    private boolean phase1Done = false;

    private final List<int[]> teammateSnapshots = new ArrayList<int[]>();

    // Phase 2 sweep — starts at spawn
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
    // HOOK 3: customThink
    // ---------------------------------------------------------------
    @Override
    protected TWThought customThink() {

        // Rebuild snapshots fresh each step
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

        // Global fuel emergency — runs before everything
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
    // PHASE 1 TIMEOUT
    // ---------------------------------------------------------------
    private void checkPhase1Timeout() {
        if (phase1Done) return;
        if (phase1.isComplete()) { phase1Done = true; return; }
        double now = getEnvironment().schedule.getTime();
        if (now >= PHASE1_TIMEOUT) {
            phase1Done = true;
            System.out.println(agentName + " Phase 1 TIMEOUT at step "
                    + (int) now + " — switching to Phase 2");
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
    // PROXIMITY YIELD
    // ---------------------------------------------------------------
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

        List<BalalaCustomTWAgentMemory.MemoryEntry> tiles =
                customMemory.getKnownTiles();
        List<BalalaCustomTWAgentMemory.MemoryEntry> holes =
                customMemory.getKnownHoles();

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
        List<BalalaCustomTWAgentMemory.MemoryEntry> holes =
                customMemory.getKnownHoles();
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
    // SWEEP
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
