package tileworld.agent;

import java.util.List;
import tileworld.Parameters;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.AstarPathGenerator;
import tileworld.planners.TWPath;


public class PerimeterAgent extends TWAgentSkeleton {

    private static final double FUEL_THRESHOLD = 0.25;  // refuel when fuel < 25 % of max
    private static final double DETOUR_LIMIT   = 10.0;  // max Manhattan detour for opp. targets
    private static final double EMA_ALPHA      = 0.2;   // EMA weight for newest observation
    private static final double NUDGE_FACTOR   = 1.05;  // expand R 5 % after surplus-fuel loop
    private static final double SURPLUS_FACTOR = 1.2;   // "surplus" threshold multiplier
    private static final int    R_MIN          = 5;     // floor on anchored radius

    private final AstarPathGenerator astar;
    private TWPath path  = null;
    private int    targX = -1, targY = -1;

    private final int[][] mapCorners;   // fixed 4 map corners
    private int[][] corners;            // active patrol corners (map or anchored box)
    private int     cornerIdx = 0;

    private boolean phase2Ready = false;

    private boolean anchoredMode = false;  // true = huge map
    private int     anchoredR    = 0;      // current box half-side
    private int     anchoredRMax = 0;      // ceiling = safeUsableFuel / 8

    private int    cornersThisCycle = 0;
    private double smoothedRatio    = 1.0; // EMA of (cornersVisited / 4)
    private double fuelAtLastRefuel = -1;  // fuel at the start of previous tick

    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================

    public PerimeterAgent(String name, int x, int y, TWEnvironment env, double fuel) {
        super(name, x, y, env, fuel); // skeleton sets agentName, phase1
        this.astar = new AstarPathGenerator(env, this, env.getxDimension() * env.getyDimension());

        int maxX = env.getxDimension() - 1;
        int maxY = env.getyDimension() - 1;
        this.mapCorners = new int[][]{{0, 0}, {maxX, 0}, {maxX, maxY}, {0, maxY}};
        this.corners    = mapCorners; // default until Phase 2 decides
    }

    // =========================================================================
    // PHASE 2 INITIALISATION  (runs exactly once, when customThink() first fires)
    // =========================================================================

    private void initPhase2() {
        if (phase2Ready) return;
        phase2Ready      = true;
        fuelAtLastRefuel = fuelLevel; // baseline for refuel-event detection

        int    W              = getEnvironment().getxDimension();
        int    H              = getEnvironment().getyDimension();
        int    perimeterLen   = 2 * (W + H - 2);
        double safeUsableFuel = Parameters.defaultFuelLevel * (1.0 - FUEL_THRESHOLD);

        if (perimeterLen <= safeUsableFuel) {
            anchoredMode = false;
            corners      = mapCorners;
            cornerIdx    = nearestCornerIdx(getX(), getY(), corners);
            System.out.println(agentName + " [Phase2] FULL_PERIMETER"
                + "  perimeter=" + perimeterLen
                + "  startCorner=" + cornerIdx);
        } else {
            anchoredMode = true;
            anchoredRMax = (int)(safeUsableFuel / 8.0);
            anchoredR    = anchoredRMax;
            recomputeAnchoredCorners();
            cornerIdx    = nearestCornerIdx(getX(), getY(), corners);
            System.out.println(agentName + " [Phase2] FUEL_ANCHORED"
                + "  perimeter=" + perimeterLen
                + "  R=" + anchoredR
                + "  startCorner=" + cornerIdx);
        }
    }

    // =========================================================================
    // ANCHORED BOX
    // =========================================================================

    /**
     * Recomputes the 4 patrol corners of the box around the fuel station.
     *
     * The box is always 2R Ã— 2R (full perimeter = 8R).
     * When the fuel station is near a map edge or corner, the box is SHIFTED
     * inward instead of truncated  this places the fuel station at a corner
     * of the patrol box rather than shrinking the patrol area.
     *
     * Example  fuel station at map corner (0, 0), R=46:
     *   Centered start:  xMin=-46, xMax=46
     *   Shift right +46: xMin=  0, xMax=92   â†� full 2R width preserved
     *   Fuel station sits at corners[0] = (xMin, yMin)
     */
    private void recomputeAnchoredCorners() {
        int mapMaxX = getEnvironment().getxDimension() - 1;
        int mapMaxY = getEnvironment().getyDimension() - 1;

        // Start with a box centered on the fuel station
        int xMin = fuelStationX - anchoredR;
        int xMax = fuelStationX + anchoredR;
        int yMin = fuelStationY - anchoredR;
        int yMax = fuelStationY + anchoredR;

        // Shift (not clamp) to keep the full 2RÃ—2R box inside map bounds.
        // When the fuel station is near an edge, it ends up at a corner of the box.
        if (xMin < 0)       { xMax -= xMin;            xMin = 0; }
        if (xMax > mapMaxX) { xMin -= (xMax - mapMaxX); xMax = mapMaxX; }
        if (yMin < 0)       { yMax -= yMin;            yMin = 0; }
        if (yMax > mapMaxY) { yMin -= (yMax - mapMaxY); yMax = mapMaxY; }

        // Safety clamp  only triggers if the map itself is narrower than 2R
        // (cannot happen in FUEL_ANCHORED mode for valid map sizes, defensive only)
        xMin = Math.max(0, xMin);
        xMax = Math.min(mapMaxX, xMax);
        yMin = Math.max(0, yMin);
        yMax = Math.min(mapMaxY, yMax);

        corners = new int[][]{{xMin, yMin}, {xMax, yMin}, {xMax, yMax}, {xMin, yMax}};
    }

    // =========================================================================
    // EXPERIENCE-BASED REFUEL TRACKING
    // =========================================================================

    /**
     * Called every tick at the top of customThink().
     * Detects a refuel event (fuel rose since last tick) and triggers
     * the EMA update + R calibration.
     */
    private void checkRefuelEvent() {
        if (fuelAtLastRefuel < 0) {
            fuelAtLastRefuel = fuelLevel;
            return;
        }
        if (fuelLevel > fuelAtLastRefuel) {
            // fuelAtLastRefuel = level just before refuelling this tick
            onRefuelCycleComplete(fuelAtLastRefuel);
        }
        fuelAtLastRefuel = fuelLevel;
    }

    /**
     * Called once per completed refuel cycle.
     * Updates smoothedRatio via EMA and adjusts anchoredR accordingly.
     *
     * @param fuelAtReturn fuel the agent had when it arrived back to refuel
     */
    private void onRefuelCycleComplete(double fuelAtReturn) {
        if (!anchoredMode) return;

        // EMA update
        double ratio  = Math.min(cornersThisCycle, 4) / 4.0;
        smoothedRatio = (1.0 - EMA_ALPHA) * smoothedRatio + EMA_ALPHA * ratio;

        // Nudge upward when a full loop completed with surplus fuel
        double surplusThreshold = SURPLUS_FACTOR * FUEL_THRESHOLD * Parameters.defaultFuelLevel;
        if (cornersThisCycle >= 4 && fuelAtReturn > surplusThreshold)
            smoothedRatio = Math.min(1.0, smoothedRatio * NUDGE_FACTOR);

        // Derive R from smoothedRatio Ã— ceiling  always apply to anchoredRMax
        int newR = Math.max(R_MIN, Math.min(anchoredRMax, (int)(anchoredRMax * smoothedRatio)));

        if (newR != anchoredR) {
            anchoredR = newR;
            recomputeAnchoredCorners();
            // Keep cornerIdx: continue patrol direction, no re-snap on R change
            System.out.println(agentName + " [Phase2] R â†’ " + anchoredR
                + "  corners=" + cornersThisCycle
                + "  smoothedRatio=" + String.format("%.2f", smoothedRatio));
        }

        cornersThisCycle = 0;
    }

    // =========================================================================
    // SKELETON HOOK customCommunicate()
    // Called by skeleton AFTER broadcastVisibleEntityInfo() + broadcastCurrentIntention()
    // =========================================================================

    @Override
    protected void customCommunicate() {
        // Skeleton already sent ArdaMessages for every visible tile/hole/fuel.
        // We additionally write to SharedBlackboard so non-skeleton teammates
        // (ZoneSweepAgent, BenTWAgent, etc.) can read the same data.
        int    r    = Parameters.defaultSensorRange;
        int    ax   = getX(), ay = getY();
        double step = getEnvironment().schedule.getTime();

        for (int dx = -r; dx <= r; dx++) {
            for (int dy = -r; dy <= r; dy++) {
                int cx = ax + dx, cy = ay + dy;
                if (!getEnvironment().isValidLocation(cx, cy)) continue;
                Object o = getEnvironment().getObjectGrid().get(cx, cy);
                if      (o instanceof TWTile)        SharedBlackboard.putTile(cx, cy, step);
                else if (o instanceof TWHole)        SharedBlackboard.putHole(cx, cy, step);
                else if (o == null)                  { SharedBlackboard.removeTile(cx, cy);
                                                       SharedBlackboard.removeHole(cx, cy); }
                else if (o instanceof TWFuelStation) SharedBlackboard.setFuel(cx, cy);
            }
        }
    }

    // =========================================================================
    // SKELETON HOOK handleTeamMessage()
    // Bridges ArdaMessages from skeleton teammates into SharedBlackboard
    // =========================================================================

    @Override
    protected void handleTeamMessage(ArdaMessage msg) {
        if (msg.getType() != ArdaMessage.MessageType.INFO) return;
        double step = getEnvironment().schedule.getTime();
        String et   = msg.getEntityType();
        if      (ENTITY_TILE.equals(et))        SharedBlackboard.putTile(msg.getX(), msg.getY(), step);
        else if (ENTITY_HOLE.equals(et))        SharedBlackboard.putHole(msg.getX(), msg.getY(), step);
        else if (ENTITY_FUEL.equals(et))        SharedBlackboard.setFuel(msg.getX(), msg.getY());
        else if (ENTITY_DELETE_TILE.equals(et)) SharedBlackboard.removeTile(msg.getX(), msg.getY());
        else if (ENTITY_DELETE_HOLE.equals(et)) SharedBlackboard.removeHole(msg.getX(), msg.getY());
    }

    // =========================================================================
    // SKELETON HOOK  customThink()
    // Full Phase 2 decision logic (skeleton guarantees fuelStationX/Y are set)
    // =========================================================================

    @Override
    protected TWThought customThink() {
        initPhase2();      // one-time mode selection
        checkRefuelEvent(); // EMA + R update on refuel detection

        int ax = getX(), ay = getY();

        // 1. Fuel check
        if (fuelLevel <= Parameters.defaultFuelLevel * FUEL_THRESHOLD) {
            clearIntention();
            if (ax == fuelStationX && ay == fuelStationY)
                return new TWThought(TWAction.REFUEL, TWDirection.Z);
            return new TWThought(TWAction.MOVE, navigate(fuelStationX, fuelStationY));
        }

        // 2. In-place actions (no movement cost)
        Object here = getEnvironment().getObjectGrid().get(ax, ay);
        if (here instanceof TWHole && hasTile()) {
            clearIntention();
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }
        if (here instanceof TWTile && carriedTiles.size() < CARRY_CAPACITY) {
            clearIntention();
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }

        // 3. Opportunistic: nearby hole in box (claim-checked)
        if (hasTile()) {
            TWHole h = (TWHole) memory.getClosestObjectInSensorRange(TWHole.class);
            if (h != null
                    && manhattan(ax, ay, h.getX(), h.getY()) <= DETOUR_LIMIT
                    && isInsidePatrolBox(h.getX(), h.getY())
                    && !isClaimedByCloserAgent(ENTITY_HOLE, h.getX(), h.getY())) {
                setIntention(ENTITY_HOLE, h.getX(), h.getY());
                return new TWThought(TWAction.MOVE, navigate(h.getX(), h.getY()));
            }

            // Carrying max tiles but no nearby hole  search skeleton's shared hole map
            if (carriedTiles.size() >= CARRY_CAPACITY) {
                int[] bh = nearestSharedInBox(getSharedHoleLocations(), ax, ay);
                if (bh != null && !isClaimedByCloserAgent(ENTITY_HOLE, bh[0], bh[1])) {
                    setIntention(ENTITY_HOLE, bh[0], bh[1]);
                    return new TWThought(TWAction.MOVE, navigate(bh[0], bh[1]));
                }
            }
        }

        // 4. Opportunistic: nearby tile in box (claim-checked)
        if (carriedTiles.size() < CARRY_CAPACITY) {
            TWTile t = (TWTile) memory.getClosestObjectInSensorRange(TWTile.class);
            if (t != null
                    && manhattan(ax, ay, t.getX(), t.getY()) <= DETOUR_LIMIT
                    && isInsidePatrolBox(t.getX(), t.getY())
                    && !isClaimedByCloserAgent(ENTITY_TILE, t.getX(), t.getY())) {
                setIntention(ENTITY_TILE, t.getX(), t.getY());
                return new TWThought(TWAction.MOVE, navigate(t.getX(), t.getY()));
            }
        }

        // 5. Perimeter / box patrol
        clearIntention();
        return new TWThought(TWAction.MOVE, patrol());
    }

    // =========================================================================
    // SKELETON HOOK  act()
    // =========================================================================

    @Override
    protected void act(TWThought t) {
        try {
            switch (t.getAction()) {
                case MOVE:
                    move(t.getDirection());
                    break;
                case PICKUP:
                    Object p = getEnvironment().getObjectGrid().get(getX(), getY());
                    if (p instanceof TWTile) pickUpTile((TWTile) p);
                    break;
                case PUTDOWN:
                    Object d = getEnvironment().getObjectGrid().get(getX(), getY());
                    if (d instanceof TWHole) putTileInHole((TWHole) d);
                    break;
                case REFUEL:
                    refuel();
                    break;
            }
        } catch (CellBlockedException e) {
            path = null; // invalidate cached path on block
        }
    }

    // =========================================================================
    // PATROL
    // =========================================================================

    private TWDirection patrol() {
        int[] corner = corners[cornerIdx];

        // Layer 2: per-step fuel safety check (ANCHORED mode only)
        // Abort toward fuel station if we cannot reach the corner AND return safely.
        if (anchoredMode) {
            double distToCorner     = manhattan(getX(), getY(), corner[0], corner[1]);
            double distCornerToFuel = manhattan(corner[0], corner[1], fuelStationX, fuelStationY);
            double fuelFloor        = FUEL_THRESHOLD * Parameters.defaultFuelLevel;
            if ((fuelLevel - distToCorner - distCornerToFuel) < fuelFloor)
                return navigate(fuelStationX, fuelStationY);
        }

        // Reached current corner advance to next
        if (getX() == corner[0] && getY() == corner[1]) {
            cornersThisCycle++;
            cornerIdx = (cornerIdx + 1) % corners.length;
            path      = null;
            corner    = corners[cornerIdx];

            // Layer 2 check for next corner too
            if (anchoredMode) {
                double distToNext   = manhattan(getX(), getY(), corner[0], corner[1]);
                double distNextFuel = manhattan(corner[0], corner[1], fuelStationX, fuelStationY);
                double fuelFloor    = FUEL_THRESHOLD * Parameters.defaultFuelLevel;
                if ((fuelLevel - distToNext - distNextFuel) < fuelFloor)
                    return navigate(fuelStationX, fuelStationY);
            }
        }

        return navigate(corner[0], corner[1]);
    }

    // =========================================================================
    // NAVIGATION
    // =========================================================================

    private TWDirection navigate(int tx, int ty) {
        if (tx != targX || ty != targY) { path = null; targX = tx; targY = ty; }
        if (path == null || !path.hasNext()) path = astar.findPath(getX(), getY(), tx, ty);
        if (path != null && path.hasNext()) {
            TWDirection d = path.popNext().getDirection();
            if (d != TWDirection.Z) return d;
        }
        return directStep(tx, ty);
    }

    private TWDirection directStep(int tx, int ty) {
        int dx = tx - getX(), dy = ty - getY();
        if (Math.abs(dx) >= Math.abs(dy)) return dx > 0 ? TWDirection.E : TWDirection.W;
        return dy > 0 ? TWDirection.S : TWDirection.N;
    }

    // =========================================================================
    // HELPERS
    // =========================================================================

    /**
     * True if (x,y) lies inside the active patrol box.
     * FULL_PERIMETER: always true.
     * FUEL_ANCHORED:  must be within [corners[0], corners[2]] bounding box.
     */
    private boolean isInsidePatrolBox(int x, int y) {
        if (!anchoredMode) return true;
        return x >= corners[0][0] && x <= corners[2][0]
            && y >= corners[0][1] && y <= corners[2][1];
    }

    /** Index of the corner in cornerSet closest (Manhattan) to (x, y). */
    private int nearestCornerIdx(int x, int y, int[][] cornerSet) {
        int    best     = 0;
        double bestDist = Double.MAX_VALUE;
        for (int i = 0; i < cornerSet.length; i++) {
            double d = manhattan(x, y, cornerSet[i][0], cornerSet[i][1]);
            if (d < bestDist) { bestDist = d; best = i; }
        }
        return best;
    }

    /**
     * Returns the closest position from a skeleton shared-location list that is
     * also inside the active patrol box, or null if none qualifies.
     */
    private int[] nearestSharedInBox(List<int[]> locations, int ax, int ay) {
        int[]  best     = null;
        double bestDist = Double.MAX_VALUE;
        for (int[] loc : locations) {
            if (!isInsidePatrolBox(loc[0], loc[1])) continue;
            double d = manhattan(ax, ay, loc[0], loc[1]);
            if (d < bestDist) { bestDist = d; best = loc; }
        }
        return best;
    }
}
