package tileworld.agent;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ThreadLocalRandom;
import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.environment.TWDirection;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.planners.AstarPathGenerator;
import tileworld.planners.TWPath;
import tileworld.planners.TWPathStep;

/**
 * Phase1Strategy - Shared fuel station discovery for all 6 agents.
 *
 * Phase 1 divides the map into 6 zones (2 cols x 3 rows).
 * Each agent claims the zone nearest to its starting position (tie-break by agent name).
 * Agents then perform a sweep of their zone using sensor-based discovery.
 * The first agent to sense the fuel station broadcasts fuel entity position via ArdaMessage.
 * Thereafter all agents switch to Phase 2 and proceed with their own custom logic.
 *
 * Sample use in agent:
 *   private Phase1Strategy phase1 = new Phase1Strategy(this);
 *
 *   public void communicate() {
 *       phase1.communicate();
 *       // Agent in Phase 1 until fuel station found to ensure group survival
 *       if (!phase1.isComplete()) return;
 *       // ... agent's custom Phase 2 logic
 *   }
 *
 *   protected TWThought think() {
 *   	 // Agent in Phase 1 until fuel station found to ensure group survival
 *       if (!phase1.isComplete()) {
 *           TWThought t = phase1.think();
 *           if (t != null) return t;
 *       }
 *       // Agent in Phase 2, fuel station is found
 *       // ... agent's custom Phase 2 logic
 *   }
 */
public class Phase1Strategy {
    // ArdaMessage entity type constants
    private static final String ENTITY_AGENT_LOCATION   = "init";
    private static final String ENTITY_FUEL_STATION     = "fuel";
    // Exploration parameters
    private static final int COL_SPACING = Parameters.defaultSensorRange * 2;
    private static final int WAYPOINT_BUFFER = Parameters.defaultSensorRange;
    private static final double FUEL_SAFETY_RATIO = 0.05;

    // Agent and PathGenerator
    private final TWAgent agent;
    private AstarPathGenerator pathGen;

    // Zone assignment
    private boolean initSent;       	// init message sent at least once
    private boolean stratInitalized;    // zone assigned, sweep path built
    private int[] zoneBoundary;        	// [xMin, yMin, xMax, yMax]

    // Fuel station discovery
    private Int2D fuelStationPos;
    private boolean complete;

    // Sweep path
    private List<Int2D> sweepPath;
    private int sweepPathIdx;

    public Phase1Strategy(TWAgent agent) {
        this.agent = agent;
        this.initSent = false;
        this.stratInitalized = false;
        this.zoneBoundary = null;
        this.fuelStationPos = null;
        this.complete = false;
        this.sweepPath = null;
        this.sweepPathIdx = 0;
    }

    public boolean isComplete() {
        return complete;
    }

    public Int2D getFuelStation() {
        return fuelStationPos;
    }

    
    // Call this at the start of the agent's communicate() method.
    // Broadcasts init on the very first call to enable zone assignment.
    // Broadcasts fuel every step after the fuel station is discovered.
    public void communicate() {
        if (!initSent) {
            initSent = true;
            agent.getEnvironment().receiveMessage(
                    ArdaMessage.info(agent.getName(), ENTITY_AGENT_LOCATION,
                            agent.getX(), agent.getY(),
                            agent.getX(), agent.getY()));
        } else if (complete && fuelStationPos != null) {
            agent.getEnvironment().receiveMessage(
                    ArdaMessage.info(agent.getName(), ENTITY_FUEL_STATION,
                            fuelStationPos.x, fuelStationPos.y,
                            agent.getX(), agent.getY()));
        }
    }

    // Call this instead of the agent's custom think() during Phase 1.
    // Returns a TWThought with the next Phase 1 action, or null if Phase 1 is completed
    public TWThought think() {
        // Check if fuel station has entered sensor range
        detectFuelStation();
        if (complete) return null;

        // Check for fuel broadcasts from other agents
        processMessages();
        if (complete) return null;

        // Assign zone and build sweep paths
        if (!stratInitalized) {
            assignZone();
            buildSweepPaths();
            stratInitalized = true;
        }

        // Opportunistic in-place actions without detour
        // There exist a pickup & putdown opportunity in the current position
        Object curPos = agent.getEnvironment().getObjectGrid()
                .get(agent.getX(), agent.getY());
        if (curPos instanceof TWTile && agent.carriedTiles.size() < 3) {
            return new TWThought(TWAction.PICKUP, TWDirection.Z);
        }
        if (curPos instanceof TWHole && agent.hasTile()) {
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        }

        // Fuel safety: if critically low and station still unknown, head toward center and wait
        if (agent.getFuelLevel() < Parameters.defaultFuelLevel * FUEL_SAFETY_RATIO) {
            int cx = agent.getEnvironment().getxDimension() / 2;
            int cy = agent.getEnvironment().getyDimension() / 2;
            return new TWThought(TWAction.MOVE, getPathDirection(cx, cy));
        }

        // Execute sweep path
        return new TWThought(TWAction.MOVE, nextSweepDirection());
    }

    // Fuel station detection
    private void detectFuelStation() {
        if (complete) return;
        int range = Parameters.defaultSensorRange;
        int xPos = agent.getX();
        int yPos = agent.getY();
        // Scan sensor block
        for (int deltaX = -range; deltaX <= range; deltaX++) {
            for (int deltaY = -range; deltaY <= range; deltaY++) {
                int scanX = xPos + deltaX;
                int scanY = yPos + deltaY;
                if (!agent.getEnvironment().isInBounds(scanX, scanY)) continue;
                Object obj = agent.getEnvironment().getObjectGrid().get(scanX, scanY);
                if (obj instanceof TWFuelStation) {
                    fuelStationPos = new Int2D(scanX, scanY);
                    complete = true;
                    System.out.println(agent.getName() + " [Phase1] Sensed fuel station at ("
                            + scanX + "," + scanY + ")");
                    return;
                }
            }
        }
    }

    // Process received messages from broadcast
    private void processMessages() {
        List<Message> messages = agent.getEnvironment().getMessages();
        for (Message m : messages) {
            if (agent.getName().equals(m.getFrom())) continue;
            if (!(m instanceof ArdaMessage)) continue;
            ArdaMessage amsg = (ArdaMessage) m;
            if (ENTITY_FUEL_STATION.equals(amsg.getEntityType()) && !complete) {
                fuelStationPos = new Int2D(amsg.getX(), amsg.getY());
                complete = true;
                System.out.println(agent.getName()
                        + " [Phase1] Received fuel station from "
                        + m.getFrom() + " at (" + amsg.getX() + "," + amsg.getY() + ")");
            }
        }
    }

    // Greedily assign each agent to its closest unclaimed zone
    // Reads broadcasted init messages containing starting positions of all agents
    // Ties are broken by agent name (lower = win)
    private void assignZone() {
        int mapW = agent.getEnvironment().getxDimension();
        int mapH = agent.getEnvironment().getyDimension();

        // Compute zone boundaries (2 cols x 3 rows = 6 zones)
        int zoneW = mapW / 2;
        int zoneH = mapH / 3;

        int[] xMinArr = new int[6];
        int[] yMinArr = new int[6];
        int[] xMaxArr = new int[6];
        int[] yMaxArr = new int[6];
        int[][] centers = new int[6][2];
        
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 2; col++) {
                int idx = row * 2 + col;
                int xMin = col * zoneW;
                int yMin = row * zoneH;
                int xMax = (col == 1) ? mapW - 1 : zoneW - 1;
                int yMax = (row == 2) ? mapH - 1 : (row + 1) * zoneH - 1;
                xMinArr[idx] = xMin; yMinArr[idx] = yMin;
                xMaxArr[idx] = xMax; yMaxArr[idx] = yMax;
                centers[idx][0] = (xMin + xMax) / 2;
                centers[idx][1] = (yMin + yMax) / 2;
            }
        }

        // Collect starting positions from init messages + self
        final Map<String, int[]> agentsPos = new HashMap<String, int[]>();
        agentsPos.put(agent.getName(), new int[]{agent.getX(), agent.getY()});

        List<Message> messages = agent.getEnvironment().getMessages();
        for (Message m : messages) {
            if (!(m instanceof ArdaMessage)) continue;
            ArdaMessage amsg = (ArdaMessage) m;
            if (ENTITY_AGENT_LOCATION.equals(amsg.getEntityType())) {
                agentsPos.put(m.getFrom(), new int[]{amsg.getX(), amsg.getY()});
            }
        }

        // Sort agent names for tie break
        final List<String> agentNames = new ArrayList<String>(agentsPos.keySet());
        Collections.sort(agentNames);
        final int numAgents = agentNames.size();

        // Build all (agentIdx, zoneIdx, manhattanDist) triples
        List<int[]> triples = new ArrayList<int[]>();
        for (int ai = 0; ai < numAgents; ai++) {
            int[] start = agentsPos.get(agentNames.get(ai));
            for (int zi = 0; zi < 6; zi++) {
                int dist = Math.abs(start[0] - centers[zi][0])
                         + Math.abs(start[1] - centers[zi][1]);
                triples.add(new int[]{ai, zi, dist});
            }
        }

        // Sort primarily by distance, then by agent name (tie, lower name win)
        Collections.sort(triples, new Comparator<int[]>() {
            public int compare(int[] a, int[] b) {
                if (a[2] != b[2]) return Integer.compare(a[2], b[2]);
                return Integer.compare(a[0], b[0]);
            }
        });

        // Greedy assignment, each agent and zone claimed at most once
        boolean[] agentDone = new boolean[numAgents];
        boolean[] zoneDone  = new boolean[6];
        Map<String, Integer> assigned = new HashMap<String, Integer>();

        for (int[] trip : triples) {
            int ai = trip[0], zi = trip[1];
            if (!agentDone[ai] && !zoneDone[zi]) {
                agentDone[ai] = true;
                zoneDone[zi]  = true;
                assigned.put(agentNames.get(ai), zi);
            }
        }

        // Determine this agent's zone
        Integer myZoneIdx = assigned.get(agent.getName());
        if (myZoneIdx == null) {
            // Fallback to pick any unclaimed zone
            for (int zi = 0; zi < 6; zi++) {
                if (!zoneDone[zi]) { myZoneIdx = zi; break; }
            }
            if (myZoneIdx == null) myZoneIdx = 0;
        }

        zoneBoundary = new int[]{
            xMinArr[myZoneIdx], yMinArr[myZoneIdx],
            xMaxArr[myZoneIdx], yMaxArr[myZoneIdx]
        };

        System.out.println(agent.getName() + " [Phase1] Zone " + myZoneIdx
                + " [(" + zoneBoundary[0] + "," + zoneBoundary[1] + ")-(" + zoneBoundary[2] + "," + zoneBoundary[3] + ")]"
                + "  numAgentsInMsg=" + agentsPos.size());
    }

    // Sweep path creation (Boustrophedon)
    // Builds an ordered list of sweep paths within assigned zone.
    // Takes into consideration sensor range COL_SPACING to reduce waste
    private void buildSweepPaths() {
        sweepPath = new ArrayList<Int2D>();
        int xMin = zoneBoundary[0], yMin = zoneBoundary[1], xMax = zoneBoundary[2], yMax = zoneBoundary[3];

        boolean goingDown = true;

        // Collect all x-positions at COL_SPACING intervals, always include xMax
        List<Integer> cols = new ArrayList<Integer>();
        for (int x = xMin; x < xMax; x += COL_SPACING) {
            cols.add(x);
        }
        // Ensure the last column (xMax) is always included
        if (cols.isEmpty() || cols.get(cols.size() - 1) != xMax) {
            cols.add(xMax);
        }

        for (int colX : cols) {
            if (goingDown) {
                sweepPath.add(new Int2D(colX, yMin));
                sweepPath.add(new Int2D(colX, yMax));
            } else {
                sweepPath.add(new Int2D(colX, yMax));
                sweepPath.add(new Int2D(colX, yMin));
            }
            goingDown = !goingDown;
        }

        sweepPathIdx = 0;
        System.out.println(agent.getName() + " [Phase1] Sweep: "
                + cols.size() + " cols, " + sweepPath.size() + " waypoints");
    }

    // Return direction towards next sweep path step
    // Falls back to map-centre heading when all path steps are exhausted.
    private TWDirection nextSweepDirection() {
        if (sweepPath == null || sweepPath.isEmpty()) {
            return getRandomUnblockedDirection();
        }

        // Advance past any sweep path steps already within sensor range
        while (sweepPathIdx < sweepPath.size()) {
            Int2D wp = sweepPath.get(sweepPathIdx);
            int dist = Math.max(Math.abs(agent.getX() - wp.x),
                                    Math.abs(agent.getY() - wp.y));
            if (dist <= WAYPOINT_BUFFER) {
                sweepPathIdx++;
            } else {
                break;
            }
        }

        if (sweepPathIdx >= sweepPath.size()) {
            // Zone fully swept — keep moving randomly to avoid getting stuck
            return getRandomUnblockedDirection();
        }

        Int2D target = sweepPath.get(sweepPathIdx);
        return getPathDirection(target.x, target.y);
    }

    // Pathfinding helpers
    private TWDirection getPathDirection(int tx, int ty) {
        if (agent.getX() == tx && agent.getY() == ty) return TWDirection.Z;

        if (pathGen == null) {
            int maxNodes = agent.getEnvironment().getxDimension()
                         * agent.getEnvironment().getyDimension();
            pathGen = new AstarPathGenerator(agent.getEnvironment(), agent, maxNodes);
        }

        TWPath path = pathGen.findPath(agent.getX(), agent.getY(), tx, ty);
        if (path != null) {
            while (path.hasNext()) {
                TWPathStep step = path.popNext();
                if (step.getDirection() != TWDirection.Z) {
                    return step.getDirection();
                }
            }
        }

        return directStep(tx, ty);
    }

    private TWDirection directStep(int tx, int ty) {
        int deltaX = tx - agent.getX();
        int deltaY = ty - agent.getY();
        if (Math.abs(deltaX) >= Math.abs(deltaY)) {
            return deltaX > 0 ? TWDirection.E : TWDirection.W;
        }
        return deltaY > 0 ? TWDirection.S : TWDirection.N;
    }

    private TWDirection getRandomUnblockedDirection() {
        TWDirection[] dirs = {TWDirection.N, TWDirection.S, TWDirection.E, TWDirection.W};
        int start = ThreadLocalRandom.current().nextInt(4);
        for (int i = 0; i < 4; i++) {
            TWDirection d = dirs[(start + i) % 4];
            int newX = agent.getX() + d.dx;
            int newY = agent.getY() + d.dy;
            if (agent.getEnvironment().isInBounds(newX, newY)
                    && !agent.getMemory().isCellBlocked(newX, newY)) {
                return d;
            }
        }
        return TWDirection.Z;
    }
}
