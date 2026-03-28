package tileworld.agent;

import tileworld.Parameters;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.AstarPathGenerator;
import tileworld.planners.TWPath;

public class HarshdeepPerimeterAgent extends TWAgent {

    private static final double FUEL_THRESHOLD = 0.25;
    private static final int    CARRY_MAX      = 3;
    private static final double DETOUR_LIMIT   = 10.0;

    private final String             name;
    private final AstarPathGenerator astar;

    private int    fuelStationX = -1, fuelStationY = -1;
    private TWPath path         = null;
    private int    targX        = -1, targY = -1;

    private final int[][] corners;
    private int cornerIdx = 0;

    public HarshdeepPerimeterAgent(String name, int x, int y, TWEnvironment env, double fuel) {
        super(x, y, env, fuel);
        this.name  = name;
        this.astar = new AstarPathGenerator(env, this, env.getxDimension() * env.getyDimension());

        int maxX = env.getxDimension() - 1;
        int maxY = env.getyDimension() - 1;
        this.corners = new int[][]{
            {0, 0}, {maxX, 0}, {maxX, maxY}, {0, maxY}
        };
    }

    // @Override
    // public void communicate() {
    //     int r = Parameters.defaultSensorRange;
    //     int ax = getX(), ay = getY();
    //     for (int dx = -r; dx <= r; dx++) {
    //         for (int dy = -r; dy <= r; dy++) {
    //             int cx = ax + dx, cy = ay + dy;
    //             if (!getEnvironment().isValidLocation(cx, cy)) continue;
    //             Object o = getEnvironment().getObjectGrid().get(cx, cy);
    //             if (o instanceof TWTile)
    //                 getEnvironment().receiveMessage(new ZoneSweepMessage(
    //                         name, "ALL", ZoneSweepMessage.Type.TILE, cx, cy));
    //             else if (o instanceof TWHole)
    //                 getEnvironment().receiveMessage(new ZoneSweepMessage(
    //                         name, "ALL", ZoneSweepMessage.Type.HOLE, cx, cy));
    //         }
    //     }
    // }

    @Override
    protected TWThought think() {
        locateFuelStation();

        int ax = getX(), ay = getY();

        // 1. Fuel
        if (fuelLevel <= Parameters.defaultFuelLevel * FUEL_THRESHOLD && fuelStationX != -1) {
            if (ax == fuelStationX && ay == fuelStationY)
                return new TWThought(TWAction.REFUEL, TWDirection.Z);
            return new TWThought(TWAction.MOVE, navigate(fuelStationX, fuelStationY));
        }

        // 2. In-place
        Object here = getEnvironment().getObjectGrid().get(ax, ay);
        if (here instanceof TWHole && hasTile())
            return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
        if (here instanceof TWTile && carriedTiles.size() < CARRY_MAX)
            return new TWThought(TWAction.PICKUP, TWDirection.Z);

        // 3. Opportunistic: close hole if carrying
        if (hasTile()) {
            TWHole h = (TWHole) memory.getClosestObjectInSensorRange(TWHole.class);
            if (h != null && manhattan(ax, ay, h.getX(), h.getY()) <= DETOUR_LIMIT)
                return new TWThought(TWAction.MOVE, navigate(h.getX(), h.getY()));
        }

        // 4. Opportunistic: close tile if not full
        if (carriedTiles.size() < CARRY_MAX) {
            TWTile t = (TWTile) memory.getClosestObjectInSensorRange(TWTile.class);
            if (t != null && manhattan(ax, ay, t.getX(), t.getY()) <= DETOUR_LIMIT)
                return new TWThought(TWAction.MOVE, navigate(t.getX(), t.getY()));
        }

        // 5. Advance perimeter patrol
        return new TWThought(TWAction.MOVE, patrol());
    }

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
            path = null;
        }
    }

    private TWDirection patrol() {
        // Advance to next corner if we've reached the current one
        int[] corner = corners[cornerIdx];
        if (getX() == corner[0] && getY() == corner[1]) {
            cornerIdx = (cornerIdx + 1) % corners.length;
            path = null;
            corner = corners[cornerIdx];
        }
        return navigate(corner[0], corner[1]);
    }

    // -----------------------------------------------------------------------
    // Navigation
    // -----------------------------------------------------------------------
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

    private double manhattan(int x1, int y1, int x2, int y2) {
        return Math.abs(x1 - x2) + Math.abs(y1 - y2);
    }

    private void locateFuelStation() {
        if (fuelStationX != -1) return;
        for (int x = 0; x < getEnvironment().getxDimension(); x++)
            for (int y = 0; y < getEnvironment().getyDimension(); y++)
                if (getEnvironment().getObjectGrid().get(x, y) instanceof TWFuelStation) {
                    fuelStationX = x; fuelStationY = y; return;
                }
    }

    @Override public String getName() { return name; }
}
