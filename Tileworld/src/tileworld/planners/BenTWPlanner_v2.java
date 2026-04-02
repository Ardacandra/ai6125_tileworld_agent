/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package tileworld.planners;

import java.util.ArrayList;
import java.util.Random;

import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.agent.BenTWAgent;
import tileworld.agent.BenTWAgent_v2;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;

/**
 * @author Wong De Shun
 */
public class BenTWPlanner_v2 implements TWPlanner {
	
	private BenTWAgent_v2 agent;
	private AstarPathGenerator pathGenerator;
	
	private ArrayList<Int2D> blacklistCells; //unreachable cells, but not obstacles

	private Int2D expGoal; //exploration
	private Int2D nearGoal; //within sensory range
	private String goalType;
	private TWPath curPlan; //current path generated based on goals

    protected static final String ENTITY_TILE = "tile";
    protected static final String ENTITY_HOLE = "hole";
    protected static final String ENTITY_FUEL = "fuel";
    protected static final String ENTITY_DELETE_TILE = "delete_tile";
    protected static final String ENTITY_DELETE_HOLE = "delete_hole";

	public BenTWPlanner_v2(BenTWAgent_v2 agent) {
		this.agent = agent;
		this.pathGenerator = new AstarPathGenerator(
				agent.getEnvironment(), 
				agent, 
				agent.getEnvironment().getxDimension() * agent.getEnvironment().getyDimension());
		this.expGoal = null;
		this.nearGoal = null;
		this.curPlan = null;
		this.goalType = "";
		blacklistCells = new ArrayList<Int2D>();
	}
	
    public TWDirection execute() {
    	//reposition far and near goals
    	generateGoals();
    	//generate plan based on expGoal/nearGoal
    	generatePlan();
    	//pop next direction from plan
    	return generateDirFromPlan();
    }
    
    private void generateGoals() {
    	if (agent.criticalFuel()) { //head to refuel
    		expGoal = nearGoal = agent.getFuelStationLocation();
    		goalType = ENTITY_FUEL;
    		return;
    	}
    	if (isAtExpGoal()) { //reached exp goal
    		expGoal = null;
    		agent.debugPrint("REACHED EXP GOAL");
    	}
    	if (isAtNearGoal()) { //reached near goal
    		nearGoal = null;
    		agent.debugPrint("REACHED NEAR GOAL");
    	}
    	if (isInvalidCell(expGoal)) { //invalid exp goal
    		expGoal = null;
    		agent.debugPrint("INVALID EXP GOAL");
    	}
    	if (isInvalidCell(nearGoal)) { //invalid near goal
    		nearGoal = null;
    		agent.debugPrint("INVALID NEAR GOAL");
    	}
    	if (expGoal == null) {
    		//generate a exp goal for exploration, nearest unexplored area in memory
    		expGoal = generateExpGoal();
    		agent.debugPrint("NEW "+goalType+" EXP GOAL GENERATED:" + expGoal.x + "," + expGoal.y);
    	}
    	if (nearGoal == null) {
    		//generate a near goal for pickup/putdown, where relevant
    		nearGoal = generateNearGoal();
    		if (nearGoal != null) {
    			agent.debugPrint("NEW "+goalType+" NEAR GOAL GENERATED:" + nearGoal.x + "," + nearGoal.y);
    		}
    	}
    }
    
    private boolean isInvalidCell(Int2D goal) {
    	if (goal == null) return false;
    	int x = goal.x;
    	int y = goal.y;
    	if (!agent.getEnvironment().isInBounds(x, y)) return true; //out of bounds
        if (agent.getMemory().isCellBlocked(x, y)) return true; //cell blocked
        if (isCellBlacklisted(x, y)) return true; //blacklisted (inaccessible)
        return false;    	
    }
    
    private boolean isAtExpGoal() {
    	if (expGoal == null) return false;
    	return (agent.getX() == expGoal.x) && (agent.getY() == expGoal.y);
    }
    
    private boolean isAtNearGoal() {
    	if (nearGoal == null) return false;
    	return (agent.getX() == nearGoal.x) && (agent.getY() == nearGoal.y);
    }
    
    public Int2D getExpGoal() {
    	return expGoal;
    }
    
    public Int2D getNearGoal() {
    	return nearGoal;
    }
    
    private Int2D generateExpGoal() {
        Int2D best = null;
        int bestScore = Integer.MIN_VALUE;

        int width = agent.getEnvironment().getxDimension();
        int height = agent.getEnvironment().getyDimension();
        
        ArrayList<Int2D> entities = new ArrayList<Int2D>();

        for (int x = 1; x <= width - 1; x++) {
            for (int y = 1; y <= height - 1; y++) {
            	//skip blocked cells
                if (agent.getMemory().isCellBlocked(x, y)) continue;
                if (isCellBlacklisted(x, y)) continue;
                
                int expScore = regionExpScore(x, y);
                int dist = manhattanDist(agent.getX(), agent.getY(), x, y);
                if (dist < 14) continue; //minDistance applied
                
                int score = expScore - dist;
                if (score > bestScore) {
                    entities = new ArrayList<Int2D>();
                    bestScore = score;
                    best = new Int2D(x, y);
                }
                if (score == bestScore) {
                	entities.add(best);
                }
            }
        }
        Random random = new Random();
        return entities.get(random.nextInt(entities.size()));
    }
    
    private Int2D generateNearGoal() {
        Int2D bestHole = null;
        int bestHoleScore = Integer.MAX_VALUE;
        ArrayList<Int2D> hole_entities = new ArrayList<Int2D>();
        Int2D bestTile = null;
        int bestTileScore = Integer.MAX_VALUE;
        ArrayList<Int2D> tile_entities = new ArrayList<Int2D>();
        
        int xPos = agent.getX();
        int yPos = agent.getY();
        
        int rng = Parameters.defaultSensorRange;
    	//loop sensor range and assign score to unknown cells
    	for (int x = xPos - rng; x < xPos + rng; x++) {
            for (int y = yPos - rng; y < yPos + rng; y++) {
            	if (!agent.getEnvironment().isInBounds(x, y)) continue;
                if (agent.getMemory().isCellBlocked(x, y)) continue;
                if (isCellBlacklisted(x, y)) continue;
                
                int dist = manhattanDist(x, y, xPos, yPos);
                
            	Object obj = agent.getMemory().getMemoryGrid().get(x, y);
            	if (obj instanceof TWHole && agent.hasTiles()) {
            		if (dist < bestHoleScore) {
                    	hole_entities = new ArrayList<Int2D>();
                    	bestHoleScore = dist;
                    	bestHole = new Int2D(x, y);
            		}
                    if (dist == bestHoleScore) {
                    	hole_entities.add(bestHole);
                    }
            	}
            	if (obj instanceof TWTile && !agent.isTilesFull()) {
                    if (dist < bestTileScore) {
                    	tile_entities = new ArrayList<Int2D>();
                    	bestTileScore = dist;
                    	bestTile = new Int2D(x, y);
                    }
            		if (dist == bestTileScore) {
            			tile_entities.add(bestTile);
            		}
            	}
            }
    	}
    	
    	Random random = new Random();
    	if (bestHole != null) {
    		goalType = ENTITY_HOLE;
    		return hole_entities.get(random.nextInt(hole_entities.size()));
    	}
    	if (bestTile != null) {
    		goalType = ENTITY_TILE;
    		return tile_entities.get(random.nextInt(tile_entities.size()));
    	}
    	return null;
    }
    
    private int regionExpScore(int xPos, int yPos) {
    	int score = 0;
    	int rng = Parameters.defaultSensorRange;
    	//loop sensor range and assign score to unknown cells
    	for (int x = xPos - rng; x < xPos + rng; x++) {
            for (int y = yPos - rng; y < yPos + rng; y++) {
            	score += cellExpScore(x, y);
            }
    	}
    	return score;
    }

    private int cellExpScore(int x, int y) {
        if (!agent.getEnvironment().isInBounds(x, y)) return 0; //out of bounds
        if (agent.getMemory().isCellBlocked(x, y)) return 0; //cell blocked
        if (agent.getMemory().getMemoryGrid().get(x, y) != null) return 0; //in-memory already
        return 1; //not in-memory (fresh area, new or decayed)
    }
    
    private boolean isCellBlacklisted(int x, int y) {
    	for (int i=0; i<blacklistCells.size();i++) {
    		if (blacklistCells.get(i).getX() == x && blacklistCells.get(i).getY() == y) {
    			return true;
    		}
    	}
    	return false;
    }
    
    private int manhattanDist(int x1, int y1, int x2, int y2) {
        return Math.abs(x1 - x2) + Math.abs(y1 - y2);
    }

    public TWPath generatePlan() {
    	if (expGoal == null) return null; //always to have an exploration goal
    	if (nearGoal != null) { //valid nearGoal
    		curPlan = pathGenerator.findPath(agent.getX(), agent.getY(), nearGoal.x, nearGoal.y);
    		if (curPlan != null) {
    			agent.setSuperIntention(goalType, nearGoal.x, nearGoal.y);
    			agent.debugPrint("NEAR GOAL PLAN:" + nearGoal.x + "," + nearGoal.y);
    			return curPlan;
    		}
    		//curPlan == null, blacklist goal spot, unreachable
    		blacklistCells.add(nearGoal);
    		nearGoal = null;
    	}
    	//invalid nearGoal, valid expGoal
    	curPlan = pathGenerator.findPath(agent.getX(), agent.getY(), expGoal.x, expGoal.y);
    	if (curPlan != null) {
			agent.setSuperIntention(goalType, expGoal.x, expGoal.y);
			agent.debugPrint("EXP GOAL PLAN:" + expGoal.x + "," + expGoal.y);
			return curPlan;
		}
    	//invalid expGoal
    	blacklistCells.add(expGoal);
    	expGoal = null;
        return curPlan;
    }
    
    private TWDirection generateDirFromPlan() {
    	if (curPlan == null || !curPlan.hasNext()) {
    		agent.debugPrint("PLAN IS NULL, RANDOM MOVE");
	    	return getMove();
	    }
	    while (curPlan.hasNext()) {
	    	TWPathStep step = curPlan.popNext();
	    	if (step.getDirection() != TWDirection.Z) {
	    		return step.getDirection();
	    	}
	    }
	    return TWDirection.Z;
    }

    public void voidPlan() {
        curPlan = null;
        expGoal = null;
        nearGoal = null;
    }
    
    public TWDirection getMove() {
    	TWDirection[] dirArray = new TWDirection[] {
    		TWDirection.N, TWDirection.S, TWDirection.E, TWDirection.W	
    	};
    	
    	for (int i = 0; i < dirArray.length; i++) {
    		TWDirection d = dirArray[agent.getEnvironment().random.nextInt(dirArray.length)];
    		int new_x = agent.getX() + d.dx;
    		int new_y = agent.getY() + d.dy;
    		
    		if (agent.getEnvironment().isInBounds(new_x, new_y) 
    				&& !agent.getMemory().isCellBlocked(new_x, new_y)) {
    			return d;
    		}
    	}
    	return TWDirection.Z;
    }

	@Override
	public boolean hasPlan() {
		return curPlan != null;
	}

	@Override
	public Int2D getCurrentGoal() {
		return expGoal;
	}

}