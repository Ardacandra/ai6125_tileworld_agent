/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package tileworld.planners;

import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.agent.BenTWAgent;
import tileworld.environment.TWDirection;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;

/**
 * @author Wong De Shun
 */
public class BenTWPlanner implements TWPlanner {
	
	private BenTWAgent agent;
	private AstarPathGenerator pathGenerator;
	
	private TWPath curPlan;
	private Int2D curGoal;
	private int stepsWithTile;
	
	public BenTWPlanner(BenTWAgent agent) {
		this.agent = agent;
		this.pathGenerator = new AstarPathGenerator(
				agent.getEnvironment(), 
				agent, 
				agent.getEnvironment().getxDimension() * agent.getEnvironment().getyDimension());
		this.curPlan = null;
		this.curGoal = null;
		this.stepsWithTile = 0;
	}

    public TWPath generatePlan() {
    	if (curGoal == null || reachedGoal()) {
        	//curPlan = null;
        	//return null;
    		curGoal = generateGoal();
        }
        curPlan = pathGenerator.findPath(agent.getX(), agent.getY(), curGoal.x, curGoal.y);
        return curPlan;
    }
    
    private boolean reachedGoal() {
    	return (agent.getX() == curGoal.x && agent.getY() == curGoal.y);
    }

    public boolean hasPlan() {
        return curPlan != null && curPlan.hasNext();
    }

    public void voidPlan() {
        curPlan = null;
        curGoal = null;
    }

    public Int2D getCurrentGoal() {
        return curGoal;
    }

    public TWDirection execute() {
        if (planInvalid()) {
        	System.out.println("REPLAN from Agent:" + agent.getName() + " (" + agent.getX() + "," + agent.getY() + "), FUEL:"+agent.knowsFuel()+", SCORE:"+agent.getScore());
        	generatePlan();
        	System.out.println("REPLAN from (" + agent.getX() + "," + agent.getY() + "), FUEL:"+agent.knowsFuel()+", SCORE:"+agent.getScore());
        }
        if (curPlan == null || !curPlan.hasNext()) {
        	System.out.println("FALLBACK RANDOM MOVE at (" + agent.getX() + "," + agent.getY() + "), (TILE:"+agent.getNumTiles()+")");
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
    
    private boolean planInvalid() {
    	if (curPlan == null || curGoal == null) return true;
    	if (agent.getX() == curGoal.x && agent.getY() == curGoal.y) return true;
    	if (agent.getMemory().isCellBlocked(curGoal.x, curGoal.y)) return true;
    	return false;
    }
    
    private Int2D generateGoal() {
    	int x = agent.getX();
    	int y = agent.getY();
    	
    	//Refuel as priority to prevent stalling
    	if (agent.criticalFuel() && agent.knowsFuel()) {
    		TWFuelStation station = agent.getFuelStation();
    		return new Int2D(station.getX(), station.getY());
    	}
    	
    	if (!agent.knowsFuel()) {
    		Int2D goal = frontierGoal(4);
    	    System.out.println("FRONTIER SWEEP MODE from (" + goal.getX() + "," + goal.getY() + ")");
    	    if (goal != null) return goal;
    	}
    	
    	//Prefer filling hole when full or is holding a tile 
    	if (agent.isFull() || agent.hasTile()) {
    		TWHole hole = agent.getMemory().getNearbyHole(x, y, 10);
    		if (hole != null && holeExists(hole)) {
    			System.out.println("TARGET HOLE from (" + hole.getX() + "," + hole.getY() + ")");
    			return new Int2D(hole.getX(), hole.getY());
    		}
    		
            Int2D frontier = frontierGoal(6);
            if (frontier != null) {
                System.out.println("EXPLORE HOLE FRONTIER from (" + x + "," + y + ") -> (" + frontier.x + "," + frontier.y + ")");
                return frontier;
            }
    		
			System.out.println("EXPLORE HOLE RANDOM from (" + agent.getX() + "," + agent.getY() + ")");
    		return newExpGoal(10);
    	}
    	
    	//Else if not full, look for tile
    	if (!agent.isFull()) {
    		TWTile tile = agent.getMemory().getNearbyTile(x, y, 10);
    		if (tile != null && tileExists(tile)) {
    			System.out.println("TARGET TILE from (" + tile.getX() + "," + tile.getY() + ")");
    			return new Int2D(tile.getX(), tile.getY());
    		}

            Int2D frontier = frontierGoal(6);
            if (frontier != null) {
                System.out.println("EXPLORE TILE FRONTIER from (" + x + "," + y + ") -> (" + frontier.x + "," + frontier.y + ")");
                return frontier;
            }
            
    		System.out.println("EXPLORE TILE RANDOM from (" + agent.getX() + "," + agent.getY() + ")");
    		return newExpGoal(10);
    	}

        Int2D frontier = frontierGoal(6);
        if (frontier != null) {
            System.out.println("EXPLORE NEW FRONTIER GOAL from (" + x + "," + y + ") -> (" + frontier.x + "," + frontier.y + ")");
            return frontier;
        }
    	
    	System.out.println("EXPLORE NEW RANDOM GOAL from (" + agent.getX() + "," + agent.getY() + ")");
    	Int2D expGoal = newExpGoal(6);
    	return expGoal;
    }
    
    private boolean inMyRegion(int x, int y) {
        int w = agent.getEnvironment().getxDimension();
        boolean agentLeft = agent.getName().toLowerCase().contains("1");

        if (agentLeft) {
            return x < w / 2;
        } else {
            return x >= w / 2;
        }
    }
    
    private Int2D frontierGoal(int minDistance) {
        Int2D best = null;
        int bestScore = Integer.MIN_VALUE;

        int width = agent.getEnvironment().getxDimension();
        int height = agent.getEnvironment().getyDimension();

        for (int x = 1; x < width - 1; x++) {
            for (int y = 1; y < height - 1; y++) {
                if (!inMyRegion(x, y)) continue;
                if (agent.getMemory().isCellBlocked(x, y)) continue;
                if (!isFrontierCell(x, y)) continue;

                int dist = Math.abs(agent.getX() - x) + Math.abs(agent.getY() - y);
                if (dist < minDistance) continue;

                int score = 0;

                score += dist;

                // prefer cells adjacent to lots of unknown space
                score += unknownNeighborCount(x, y) * 5;

                if (score > bestScore) {
                    bestScore = score;
                    best = new Int2D(x, y);
                }
            }
        }

        return best;
    }
    
    private boolean isFrontierCell(int x, int y) {
        return isUnknown(x + 1, y) || isUnknown(x - 1, y)
            || isUnknown(x, y + 1) || isUnknown(x, y - 1);
    }

    private boolean isUnknown(int x, int y) {
        if (!agent.getEnvironment().isInBounds(x, y)) return false;
        return agent.getMemory().getMemoryGrid().get(x, y) == null;
    }

    private int unknownNeighborCount(int x, int y) {
        int count = 0;
        if (isUnknown(x + 1, y)) count++;
        if (isUnknown(x - 1, y)) count++;
        if (isUnknown(x, y + 1)) count++;
        if (isUnknown(x, y - 1)) count++;
        return count;
    }
        
    private Int2D newExpGoal(int distanceThres) {
    	Int2D bestExpGoal = null;
    	int bestScore = Integer.MIN_VALUE;
    	
    	for (int i = 0; i < 50; i++) {
    		int new_x = agent.getEnvironment().random.nextInt(agent.getEnvironment().getxDimension());
    		int new_y = agent.getEnvironment().random.nextInt(agent.getEnvironment().getyDimension());
    		
    		if (agent.getMemory().isCellBlocked(new_x, new_y)) continue;
    		
    		int distance = Math.abs(agent.getX() - new_x) + Math.abs(agent.getY() - new_y);
    		if (distance < distanceThres) continue;
    		
    		int score = distance;
    		if (agent.getMemory().getMemoryGrid().get(new_x, new_y) == null) {
    			score += distanceThres;
    		}
    		if (score > bestScore) {
    			bestScore = score;
    			bestExpGoal = new Int2D(new_x, new_y);
    		}
    	}
    	
    	return bestExpGoal;
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
    
    private boolean holeExists(TWHole hole) {
        if (hole == null) return false;

        Object obj = agent.getEnvironment().getObjectGrid().get(hole.getX(), hole.getY());
        if (obj instanceof TWHole) {
            return true;
        }

        // stale memory entry
        agent.getMemory().removeObject(hole);
        return false;
    }
    
    private boolean tileExists(TWTile tile) {
        if (tile == null) return false;

        Object obj = agent.getEnvironment().getObjectGrid().get(tile.getX(), tile.getY());
        if (obj instanceof TWTile) {
            return true;
        }

        // stale memory entry
        agent.getMemory().removeObject(tile);
        return false;
    }

}

