package tileworld.agent;

import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.BenTWPlanner_v2;

public class BenTWAgent_v2 extends TWAgentSkeleton {
	
	private boolean DEBUG_LOGGING = false;
	
	private BenTWAgentWorkingMemory_v2 customMemory;
	private BenTWPlanner_v2 planner;
	
	public BenTWAgent_v2(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(name, xpos, ypos, env, fuelLevel);
        this.customMemory = new BenTWAgentWorkingMemory_v2(this, env.schedule, env.getxDimension(), env.getyDimension());
        this.planner = new BenTWPlanner_v2(this);
    }
	
	/** Implemented by subclass to handle Phase 2 communication */
    protected void customCommunicate() {}

    /** Implemented by subclass to handle custom message types (tiles, holes, intents) */
    protected void handleTeamMessage(ArdaMessage msg) {}

    /** Implemented by subclass to handle Phase 2 decision making */
    protected TWThought customThink() {
    	sense();
    	//customCommunicate();
    	//handleTeamMessage(msg);
    	
    	//If standing on fuel station, and fuel below threshold, refuel
    	if (shouldRefuel() && knowsFuelStation() && getEnvironment().inFuelStation(this)) {
    		debugPrint("REFUEL THOUGHT");
    		return new TWThought(TWAction.REFUEL, TWDirection.Z);
    	}
    	
    	// If standing on a hole, agent has tiles
    	if (hasTiles() && getHole() != null) {
    		debugPrint("PUTDOWN TILE THOUGHT");
    		return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
    	}
    	
    	// If standing on a tile, agent not fully filled, and fuel not critical, pickup
    	if (!isTilesFull() && getTile() != null && !criticalFuel()) {
    		debugPrint("PICKUP TILE THOUGHT");
    		return new TWThought(TWAction.PICKUP, TWDirection.Z);
    	}
    	
    	TWDirection move = planner.execute();
    	if (move == null || move == TWDirection.Z) {
    		move = planner.getMove();
    		debugPrint("RANDOM MOVE THOUGHT:" + move);
    	}
    	
    	return new TWThought(TWAction.MOVE, move);
    }
    
    protected void act(TWThought thought) {
		if (thought == null) return;
    	try {
    		switch (thought.getAction()) {
    		case PICKUP:
    			TWTile tile = getTile();
    			if (tile != null) {
    				pickUpTile(tile);
    	    		debugPrint("PICKUP ACT");
    				memory.removeObject(tile);
    			}
    			break;
    		case PUTDOWN:
    			TWHole hole = getHole();
    			if (hole != null) {
    				putTileInHole(hole);
    	    		debugPrint("PUTDOWN ACT");
    				memory.removeObject(hole);
    			}
    			break;
    		case REFUEL:
				this.refuel();
	    		debugPrint("REFUEL ACT");
    			break;
    		case MOVE:
    		default:
	    		debugPrint("MOVE ACT:" + thought.getDirection());
    			this.move(thought.getDirection());
    			break;
    		}
    	}
    	catch (CellBlockedException e) {
    		debugPrint("CELL BLOCKED ACT");
    	}
    }
    
    private TWTile getTile() {
        Object obj = getEnvironment().getObjectGrid().get(this.getX(), this.getY());
        if (obj instanceof TWTile) {
        	return (TWTile) obj;
        }
        return null;
    }
    
    private TWHole getHole() {
        Object obj = getEnvironment().getObjectGrid().get(this.getX(), this.getY());
        if (obj instanceof TWHole) {
        	return (TWHole) obj;
        }
        return null;
    }
    
    // Am I too full to refuel? prevent refuel spam
    private boolean shouldRefuel() {
    	return this.getFuelLevel() < Parameters.defaultFuelLevel * 0.75;
    }
    
    // Am I too low? ignore other tasks and get straight to the fuel station
    public boolean criticalFuel() {
    	return this.getFuelLevel() < (int) getEnvironment().getxDimension() * 2.1;
    }
    
    private boolean knowsFuelStation() {
    	return (fuelStationX > 0) && (fuelStationY > 0);
    }
    
    public boolean isTilesFull() {
    	return carriedTiles.size() >= 3;
    }
    
    public boolean hasTiles() {
    	return carriedTiles.size() > 0;
    }
    
    public Int2D getFuelStationLocation() {
    	return new Int2D(fuelStationX, fuelStationY);
    }
    
    public void setSuperIntention(String entityType, int targetX, int targetY) {
    	setIntention(entityType, targetX, targetY);
    }
    
    public void debugPrint(String msg) {
    	if (DEBUG_LOGGING) {
    		System.out.println(this.getName()
				+ " STEP: " + this.getEnvironment().schedule.getSteps()
				+ " FUEL: " + this.getFuelLevel()
				+ " REWARD: " + this.getScore()
				+ " TILES: " + this.carriedTiles.size()
				+ " POS: " + this.getX() + "," + this.getY()
				+ " " + msg);
    	}
    }
    
    public BenTWAgentWorkingMemory_v2 getCustomMemory() {
    	return customMemory;
    }
}
