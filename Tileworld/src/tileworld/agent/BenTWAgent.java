/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package tileworld.agent;

import java.util.ArrayList;

import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.BenTWPlanner;

/**
 * @author Wong De Shun
 */
public class BenTWAgent extends TWAgent {
	private String name;
	private BenTWPlanner planner;
	
	private Int2D fuelStation;
	private int stepCount;
	private boolean fuelBroadcasted;
	
    public BenTWAgent(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos,ypos,env,fuelLevel);
        this.name = name;
        this.planner = new BenTWPlanner(this);
        this.fuelStation = null;
        this.stepCount = 0;
        this.fuelBroadcasted = false;
    }

    protected TWThought think() {
    	//Sense Phase - ensure state of world is updated
    	sense();
    	stepCount++;
    	updateFuelStation();
    	communicate();
    	readMessage();
    	
    	//Handle pickup, putdown, refuel actions, shortcut the rest of thinking process
    	if (getTile() != null && carriedTiles.size() < 3) {
    		planner.voidPlan();
    		return new TWThought(TWAction.PICKUP, TWDirection.Z);
    	}
    	
    	if (getHole() != null && this.hasTile()) {
    		planner.voidPlan();
    		return new TWThought(TWAction.PUTDOWN, TWDirection.Z);
    	}
    	
    	if (fuelStation != null && shouldRefuel() && getEnvironment().inFuelStation(this)) {
    		planner.voidPlan();
    		return new TWThought(TWAction.REFUEL, TWDirection.Z);
    	}
    	
    	TWDirection move = planner.execute();
    	if (move == null || move == TWDirection.Z) {
    		planner.voidPlan();
    		move = planner.getMove();
    	}
    	
        return new TWThought(TWAction.MOVE, move);
    }

    @Override
    protected void act(TWThought thought) {
    	if (thought == null) return;
    	
    	try {
    		switch (thought.getAction()) {
    		case MOVE:
    			move(thought.getDirection());
    			break;
    		case PICKUP:
    			TWTile tile = getTile();
    			if (tile != null) {
    				pickUpTile(tile);
    				memory.removeObject(tile);
    			}
    			break;
    		case PUTDOWN:
    			TWHole hole = getHole();
    			if (hole != null) {
    				putTileInHole(hole);
    				memory.removeObject(hole);
    			}
    			break;
    		case REFUEL:
    			if (getEnvironment().inFuelStation(this)) {
        			refuel();
    			}
    			break;
    		}
    	}
    	catch (CellBlockedException e) {
    		planner.voidPlan();
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
    
    public TWFuelStation getFuelStation() {
        Object obj = getEnvironment().getObjectGrid().get(fuelStation.getX(), fuelStation.getY());
        if (obj instanceof TWFuelStation) {
        	return (TWFuelStation) obj;
        }
        return null;
    }

    private boolean shouldRefuel() {
    	return fuelLevel < Parameters.defaultFuelLevel * 0.5;
    }

    @Override
    public String getName() {
        return name;
    }

	public boolean criticalFuel() {
		return fuelLevel < Parameters.defaultFuelLevel * 0.25;
	}

	public boolean knowsFuel() {
		return fuelStation != null;
	}

	public boolean isFull() {
		return carriedTiles.size() >= 3;
	}
	
	public int getScore() {
		return this.score;
	}
	
	public int getNumTiles() {
		return carriedTiles.size();
	}
	
	public int getStepCount() {
	    return stepCount;
	}
	
	private void updateFuelStation() {
        int range = Parameters.defaultSensorRange;

        for (int dx = -range; dx <= range; dx++) {
            for (int dy = -range; dy <= range; dy++) {
                int nx = getX() + dx;
                int ny = getY() + dy;

                if (!getEnvironment().isInBounds(nx, ny)) {
                    continue;
                }

                if (Math.max(Math.abs(dx), Math.abs(dy)) > range) {
                    continue;
                }

                Object obj = getEnvironment().getObjectGrid().get(nx, ny);
                if (obj instanceof TWFuelStation) {
                    fuelStation = new Int2D(nx, ny);
                    return;
                }
            }
        }
    }
	
	@Override
	public void communicate() {
		if (fuelStation != null && !fuelBroadcasted) {
	        fuelBroadcasted = true;
	        String msg = "FUEL:" + fuelStation.x + "," + fuelStation.y;

	        Message message = new Message(
	            getName(),   // from
	            null,        // to (broadcast)
	            msg
	        );

	        getEnvironment().receiveMessage(message);
	        System.out.println(getName() + " BROADCAST FUEL at (" 
	            + fuelStation.x + "," + fuelStation.y + ")");
	    }
	}
	
	private void readMessage() {
		ArrayList<Message> messages = getEnvironment().getMessages();

		if (messages != null) {
		    for (Message m : messages) {

		        if (m.getFrom().equals(getName())) continue;

		        String msg = m.getMessage();

		        if (msg.startsWith("FUEL:")) {
		            String[] parts = msg.substring(5).split(",");

		            int x = Integer.parseInt(parts[0]);
		            int y = Integer.parseInt(parts[1]);

		            if (fuelStation == null) {
		                fuelStation = new Int2D(x, y);
		                System.out.println(getName() + 
		                    " learned fuel at (" + x + "," + y + ")");
		            }
		        }
		    }
		}
	}
}
