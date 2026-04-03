package tileworld.agent;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import javax.swing.text.html.HTMLDocument;
import sim.engine.Schedule;
import sim.field.grid.ObjectGrid2D;
import sim.util.Bag;
import sim.util.Int2D;
import sim.util.IntBag;
import tileworld.environment.NeighbourSpiral;
import tileworld.Parameters;
import tileworld.environment.TWEntity;


import tileworld.environment.TWHole;
import tileworld.environment.TWObject;
import tileworld.environment.TWObstacle;
import tileworld.environment.TWTile;

/**
 * @author Wong De Shun
 */
public class BenTWAgentWorkingMemory_v2 extends TWAgentWorkingMemory {

	private TWAgentPercept[][] visited;
	private Schedule mySchedule; 

	// x, y: the dimension of the grid
	public BenTWAgentWorkingMemory_v2(TWAgent moi, Schedule schedule, int x, int y) {
		super(moi, schedule, x, y);
		this.visited = new TWAgentPercept[x][y];
		this.mySchedule = schedule;
	}

	/**
	 * Called at each time step, updates the memory map of the agent.
	 * Note that some objects may disappear or be moved, in which case part of
	 * sensed may contain null objects
	 *
	 * Also note that currently the agent has no sense of moving objects, so
	 * an agent may remember the same object at two locations simultaneously.
	 * 
	 * Other agents in the grid are sensed and passed to this function. But it
	 * is currently not used for anything. Do remember that an agent sense itself
	 * too.
	 *
	 * @param sensedObjects bag containing the sensed objects
	 * @param objectXCoords bag containing x coordinates of objects
	 * @param objectYCoords bag containing y coordinates of object
	 * @param sensedAgents bag containing the sensed agents
	 * @param agentXCoords bag containing x coordinates of agents
	 * @param agentYCoords bag containing y coordinates of agents
	 */
	public void updateMemory(Bag sensedObjects, IntBag objectXCoords, IntBag objectYCoords, Bag sensedAgents, IntBag agentXCoords, IntBag agentYCoords) {
		super.updateMemory(sensedObjects, objectXCoords, objectYCoords, sensedAgents, agentXCoords, agentYCoords);
		
		for (int i = 0; i < sensedObjects.size(); i++) {
			TWEntity o = (TWEntity) sensedObjects.get(i);
			visited[o.getX()][o.getY()] = new TWAgentPercept(o, mySchedule.getTime());
		}
	}
	
	public boolean hasVisited(int x, int y) {
		TWAgentPercept ent = visited[x][y];
		if (ent == null) return false;
		return (Parameters.lifeTime * 0.8) > Math.abs(mySchedule.getTime() - (visited[x][y]).getT());
	}

	/**
	 * removes all facts earlier than now - max memory time. 
	 * TODO: Other facts are
	 * remove probabilistically (exponential decay of memory)
	 */
	public void decayMemory() {
		// put some decay on other memory pieces (this will require complete
		// iteration over memory though, so expensive.
		//This is a simple example of how to do this.
		//        for (int x = 0; x < this.objects.length; x++) {
		//       for (int y = 0; y < this.objects[x].length; y++) {
		//           TWAgentPercept currentMemory =  objects[x][y];
		//           if(currentMemory!=null && currentMemory.getT() < schedule.getTime()-MAX_TIME){
		//               memoryGrid.set(x, y, null);
		//               memorySize--;
		//           }
		//       }
		//   }
	}

}
