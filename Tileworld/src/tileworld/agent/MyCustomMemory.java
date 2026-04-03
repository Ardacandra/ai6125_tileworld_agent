package tileworld.agent;

import sim.engine.Schedule;
import tileworld.environment.TWObject;
import tileworld.environment.TWTile;
import tileworld.environment.TWHole;
import sim.util.Int2D;
import java.util.HashMap;
import java.util.Map;

public class MyCustomMemory extends TWAgentWorkingMemory {
    
    
    private final Map<Int2D, Double> lastSeenTime;
    private final int memoryLifespan;
    private final Schedule schedule;

    public MyCustomMemory(TWAgent moi, Schedule schedule, int x, int y, int lifespan) {
        super(moi, schedule, x, y);
        this.lastSeenTime = new HashMap<>();
        this.memoryLifespan = lifespan;
        this.schedule = schedule;
    }

    
    @Override
    public void updateMemory(sim.util.Bag sensedObjects, 
                            sim.util.IntBag objectXCoords, 
                            sim.util.IntBag objectYCoords, 
                            sim.util.Bag sensedAgents, 
                            sim.util.IntBag agentXCoords, 
                            sim.util.IntBag agentYCoords) {
        
        
        super.updateMemory(sensedObjects, objectXCoords, objectYCoords, 
                           sensedAgents, agentXCoords, agentYCoords);

        
        double now = schedule.getTime();
        for (int i = 0; i < sensedObjects.size(); i++) {
            Object obj = sensedObjects.get(i);
            if (obj instanceof TWTile || obj instanceof TWHole) {
                TWObject o = (TWObject) obj;
                lastSeenTime.put(new Int2D(o.getX(), o.getY()), now);
            }
        }
    }

    
    public void clearLocation(int x, int y) {
        this.removeAgentPercept(x, y); // Clears parent's objects[][]
        this.getMemoryGrid().set(x, y, null); // Clears visual grid
        this.lastSeenTime.remove(new Int2D(x, y)); // Clears our timestamp map
    }

    
    public double getFreshness(int x, int y, double currentStep) {
        Double seenAt = lastSeenTime.get(new Int2D(x, y));
        
        if (seenAt == null) {
            return 0.0;
        }
        
        double age = currentStep - seenAt;
        if (age >= memoryLifespan) {
            return 0.0;
        }
        
        // Linear decay: 1.0 (new) -> 0.0 (old)
        return 1.0 - (age / (double) memoryLifespan);
    }
}
