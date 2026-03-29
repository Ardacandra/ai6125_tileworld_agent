package tileworld;

import java.io.OutputStream;
import java.io.PrintStream;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ThreadLocalRandom;
import sim.util.Bag;
import tileworld.agent.TWAgent;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWObject;

/**
 * Headless assessment runner for Config 1 and Config 2 in the README.
 *
 * This class is meant to be run directly as a Java application.
 *
 * Note:
 * - It does not require any source-code changes to TWEnvironment or TWObjectCreator.
 * - Config 2 is applied through runtime reflection overrides inside this runner.
 */
public class AssessmentRunner {

    private static final int STEPS = 5000;
    private static final int RUNS_PER_CONFIG = 10;
    private static final boolean QUIET_AGENT_LOGS = true;
    private static final double EPS = 1e-9;

    private static final class Config {
        final String name;
        final int x;
        final int y;
        final double mu;
        final double sigma;
        final int lifeTime;
        final int fuel;
        final int agents;

        Config(String name, int x, int y, double mu, double sigma, int lifeTime, int fuel, int agents) {
            this.name = name;
            this.x = x;
            this.y = y;
            this.mu = mu;
            this.sigma = sigma;
            this.lifeTime = lifeTime;
            this.fuel = fuel;
            this.agents = agents;
        }
    }

    private static final class AgentAggregate {
        String className = "";
        int runsObserved;
        int totalScore;
        double totalFinalFuel;
        int ranOutCount;
        int totalRefuelCount;
        int totalMaxTilesCarried;
        int peakTilesCarried;
        double totalFuelConsumed;
        long totalIdleSteps;

        void addRun(int score, double finalFuel, boolean ranOut, int refuelCount,
                int maxTilesCarried, double fuelConsumed, long idleSteps) {
            runsObserved++;
            totalScore += score;
            totalFinalFuel += finalFuel;
            totalRefuelCount += refuelCount;
            totalMaxTilesCarried += maxTilesCarried;
            totalFuelConsumed += fuelConsumed;
            totalIdleSteps += idleSteps;
            peakTilesCarried = Math.max(peakTilesCarried, maxTilesCarried);
            if (ranOut) {
                ranOutCount++;
            }
        }
    }

    private static final class RunSummary {
        int runReward;
        long stepsExecuted;
        Map<String, String> classNameByAgent = new LinkedHashMap<String, String>();
        Map<String, Integer> scoreByAgent = new LinkedHashMap<String, Integer>();
        Map<String, Double> finalFuelByAgent = new LinkedHashMap<String, Double>();
        Map<String, Integer> refuelCountByAgent = new LinkedHashMap<String, Integer>();
        Map<String, Integer> maxTilesByAgent = new LinkedHashMap<String, Integer>();
        Map<String, Double> fuelConsumedByAgent = new LinkedHashMap<String, Double>();
        Map<String, Long> idleStepsByAgent = new LinkedHashMap<String, Long>();
        Set<String> ranOutAgents = new HashSet<String>();
    }

    public static void main(String[] args) {
        Config config1 = new Config("Configuration 1", 50, 50, 0.2, 0.05, 100, 500, 6);
        Config config2 = new Config("Configuration 2", 80, 80, 2.0, 0.5, 30, 500, 6);

        runConfiguration(config1);
        runConfiguration(config2);
    }

    private static void runConfiguration(Config config) {
        System.out.println("==================================================");
        System.out.println(config.name);
        printConfig(config);

        int totalReward = 0;
        int minReward = Integer.MAX_VALUE;
        int maxReward = Integer.MIN_VALUE;
        List<Integer> rewards = new ArrayList<Integer>();
        Map<String, AgentAggregate> perAgent = new LinkedHashMap<String, AgentAggregate>();
        int totalRefuelsAcrossRuns = 0;
        double totalFuelConsumedAcrossRuns = 0;
        long totalIdleStepsAcrossRuns = 0;

        for (int run = 1; run <= RUNS_PER_CONFIG; run++) {
            int seed = ThreadLocalRandom.current().nextInt(1, Integer.MAX_VALUE);
            RunSummary summary = runSingleAssessment(seed, config);

            rewards.add(summary.runReward);
            totalReward += summary.runReward;
            minReward = Math.min(minReward, summary.runReward);
            maxReward = Math.max(maxReward, summary.runReward);

            for (Map.Entry<String, Integer> scoreEntry : summary.scoreByAgent.entrySet()) {
                String agentName = scoreEntry.getKey();
                int score = scoreEntry.getValue();
                double finalFuel = summary.finalFuelByAgent.get(agentName);
                boolean ranOut = summary.ranOutAgents.contains(agentName);
                int refuels = summary.refuelCountByAgent.get(agentName);
                int maxTiles = summary.maxTilesByAgent.get(agentName);
                double fuelConsumed = summary.fuelConsumedByAgent.get(agentName);
                long idleSteps = summary.idleStepsByAgent.get(agentName);

                AgentAggregate aggregate = perAgent.get(agentName);
                if (aggregate == null) {
                    aggregate = new AgentAggregate();
                    aggregate.className = summary.classNameByAgent.getOrDefault(agentName, "");
                    perAgent.put(agentName, aggregate);
                }
                aggregate.addRun(score, finalFuel, ranOut, refuels, maxTiles, fuelConsumed, idleSteps);
                totalRefuelsAcrossRuns += refuels;
                totalFuelConsumedAcrossRuns += fuelConsumed;
                totalIdleStepsAcrossRuns += idleSteps;
            }

            System.out.println("Run " + run + " seed=" + seed
                    + " reward=" + summary.runReward
                    + " ranOut=" + summary.ranOutAgents
                    + " steps=" + summary.stepsExecuted);
        }

        double meanReward = totalReward / (double) RUNS_PER_CONFIG;
        double stdReward = computeStdDev(rewards, meanReward);

        System.out.println("--- Aggregate Result ---");
        System.out.println("Runs: " + RUNS_PER_CONFIG + ", Steps each: " + STEPS);
        System.out.println("Reward avg=" + String.format("%.2f", meanReward)
                + " min=" + minReward
                + " max=" + maxReward
                + " std=" + String.format("%.2f", stdReward));
        System.out.println("Reward per agent avg="
            + String.format("%.2f", meanReward / config.agents)
            + " | reward per 1000 steps="
            + String.format("%.2f", meanReward * 1000.0 / STEPS));
        System.out.println("Team avg refuels/run="
            + String.format("%.2f", totalRefuelsAcrossRuns / (double) RUNS_PER_CONFIG)
            + " | team avg fuel consumed/run="
            + String.format("%.2f", totalFuelConsumedAcrossRuns / RUNS_PER_CONFIG)
            + " | team idle ratio="
            + String.format("%.2f", (100.0 * totalIdleStepsAcrossRuns)
                / (RUNS_PER_CONFIG * STEPS * config.agents))
            + "%");

        System.out.println("--- Per-Agent Averages ---");
        for (Map.Entry<String, AgentAggregate> entry : perAgent.entrySet()) {
            String agent = entry.getKey();
            AgentAggregate agg = entry.getValue();
            double avgScore = agg.totalScore / (double) agg.runsObserved;
            double avgFinalFuel = agg.totalFinalFuel / (double) agg.runsObserved;
            double avgRefuels = agg.totalRefuelCount / (double) agg.runsObserved;
            double avgMaxTiles = agg.totalMaxTilesCarried / (double) agg.runsObserved;
            double avgFuelConsumed = agg.totalFuelConsumed / (double) agg.runsObserved;
            double idleRatio = 100.0 * agg.totalIdleSteps / (agg.runsObserved * (double) STEPS);
            System.out.println(agent + " (" + agg.className + ")"
                    + " | avgReward=" + String.format("%.2f", avgScore)
                    + " | avgFinalFuel=" + String.format("%.2f", avgFinalFuel)
                + " | avgRefuels=" + String.format("%.2f", avgRefuels)
                + " | avgMaxTilesCarried=" + String.format("%.2f", avgMaxTiles)
                + " | peakTilesCarried=" + agg.peakTilesCarried
                + " | avgFuelConsumed=" + String.format("%.2f", avgFuelConsumed)
                + " | idleRatio=" + String.format("%.2f", idleRatio) + "%"
                + " | ranOutInRuns=" + agg.ranOutCount + "/" + RUNS_PER_CONFIG);
        }
    }

    private static RunSummary runSingleAssessment(int seed, Config config) {
        TWEnvironment env = new TWEnvironment(seed);

        applyEnvironmentOverrides(env, config);

        PrintStream originalOut = System.out;
        if (QUIET_AGENT_LOGS) {
            System.setOut(new PrintStream(new OutputStream() {
                @Override
                public void write(int b) {
                    // Ignore verbose per-step logs from agents.
                }
            }));
        }

        try {
            env.start();

            List<TWAgent> agents = collectAgents(env);
            Set<String> ranOut = new HashSet<String>();
            Map<String, Double> previousFuel = new LinkedHashMap<String, Double>();
            Map<String, Integer> refuelCounts = new LinkedHashMap<String, Integer>();
            Map<String, Integer> maxTilesCarried = new LinkedHashMap<String, Integer>();
            Map<String, Double> fuelConsumed = new LinkedHashMap<String, Double>();
            Map<String, Long> idleSteps = new LinkedHashMap<String, Long>();

            for (TWAgent agent : agents) {
                String name = agent.getName();
                previousFuel.put(name, agent.getFuelLevel());
                refuelCounts.put(name, 0);
                int carriedNow = getCarriedTileCount(agent);
                maxTilesCarried.put(name, carriedNow);
                fuelConsumed.put(name, 0.0);
                idleSteps.put(name, 0L);
            }

            long steps = 0;
            while (steps < STEPS) {
                if (!env.schedule.step(env)) {
                    break;
                }
                steps = env.schedule.getSteps();

                enforceConfiguredLifetime(env, config.lifeTime);

                for (TWAgent agent : agents) {
                    String name = agent.getName();
                    double prevFuel = previousFuel.get(name);
                    double currentFuel = agent.getFuelLevel();
                    double delta = currentFuel - prevFuel;

                    if (delta > EPS) {
                        refuelCounts.put(name, refuelCounts.get(name) + 1);
                    } else if (delta < -EPS) {
                        fuelConsumed.put(name, fuelConsumed.get(name) - delta);
                    } else {
                        idleSteps.put(name, idleSteps.get(name) + 1);
                    }

                    previousFuel.put(name, currentFuel);

                    int carried = getCarriedTileCount(agent);
                    if (carried > maxTilesCarried.get(name)) {
                        maxTilesCarried.put(name, carried);
                    }

                    if (currentFuel <= 0) {
                        ranOut.add(agent.getName());
                    }
                }
            }

            RunSummary summary = new RunSummary();
            summary.runReward = env.getReward();
            summary.stepsExecuted = steps;
            summary.ranOutAgents.addAll(ranOut);

            for (TWAgent agent : agents) {
                String name = agent.getName();
                summary.classNameByAgent.put(name, agent.getClass().getSimpleName());
                summary.scoreByAgent.put(name, agent.getScore());
                summary.finalFuelByAgent.put(name, agent.getFuelLevel());
                summary.refuelCountByAgent.put(name, refuelCounts.get(name));
                summary.maxTilesByAgent.put(name, maxTilesCarried.get(name));
                summary.fuelConsumedByAgent.put(name, fuelConsumed.get(name));
                summary.idleStepsByAgent.put(name, idleSteps.get(name));
            }

            env.finish();
            return summary;
        } finally {
            if (QUIET_AGENT_LOGS) {
                System.setOut(originalOut);
            }
        }
    }

    private static void applyEnvironmentOverrides(TWEnvironment env, Config config) {
        setIntField(env, "xDimension", config.x);
        setIntField(env, "yDimension", config.y);

        Object tileCreator = getField(env, "tileCreator");
        Object holeCreator = getField(env, "holeCreator");
        Object obstacleCreator = getField(env, "obstacleCreator");

        setDoubleField(tileCreator, "mean", config.mu);
        setDoubleField(holeCreator, "mean", config.mu);
        setDoubleField(obstacleCreator, "mean", config.mu);

        setDoubleField(tileCreator, "dev", config.sigma);
        setDoubleField(holeCreator, "dev", config.sigma);
        setDoubleField(obstacleCreator, "dev", config.sigma);
    }

    private static void enforceConfiguredLifetime(TWEnvironment env, int configuredLifeTime) {
        double now = env.schedule.getTime();
        capRemainingLife(getBag(env, "tiles"), now, configuredLifeTime);
        capRemainingLife(getBag(env, "holes"), now, configuredLifeTime);
        capRemainingLife(getBag(env, "obstacles"), now, configuredLifeTime);
    }

    private static void capRemainingLife(Bag bag, double now, int maxRemainingLife) {
        for (int i = 0; i < bag.size(); i++) {
            Object o = bag.get(i);
            if (!(o instanceof TWObject)) {
                continue;
            }
            TWObject twObject = (TWObject) o;
            if (twObject.getTimeLeft(now) > maxRemainingLife) {
                twObject.setDeathTime(now + maxRemainingLife);
            }
        }
    }

    private static List<TWAgent> collectAgents(TWEnvironment env) {
        List<TWAgent> agents = new ArrayList<TWAgent>();
        for (int x = 0; x < env.getxDimension(); x++) {
            for (int y = 0; y < env.getyDimension(); y++) {
                Object o = env.getAgentGrid().get(x, y);
                if (o instanceof TWAgent) {
                    agents.add((TWAgent) o);
                }
            }
        }
        return agents;
    }

    private static double computeStdDev(List<Integer> values, double mean) {
        double sum = 0;
        for (int value : values) {
            double delta = value - mean;
            sum += delta * delta;
        }
        return Math.sqrt(sum / values.size());
    }

    private static void printConfig(Config config) {
        System.out.println("Env: " + config.x + "x" + config.y
                + ", Agents: " + config.agents
                + ", Initial fuel: " + config.fuel
                + ", Steps: " + STEPS
                + ", Runs: " + RUNS_PER_CONFIG);
        System.out.println("Object creation rate: N(mu=" + config.mu + ", sigma=" + config.sigma + ")");
        System.out.println("Object lifetime: " + config.lifeTime);
        System.out.println("Extra metrics: refuel count, max carried tiles, fuel consumed, idle ratio");
    }

    @SuppressWarnings("unchecked")
    private static int getCarriedTileCount(TWAgent agent) {
        try {
            Field field = TWAgent.class.getDeclaredField("carriedTiles");
            field.setAccessible(true);
            List<Object> carried = (List<Object>) field.get(agent);
            return carried.size();
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("Failed to read carriedTiles from TWAgent", e);
        }
    }

    private static Object getField(Object target, String fieldName) {
        try {
            Field field = target.getClass().getDeclaredField(fieldName);
            field.setAccessible(true);
            return field.get(target);
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("Failed to read field '" + fieldName + "'", e);
        }
    }

    private static Bag getBag(Object target, String fieldName) {
        Object value = getField(target, fieldName);
        if (!(value instanceof Bag)) {
            throw new IllegalStateException("Field '" + fieldName + "' is not a Bag");
        }
        return (Bag) value;
    }

    private static void setIntField(Object target, String fieldName, int value) {
        try {
            Field field = target.getClass().getDeclaredField(fieldName);
            field.setAccessible(true);
            field.setInt(target, value);
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("Failed to set int field '" + fieldName + "'", e);
        }
    }

    private static void setDoubleField(Object target, String fieldName, double value) {
        try {
            Field field = target.getClass().getDeclaredField(fieldName);
            field.setAccessible(true);
            field.setDouble(target, value);
        } catch (ReflectiveOperationException e) {
            throw new IllegalStateException("Failed to set double field '" + fieldName + "'", e);
        }
    }
}
