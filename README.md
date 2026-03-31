# AI6125-Tileworld Agent

Building intelligent agents for the Tileworld environment.

This repository contains the Java source code for a Tileworld simulation framework, including:
- Environment and entities (`environment/`)
- Agent base classes and utilities (`agent/`)
- Path planning utilities (`planners/`)
- Runtime parameters and entry points (`TileworldMain`, `Parameters`, GUI)

## Goal

The goal is to implement an agent that maximizes total rewards in the Tileworld setting.

## Agent Implementation Modules

Three modules to be implemented:

1. Planning module
	- After sensing/communication, the agent's memory is updated.
	- The agent then plans its next action accordingly.

2. Memory module
	- Agents store environmental information in memory.
	- The default module is `TWAgentWorkingMemory`.

3. Communication module
	- At every step, each agent can broadcast a message encoded by `Message`.

## Assessment Method

Evaluation is based on three experiment configurations.

- Number of agents: 6
- Steps per experiment: 5000
- Initial fuel per agent: 500
- Repetitions: 10 runs for each configuration
- Metric: average reward over the 10 runs

### Configuration 1

- Environment size: `50 x 50`
- Average object creation rate: `NormalDistribution(mu=0.2, sigma=0.05)`
- Object lifetime: `100`

### Configuration 2

- Environment size: `80 x 80`
- Average object creation rate: `NormalDistribution(mu=2, sigma=0.5)`
- Object lifetime: `30`

### Configuration 3

- Hidden configuration (used in assessment)

## Assessment Runner

For a headless evaluation of Configuration 1 and Configuration 2, run `tileworld.AssessmentRunner` as a Java application.

- File: `Tileworld/src/tileworld/AssessmentRunner.java`
- Agent initialization mix (6 total): `3 x ArdaTWAgent_v1`, `3 x AmeyaGreedyBFSAgentWithMemory`
- It executes `10` runs per configuration and `5000` steps per run.
- It prints environment settings, run-level reward summaries, and aggregate statistics.
- It reports per-agent metrics including reward, final fuel, refuel count, max carried tiles, fuel consumption, idle ratio, and ran-out-of-fuel counts.