#!/usr/bin/env python

import rvo2
import matplotlib.pyplot as plt
import numpy as np
import random

import pc2obs

sim = rvo2.PyRVOSimulator(1/60., 15.0, 10, 5.0, 2.0, 0.2, 3.0)

# parameter description (implementations in RVOSimulator.cpp)
# timeStep: 
# neighborDist : max dist (center to center) to other agents
# maxNeighbors: max num of other agengts the agent takes into account in navigation
# timeHorizon: min amount of time to compute to be safe with respect to agents
# timeHorizonObst: min amount of time to compute to be safe with respect to obstacles
# radius: radius of agent
# maxSpeed: max speed of te agent
# velocity (Vector2 &) : current velocity of the agent (setAgentPrefVelocity())

COL = 10.0
ROW = 10.0
MIN_OBS_SIZE = 0.6 / 2
MAX_OBS_SIZE = 1.4 / 2
obs_center_size = [(random.uniform(-COL, COL), random.uniform(0, ROW), random.uniform(MIN_OBS_SIZE, MAX_OBS_SIZE)) for i in range(15)]
# osb_position must be convex in counter clock wise order
obs_position_list = [[(x-size, y-size),(x+size, y-size), (x+size, y+size), (x-size, y+size)] for x,y,size in obs_center_size]
obs = [sim.addObstacle(obs_position) for obs_position in obs_position_list]

# single obstacle for test
# obs_position_list = [[(6.1,6.1), (4.1, 6.1), (4.1, 4.1)]]
#o1 = sim.addObstacle([(6.1,6.1), (4.1, 6.1), (4.1, 4.1)])
obs_position_list = np.array(obs_position_list)
sim.processObstacles()

agents_position =[(-1,-1)]

agents = [sim.addAgent(position, 15.0, 10, 5.0, 2.0, 0.2, 3.0, (3,3)) for position in agents_position]
agents_velocity = [(3.0, 3.0)]
for agent, velocity in zip(agents, agents_velocity):
    sim.setAgentPrefVelocity(agent, velocity)

print('Simulation has %i agents and %i obstacle vertices in it.' %
	(sim.getNumAgents(), sim.getNumObstacleVertices()))

print("init pc2obs")
pc2obs.pc2obs_init()

print('Running simulation')

for step in range(1000):
    samples = pc2obs.pc2obs(voxel_size = 0.3)

    if step == 200:
        agents_velocity = [(0, 0)]
        print("stop")
    elif step == 280:
        agents_velocity = [(-3, 2)]
        print("go")
    for agent, velocity in zip(agents, agents_velocity):
        sim.setAgentPrefVelocity(agent, velocity)

    sim.doStep()
    positions = [sim.getAgentPosition(agent) for agent in agents]

    #print('step=%2i  t=%.3f  %s' % (step, sim.getGlobalTime(), '  '.join(positions)))
    positions = np.array(positions)
    for obs_position in obs_position_list:
        plt.plot(np.hstack([obs_position[:,0],obs_position[0][0]]), np.hstack([obs_position[:,1],obs_position[0][1]]))
    plt.scatter(positions[:,0], positions[:,1], label='agents')
    plt.scatter(samples[:,0], samples[:,1], label='samples')
    plt.legend()
    plt.title("Trajectories of the agnets")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.xlim(-5,10)
    plt.ylim(-5,10)
    plt.pause(0.001)
    plt.cla()




