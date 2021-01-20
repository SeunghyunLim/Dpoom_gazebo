#!/usr/bin/env python

import rvo2
import matplotlib.pyplot as plt
import numpy as np
import random
import rospy
import time

import pc2obs

import easyGo

SIMUL_HZ = 10.0

sim = rvo2.PyRVOSimulator(1/SIMUL_HZ, 15.0, 10, 5.0, 2.0, 0.15, 3.0)

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
voxel_size = 0.3
size = voxel_size/2

#ROBOT MOVE
SPEED = 10 # 14
ROTATE_SPEED = 15 # 25

def GoEasy(direc):
	if direc == 4: # Backward
		easyGo.mvStraight(- SPEED, -1)
	elif direc == 0 or direc == 1: # Go straight
		easyGo.mvStraight(SPEED, -1)
	elif direc == 2: # turn left
		easyGo.mvRotate(ROTATE_SPEED, -1, False)
	elif direc == 3: # turn right
		easyGo.mvRotate(ROTATE_SPEED, -1, True)
	elif direc == 5: # stop
		easyGo.stop()

#MIN_OBS_SIZE = 0.6 / 2
#MAX_OBS_SIZE = 1.4 / 2


# make random square object
'''
obs_center_size = [(random.uniform(-COL, COL), random.uniform(0, ROW), random.uniform(MIN_OBS_SIZE, MAX_OBS_SIZE)) for i in range(15)]
# osb_position must be convex in counter clock wise order
obs_position_list = [[(x-size, y-size),(x+size, y-size), (x+size, y+size), (x-size, y+size)] for x,y,size in obs_center_size]
obs = [sim.addObstacle(obs_position) for obs_position in obs_position_list]
'''

# single obstacle for test
# obs_position_list = [[(6.1,6.1), (4.1, 6.1), (4.1, 4.1)]]
#o1 = sim.addObstacle([(6.1,6.1), (4.1, 6.1), (4.1, 4.1)])
# obs_position_list = np.array(obs_position_list)
sim.processObstacles()

agents_position =[(0,0)]

agents = [sim.addAgent(position, 15.0, 10, 5.0, 2.0, 0.15, 3.0, (0.0,3.0)) for position in agents_position]
agents_velocity = [(0.0, 0.5)]
for agent, velocity in zip(agents, agents_velocity):
	sim.setAgentPrefVelocity(agent, velocity)

print('Simulation has %i agents and %i obstacle vertices in it.' %
	(sim.getNumAgents(), sim.getNumObstacleVertices()))

print("init pc2obs")
pc2obs.pc2obs_init()

print('Running simulation')

pc2obs_time = 0.0
lpp_time = 0.0
for step in range(1000):
	t1 = time.time()
	samples, robot_state = pc2obs.pc2obs(voxel_size = voxel_size)
	t2 = time.time()
	# print(samples)
	if type(samples) == type(False):
		print("not connected")
		continue


	sim.clearObstacle()
	obs_position_list = [[(x-size, y-size),(x+size, y-size), (x+size, y+size), (x-size, y+size)] for x,y,z in samples]
	obs = [sim.addObstacle(obs_position) for obs_position in obs_position_list]
	sim.processObstacles()

	#for agent, velocity in zip(agents, agents_velocity):
	#    sim.setAgentPrefVelocity(agent, velocity)

	# always set agent position as origin
	sim.setAgentPosition(0, (0,0))
	positions = [sim.getAgentPosition(agent) for agent in agents]

	sim.doStep()
	#positions = [sim.getAgentPosition(agent) for agent in agents]

	#print('step=%2i  t=%.3f  %s' % (step, sim.getGlobalTime(), '  '.join(positions)))
	positions = np.array(positions)
	obs_position_list = np.array(obs_position_list)
	velocity = [sim.getAgentVelocity(agent) for agent in agents]
	print("Agent velocity: {}, {}".format(velocity[0][0], velocity[0][1]))
	if velocity[0][0] < 0:
		direc = 2 # turn left
	elif velocity[0][0] > 0:
		direc = 3 # turn right
	elif velocity[0][1] > 0:
		direc = 1 # go straight
	elif velocity[0][1] < 0:
		direc = 4 # backward
	else:
		direc = 5 # stop
	GoEasy(direc)
	t3 = time.time()

	pc2obs_time += t2-t1
	lpp_time += t3-t2

	print("pc2obs took: {} sec".format(t2-t1))
	print("OCRA took: {} sec".format(t3-t2))
	print("Average took: {} sec, {} sec".format(pc2obs_time/step, lpp_time/step))

	plt.arrow(positions[0][0], positions[0][1], velocity[0][0], velocity[0][1], width=0.05)
	for obs_position in obs_position_list:
		plt.plot(np.hstack([obs_position[:,0],obs_position[0][0]]), np.hstack([obs_position[:,1],obs_position[0][1]]))
	plt.scatter(positions[:,0], positions[:,1], label='agents')
	if len(samples) != 0:
		plt.scatter(samples[:,0], samples[:,1], label='samples')
	plt.legend()
	plt.title("Trajectories of the agnets")
	plt.xlabel("x (m)")
	plt.ylabel("y (m)")
	plt.xlim(-5,5)
	plt.ylim(-2,8)
	plt.pause(0.001)
	plt.cla()
	print("{:.6f} sec simulated".format(step/SIMUL_HZ))

easyGo.stop()
rospy.signal_shutdown("esc")
sys.exit(1)
