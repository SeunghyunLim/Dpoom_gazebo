#!/usr/bin/env python

import rvo2
import matplotlib.pyplot as plt
import numpy as np
import random
import rospy
import time
import math
import easyGo
import pc2obs
import os

SIMUL_HZ = 10.0

sim = rvo2.PyRVOSimulator(1/SIMUL_HZ, 15.0, 15, 2.0, 2.0, 0.15, 3.0)

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
SPEED = 15 # 14
ROTATE_SPEED = 25 # 25
ANGULAR_SPEED = 0.2

# Set goal position
GOAL_X = 0
GOAL_Y = 3
robot_state = [0, -8, 0]

def GoEasy(direc, speed_ratio):
    if direc == 4: # Backward
        easyGo.mvStraight(- SPEED*speed_ratio, -1)
    elif direc == 0 or direc == 1: # Go straight
        easyGo.mvStraight(SPEED*speed_ratio, -1)
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

agents = [sim.addAgent(position, 15.0, 10, 2.0, 2.0, 0.15, 0.25, (0.0,0.0)) for position in agents_position]
agents_velocity = [(0.0, 0.25)]
for agent, velocity in zip(agents, agents_velocity):
    sim.setAgentPrefVelocity(agent, velocity)

print('Simulation has %i agents and %i obstacle vertices in it.' %
    (sim.getNumAgents(), sim.getNumObstacleVertices()))

print("init pc2obs")
pc2obs.pc2obs_init()

print('Running simulation')

pc2obs_time = 0.0
lpp_time = 0.0
dist = 20.0
step = 1

obs_flg = 0

sim_time = False
while type(sim_time) == type(False):
    samples, robot_state, sim_time = pc2obs.pc2obs(voxel_size = voxel_size)
t0 = float(sim_time)
while(dist > 0.8):
    t1 = time.time()
    samples, robot_state, sim_time = pc2obs.pc2obs(voxel_size = voxel_size)
    t2 = time.time()
    # print(samples)
    if type(samples) == type(False):
        print("not connected")
        continue
    dist = math.sqrt((GOAL_X - robot_state[1])**2 + (-GOAL_Y - robot_state[0])**2)
    if obs_flg == 0 and dist < 10:
        os.system("sh ./init.sh")
        obs_flg = 1

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
    speed_ratio = math.sqrt(velocity[0][0]**2+velocity[0][1]**2) / 0.25 # 0 ~ 1

    if velocity[0][0] < -0.02:
        direc = 2 # turn left
    elif velocity[0][0] > 0.02:
        direc = 3 # turn right
    elif velocity[0][1] > 0.02:
        direc = 1 # go straight
    elif velocity[0][1] < -0.02:
        direc = 4 # backward
    else:
        direc = 5 # stop
    if direc == 1:
        diff_angle = (-robot_state[2] + math.atan2(GOAL_X - robot_state[1], -GOAL_Y - robot_state[0]))
        if diff_angle > 0:
            v_ang = ANGULAR_SPEED * min(diff_angle/(math.pi/2), 1)
        else:
            v_ang = ANGULAR_SPEED * max(diff_angle/(math.pi/2), -1)
        easyGo.mvCurve(SPEED*speed_ratio, -v_ang)
    else:
        GoEasy(direc, speed_ratio)
    t3 = time.time()

    pc2obs_time += t2-t1
    lpp_time += t3-t2

    print("pc2obs took: {} sec".format(t2-t1))
    print("OCRA took: {} sec".format(t3-t2))
    print("Average took: {} sec, {} sec".format(pc2obs_time/step, lpp_time/step))
    print("Distance to the Goal: {}".format(dist))

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
    #print("{:.6f} sec simulated".format(step/SIMUL_HZ))
    step = step+1
    print("iter time: {}".format(time.time() - t1))
    print("NAV TIME {}".format(float(sim_time) - t0))
    print()
print("TOTAL TIME {}".format(float(sim_time) - t0))
easyGo.stop()
rospy.signal_shutdown("esc")
