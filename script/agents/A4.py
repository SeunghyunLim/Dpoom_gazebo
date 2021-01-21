#!/usr/bin/env python

import rvo2
import matplotlib.pyplot as plt
import numpy as np
import random
import rospy
import time
import threading
import math
import sys
from utils import *
from nav_msgs.msg import Odometry

import A4easyGo as easyGo

rospy.init_node('A4_mvs', anonymous=False)

SIMUL_HZ = 10.0

sim = rvo2.PyRVOSimulator(1/SIMUL_HZ, 15.0, 10, 5.0, 2.0, 0.15, 3.0)


COL = 10.0
ROW = 10.0
voxel_size = 0.5
size = voxel_size/2

#ROBOT MOVE
SPEED = 20 # 14
ROTATE_SPEED = 25 # 25
ORCA_THRES = 2

global Target
Target = [2.5, -2.5]


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

global obs_pos, self_pos, self_yaw
obs_pos = [[0, 0], [0, 0], [0, 0], [0, 0]]
self_pos = [0, 0]
self_yaw = 0.0

def ob1_callback(data):
    global self_pos
    global obs_pos
    global self_yaw
    _x = data.pose.pose.position.x
    _y = data.pose.pose.position.y
    relative_x = _x - self_pos[1]
    relative_y = _y - self_pos[0]
    x2 = math.cos(1.57-self_yaw) * relative_x - math.sin(1.57-self_yaw) * relative_y
    y2 = math.sin(1.57-self_yaw) * relative_x + math.cos(1.57-self_yaw) * relative_y
    obs_pos[0] = [x2, y2]

def ob2_callback(data):
    global self_pos
    global obs_pos
    global self_yaw
    _x = data.pose.pose.position.x
    _y = data.pose.pose.position.y
    relative_x = _x - self_pos[1]
    relative_y = _y - self_pos[0]
    x2 = math.cos(1.57-self_yaw) * relative_x - math.sin(1.57-self_yaw) * relative_y
    y2 = math.sin(1.57-self_yaw) * relative_x + math.cos(1.57-self_yaw) * relative_y
    obs_pos[1] = [x2, y2]

def ob3_callback(data):
    global self_pos
    global obs_pos
    global self_yaw
    _x = data.pose.pose.position.x
    _y = data.pose.pose.position.y
    relative_x = _x - self_pos[1]
    relative_y = _y - self_pos[0]
    x2 = math.cos(1.57-self_yaw) * relative_x - math.sin(1.57-self_yaw) * relative_y
    y2 = math.sin(1.57-self_yaw) * relative_x + math.cos(1.57-self_yaw) * relative_y
    obs_pos[2] = [x2, y2]

def ob4_callback(data):
    global self_pos
    global obs_pos
    global self_yaw
    _x = data.pose.pose.position.x
    _y = data.pose.pose.position.y
    relative_x = _x - self_pos[1]
    relative_y = _y - self_pos[0]
    x2 = math.cos(1.57-self_yaw) * relative_x - math.sin(1.57-self_yaw) * relative_y
    y2 = math.sin(1.57-self_yaw) * relative_x + math.cos(1.57-self_yaw) * relative_y
    obs_pos[3] = [x2, y2]

def self_callback(data):
    global self_pos, self_yaw
    _x = data.pose.pose.position.x
    _y = data.pose.pose.position.y
    ox = data.pose.pose.orientation.x
    oy = data.pose.pose.orientation.y
    oz = data.pose.pose.orientation.z
    ow = data.pose.pose.orientation.w
    self_yaw = qut2eu(ox, oy, oz, ow)
    self_pos = [_y, _x]

def listener():
    print('listener ready')
    rospy.Subscriber("/tb3_3/odom", Odometry, self_callback)
    rospy.Subscriber("/tb3_0/odom", Odometry, ob1_callback)
    rospy.Subscriber("/tb3_1/odom", Odometry, ob2_callback)
    rospy.Subscriber("/tb3_2/odom", Odometry, ob3_callback)
    rospy.Subscriber("/tb3_4/odom", Odometry, ob4_callback)
    rospy.spin()


def orca(verbose=False):
    global obs_pos
    global self_pos
    global self_yaw
    global Target
    sim.processObstacles()
    agents_position =[(0,0)]
    agents = [sim.addAgent(position, 15.0, 10, 5.0, 2.0, 0.15, 3.0, (0.0,3.0)) for position in agents_position]
    agents_velocity = [(0.0, 0.5)]
    for agent, velocity in zip(agents, agents_velocity):
        sim.setAgentPrefVelocity(agent, velocity)

    pc2obs_time = 0.0
    lpp_time = 0.0
    dist = math.sqrt((Target[0] - self_pos[1])**2 + (Target[1] - self_pos[0])**2)
    step = 0
    while(dist > 0.3):
        samples = np.array(obs_pos)
        if type(samples) == type(False):
            continue
        t1 = time.time()
        t2 = time.time()
        dist = math.sqrt((Target[0] - self_pos[1])**2 + (Target[1] - self_pos[0])**2)
        _obs = [[math.sqrt(y**2 + x**2)] for x,y in samples]
        min_obs_dist = min(_obs)
        min_obs_idx = np.argmin(_obs)
        if samples[min_obs_idx][1] <= 0:
            min_obs_dist[0] = 100
        sim.clearObstacle()
        obs_position_list = [[(x-size, y-size),(x+size, y-size), (x+size, y+size), (x-size, y+size)] for x,y in samples]
        obs = [sim.addObstacle(obs_position) for obs_position in obs_position_list]
        sim.processObstacles()

        sim.setAgentPosition(0, (0,0))
        positions = [sim.getAgentPosition(agent) for agent in agents]
        sim.doStep()

        positions = np.array(positions)
        obs_position_list = np.array(obs_position_list)
        velocity = [sim.getAgentVelocity(agent) for agent in agents]
        print(min_obs_dist)
        if min_obs_dist[0] > ORCA_THRES:
            print('DAP')
            dist = math.sqrt((Target[0] - self_pos[1])**2 + (Target[1] - self_pos[0])**2)
            current_angle = self_yaw   ##rad.
            desired_angle = math.atan2(Target[1]-self_pos[0],
                                                    Target[0]-self_pos[1])
            desired_angle = math.atan2(Target[1]-self_pos[0],
                                                    Target[0]-self_pos[1])
            rot_angle = self_yaw - desired_angle
            if np.abs(rot_angle) > math.pi:
                rot_angle = -np.sign(rot_angle) * (2*math.pi - np.abs(rot_angle))
            #print(self_yaw, 'for ', desired_angle)
            if abs(rot_angle) < 0.2:
                direc = 1 # go straight
            elif rot_angle >= 0.2:
                direc = 3 # turn right
            elif rot_angle <= -0.2:
                direc = 2 # turn right
        else:
            print('ORCA')
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

        # plt.arrow(positions[0][0], positions[0][1], velocity[0][0], velocity[0][1], width=0.05)
        # for obs_position in obs_position_list:
        #     plt.plot(np.hstack([obs_position[:,0],obs_position[0][0]]), np.hstack([obs_position[:,1],obs_position[0][1]]))
        # plt.scatter(positions[:,0], positions[:,1], label='agents')
        # if len(samples) != 0:
        #     plt.scatter(samples[:,0], samples[:,1], label='samples')
        # plt.legend()
        # plt.title("Trajectories of the agnets")
        # plt.xlabel("x (m)")
        # plt.ylabel("y (m)")
        # plt.xlim(-5,5)
        # plt.ylim(-2,8)
        # plt.pause(0.001)
        # plt.cla()
        print("{:.6f} sec simulated".format(step/SIMUL_HZ))


        time.sleep(0.1)
        step += 1

    easyGo.stop()
    rospy.signal_shutdown("esc")
    sys.exit(1)

def load_orca():
    print("orca ready")
    orca_thread = threading.Thread(target=orca)
    orca_thread.start()

if __name__ == "__main__":
    try:
        load_orca()
        listener()
    except KeyboardInterrupt:
        print("Interrupted by key")
