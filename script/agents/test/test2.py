import rvo2
import matplotlib.pyplot as plt
import numpy as np
import random
import rospy
import time
import threading
import math
from utils import *
from nav_msgs.msg import Odometry

import A1easyGo as easyGo

import sys
sys.path.append("..")

rospy.init_node('A1_mvs', anonymous=False)

PI = math.pi


#ROBOT MOVE
SPEED = 20 # 14
ROTATE_SPEED = 15 # 25

global Target
Target = [1.0, -1.7]

global obs_pos, self_pos, self_yaw
obs_pos = [[0, 0]]
self_pos = [0, 0]
self_yaw = 0.0

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
    #obs_pos = [[relative_y, relative_x]]
    obs_pos = [[y2, x2]]
    #print(obs_pos)

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
	rospy.Subscriber("/tb3_0/odom", Odometry, self_callback)
	rospy.Subscriber("/tb3_1/odom", Odometry, ob1_callback)
	rospy.spin()


def load_dap():
	print("dap ready")
	dap_thread = threading.Thread(target=dap)
	dap_thread.start()

def dap():     # path is 2D array (x, y)
    global self_pos
    global self_yaw
    global Target
    dist = math.sqrt((Target[0] - self_pos[1])**2 + (Target[1] - self_pos[0])**2)
    while(dist > 0.3):
        dist = math.sqrt((Target[0] - self_pos[1])**2 + (Target[1] - self_pos[0])**2)
        current_angle = self_yaw   ##rad.
        #print('current_angle :', current_angle)
        if Target[1]-self_pos[0]>=0 :   # pi/2 <= desired_angle <= pi/2
            desired_angle = math.atan2(Target[1]-self_pos[0],
                                                Target[0]-self_pos[1])
        else :   #abs(desired_angle) >= pi/2
            desired_angle = math.atan2(Target[1]-self_pos[0],
                                                Target[0]-self_pos[1])
        #print(self_yaw, 'for ', desired_angle)
        if abs(self_yaw - desired_angle) < 0.1:
            direc = 1 # go straight
        elif self_yaw - desired_angle >= 0.1:
            direc = 3 # turn right
        elif self_yaw - desired_angle <= -0.1:
            direc = 2 # turn right
        GoEasy(direc)
        time.sleep(0.1)
    easyGo.stop()

if __name__ == "__main__":
	try:
		load_dap()
		listener()
	except KeyboardInterrupt:
		print("Interrupted by key")
