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

import sys
sys.path.append("..")
import A1scan2obs as pc2obs

rospy.init_node('A1_mvs', anonymous=False)

global obs_pos, self_pos, self_yaw
obs_pos = [[0, 0]]
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

def plot():
    global obs_pos
    while True:
        plt.scatter(obs_pos[0][1], obs_pos[0][0])
        print('x= ', obs_pos[0][1],' y= ', obs_pos[0][0])
        plt.xlim(-5, 5)
        plt.ylim(-5, 5)
        plt.pause(0.001)


def load_plot():
	print("plot ready")
	plt_thread = threading.Thread(target=plot)
	plt_thread.start()

if __name__ == "__main__":
	try:
		load_plot()
		listener()
	except KeyboardInterrupt:
		print("Interrupted by key")
