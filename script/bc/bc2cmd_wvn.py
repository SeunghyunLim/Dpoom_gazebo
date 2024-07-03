#!/usr/bin/env python

import rospy
import sys
import os
import math
import time
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import argparse
import threading
import torch
import torch.nn as nn
from torch.utils.data import Dataset
from torch.utils.data import DataLoader
import torch.optim as optim
import easyGo
import cv2
from cv_bridge import CvBridge, CvBridgeError
from time import sleep
import numpy as np
from collections import OrderedDict

from morp import *

# MODEL_NAME = 'wvn_morp_h4'
MODEL_NAME = 'history4'
# MODEL_NAME = 'history_10'
# MODEL_NAME = 'history4_new'
HISTORY = 4
parser = argparse.ArgumentParser()
parser.add_argument('--keyboard', action='store_true')
parser.add_argument('--control', action='store_true')
parser.add_argument('--plot', action='store_true')
args = parser.parse_args()

totalTime = 0
count = 0

MAX_SPEED = 15
MAX_STEER = 25

DRIVE_INDEX = -1  # last drive index
device = 'cpu'
# device = 'cuda'


from rosgraph_msgs.msg import Clock
sim_time =0.0
def time_callback(data):
    global sim_time
    _sec = data.clock.secs
    _nsec = data.clock.nsecs
    sim_time = _sec + _nsec * 0.000000001


sim_time = 0.0
flg = 0

def state_callback(data):
    global robot_state
    q = data.pose.pose.orientation
    yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
    robot_state = [-data.pose.pose.position.y, data.pose.pose.position.x, -yaw]

def model_input_callback(data):
    global temp
    temp = data.data
    # print(temp)

def listener():
    rospy.Subscriber("/odom", Odometry, state_callback)
    rospy.Subscriber("/clock", Clock, time_callback)
    rospy.Subscriber("/wvn_morp/model_input", Float32MultiArray, model_input_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

class SimpleNet(torch.nn.Module):

    def __init__(self, in_features, out_features, hidden_features):
        super(SimpleNet, self).__init__()
        self.lin1 = nn.Linear(in_features=in_features,
                              out_features=hidden_features, bias=True)
        self.lin2 = nn.Linear(in_features=hidden_features,
                              out_features=hidden_features * 2, bias=True)
        self.lin3 = nn.Linear(in_features=hidden_features * 2,
                              out_features=hidden_features * 3, bias=True)
        self.lin4 = nn.Linear(in_features=hidden_features * 3,
                              out_features=hidden_features, bias=True)
        self.lin5 = nn.Linear(in_features=hidden_features,
                              out_features=out_features, bias=True)
        # self.lin4 = nn.Linear(in_features=hidden_features * 3,
        #                       out_features=hidden_features * 6, bias=True)
        # self.lin5 = nn.Linear(in_features=hidden_features * 6,
        #                       out_features=hidden_features * 3, bias=True)
        # self.lin6 = nn.Linear(in_features=hidden_features * 3,
        #                       out_features=out_features, bias=True)
        self.act = nn.ReLU()

    def forward(self, x):
        prev_x = x.clone().detach()  # save previous state
        x = self.act(self.lin1(x))
        x = self.act(self.lin2(x))
        x = self.act(self.lin3(x))
        x = self.act(self.lin4(x))
        x = self.lin5(x)
        # x = self.act(self.lin5(x))
        # x = self.lin6(x)
        # return x
        return x

n_deadends = 7
n_state = 3
n_command = 2

input_size = HISTORY*(n_deadends+n_command+n_state)
model = SimpleNet(input_size, n_command, input_size)
model = model.to(device)  # kaiming init
#wandb.watch(model)

if device == 'cuda':
    model = nn.DataParallel(model)
    torch.backends.cudnn.benchmark = True
print('Am I using CPU or GPU : {}'.format(device))

print('==> Resuming from checkpoint')
assert os.path.isdir('checkpoint'), 'Error: no checkpoint dir found'
checkpoint = torch.load('./checkpoint/' + MODEL_NAME + '.pth')

# if device == 'cpu' and MODEL_NAME!='dagger__2':
#     new_checkpoint = OrderedDict()
#     for k, v in checkpoint['model'].items():
#         name = k[7:]
#         new_checkpoint[name] = v
#     model.load_state_dict(new_checkpoint)
    
# else:
#     model.load_state_dict(checkpoint['model'])

model.load_state_dict(checkpoint['model'])

best_error = checkpoint['error']
start_epoch = checkpoint['epoch']

optimizer = torch.optim.Adam(model.parameters(), lr=0.01)
criterion = nn.MSELoss()

t = time.time()

def main():
    # Configure depth and color streams
    global ROW, COL, GRN_ROI, bridge, sim_time, temp

    morp_listener = threading.Thread(target=listener)
    morp_listener.start()


    fpsFlag = False
    numFrame = 1
    fps = 0.0

    bridge = CvBridge()
    depth_scale = 1.0
    startTime = time.time()
    ground_seg_time = 0.0
    lpp_time = 0.0
    dist = 10.0

    obs_flg = 0
    #while sim_time == 0.0:
    #    continue
    t0 = sim_time

    print(dist)
    history_data = []

    model.eval()
    control_speed = 0
    control_steer = 0

    PI = 3.1415926535897

    temp = np.zeros(13)

    print('get into while')

    while(dist > 0.8):
        # print('step')
        #t1 = time.time()
        # global depth_image_raw, color_image_raw, robot_state, sim_time
        if type(temp) == type(0) :
            print('stuck')
            sleep(0.1)
            continue

        dist = math.sqrt((GOAL_X - robot_state[1])**2 + (-GOAL_Y - robot_state[0])**2)
        if obs_flg == 0 and dist < 10:
            # os.system("sh ./init.sh")
            obs_flg = 1

        if history_data == []:
            history_data = np.array(temp).reshape((1,-1))
            history_data = np.repeat(history_data, HISTORY, axis=0)
        else:
            history_data = np.roll(history_data, -1, axis=0) # data in the front is oldest
            history_data[-1] = np.array(temp)

        goal_x = history_data[:, 1]
        goal_y = history_data[:, 2]

        deadends = history_data[:, 6:] / 210
        commands = history_data[:, 3:5]
        yaw = history_data[:, 5]

        dead_ends = np.hstack(deadends)
        commands = np.hstack(commands)
        goal_x = np.hstack(goal_x)
        goal_y = np.hstack(goal_y)
        yaw = np.hstack(yaw)


        model_input = list(dead_ends)
        model_input.extend(list(commands))
        model_input.extend(list(goal_x))
        model_input.extend(list(goal_y))
        model_input.extend(list(yaw))


        with torch.no_grad():
            model_command = model(torch.FloatTensor(model_input))

        model_command = np.array(model_command)
        control_speed = model_command[0]
        control_steer = model_command[1]

        print(model_input)
        print(control_speed, control_steer)

        if numFrame < 15:
            easyGo.mvCurve(0.26/(2*PI/360), 0.0/(2*PI/360))
            # print('numF < 15')
        else:
            easyGo.mvCurve(control_speed/(2*PI/360), control_steer/(2*PI/360))
            # print('else')

        if rospy.is_shutdown():
            easyGo.mvCurve(0, 0)
            easyGo.stop()
            break
        numFrame += 1
        sleep(0.1)

    easyGo.mvCurve(0, 0)
    easyGo.stop()
    print("TOTAL TIME {}".format(float(sim_time) - t0))
    rospy.signal_shutdown("esc")

if __name__ == "__main__":
    rospy.init_node('robot_mvs', anonymous=False)
    
    main()
    exit()
