#!/usr/bin/env python

import numpy as np
import cv2
import matplotlib.pyplot as plt
import math
import time
import sys

from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy

from cv_bridge import CvBridge
import threading
from time import sleep
from rosgraph_msgs.msg import Clock
import csv

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--keyboard', action='store_true')
parser.add_argument('--control', action='store_true')
parser.add_argument('--plot', action='store_true')
args = parser.parse_args()

from morp import *
from wvn_utils import *
from rosgraph_msgs.msg import Clock

from wild_visual_navigation import WVN_ROOT_DIR
from wild_visual_navigation.feature_extractor import FeatureExtractor
from wild_visual_navigation.cfg import ExperimentParams
from wild_visual_navigation.image_projector import ImageProjector
from wild_visual_navigation.model import get_model
from wild_visual_navigation.utils import ConfidenceGenerator
from PIL import Image as Im
import torch

import torch.nn.functional as F
from omegaconf import OmegaConf
from wild_visual_navigation.utils import Data
from os.path import join
import os
from matplotlib import cm
import seaborn as sns

import torchvision.transforms as transforms
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

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

from rosgraph_msgs.msg import Clock

# MODEL_NAME = 'dagger__2'
# MODEL_NAME = 'history4_new'
MODEL_NAME = 'wvn_morp_h4'
HISTORY = 4

totalTime = 0
count = 0

MAX_SPEED = 15
MAX_STEER = 25

DRIVE_INDEX = -1  # last drive index

################################################################
params = OmegaConf.structured(ExperimentParams)
anomaly_detection = False
# device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
device = "cpu"

if anomaly_detection:
    confidence_generator = ConfidenceGenerator(
        method=params.loss_anomaly.method, std_factor=params.loss_anomaly.confidence_std_factor
    )
else:
    confidence_generator = ConfidenceGenerator(
        method=params.loss.method, std_factor=params.loss.confidence_std_factor
    )

# model_name = "indoor_mpi"
# model_name = "mpi_outdoor_in_hallway"
# model_name = "mpi_outdoor_paved_road"
# model_name = "mpi_outdoor_after_stepping_on_grass"
# model_name = "jackal_outdoor"
model_name = "dpoom"

  # choices = custom
network_input_image_height = 224
network_input_image_width = 224
segmentation_type = "stego"
  # choices=["slic", "grid", "random", "stego"]
feature_type = "stego"
  # choices=["dino", "dinov2", "stego"]
dino_patch_size = 8
  # choices=[8, 16]
dino_backbone = "vit_small"
slic_num_components = 100
prediction_per_pixel=True

feature_extractor = FeatureExtractor(
    device=device,
    segmentation_type=segmentation_type,
    feature_type=feature_type,
    patch_size=dino_patch_size,
    backbone_type=dino_backbone,
    input_size=network_input_image_height,
    slic_num_components=slic_num_components,
)

params.model.simple_mlp_cfg.input_size = feature_extractor.feature_dim
params.model.double_mlp_cfg.input_size = feature_extractor.feature_dim
params.model.simple_gcn_cfg.input_size = feature_extractor.feature_dim
params.model.linear_rnvp_cfg.input_size = feature_extractor.feature_dim

# Load traversability model
model_wvn = get_model(params.model).to(device)
model_wvn.eval()
torch.set_grad_enabled(False)

p = join(WVN_ROOT_DIR, "assets", "checkpoints", f"{model_name}.pt")
model_state_dict = torch.load(p)
model_wvn.load_state_dict(model_state_dict, strict=False)
print(f"\nLoaded model `{model_name}` successfully!")

# Confidence generator
cg = model_state_dict["confidence_generator"]
# Only mean and std are needed
confidence_generator.var = cg["var"]
confidence_generator.mean = cg["mean"]
confidence_generator.std = cg["std"]

global H, W, virtual_lane_available

H, W = network_input_image_height, network_input_image_width

t = time.time()
sim_time = 0.0
virtual_lane_available = 0

def time_callback(data):
    global sim_time
    _sec = data.clock.secs
    _nsec = data.clock.nsecs
    sim_time = _sec + _nsec * 0.000000001

sim_time = 0.0
flg = 0
t0 = 0

def state_callback(data):
    global robot_state
    q = data.pose.pose.orientation
    yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
    robot_state = [-data.pose.pose.position.y, data.pose.pose.position.x, -yaw]

def cmd_callback(data):
    global cmd_vel
    cmd_vel = data

def img_callback(data):
    global H, W, virtual_lane_available, t0
    t_loop = time.time()
    # data : Image
    torch_image = ros_image_to_torch(data, device=device)
    C, H_in, W_in = torch_image.shape


    # K can be ignored given that no reprojection is performed
    image_projector = ImageProjector(
        K=torch.eye(4, device=device)[None],
        h=H_in,
        w=W_in,
        new_h=H,
        new_w=W,
    )
    torch_image = image_projector.resize_image(torch_image)
    C, H, W = torch_image.shape

    t1 = time.time()

    # Extract features
    _, feat, seg, center, dense_feat = feature_extractor.extract(
        img=torch_image[None],
        return_centers=False,
        return_dense_features=True,
        n_random_pixels=100,
    )

    # Forward pass to predict traversability
    if prediction_per_pixel:
        data = Data(x=dense_feat[0].permute(1, 2, 0).reshape(-1, dense_feat.shape[1]))
    else:
        input_feat = feat[seg.reshape(-1)]
        data = Data(x=input_feat)

    # Inference model
    prediction = model_wvn.forward(data)

    # Calculate traversability
    if not anomaly_detection:
        out_trav = prediction.reshape(H, W, -1)[:, :, 0]
    else:
        losses = prediction["logprob"].sum(1) + prediction["log_det"]
        confidence = confidence_generator.inference_without_update(x=-losses)
        trav = confidence
        out_trav = trav.reshape(H, W, -1)[:, :, 0]
    
    trav, seg_img, cmap = plot_detectron_classification(torch_image, out_trav)

    t2 = time.time()

    ######################################
    ################ MORP ################
    ######################################
    out_trav_np = out_trav.cpu().detach().numpy()
    out_trav_np2, trav2 = preGroundSeg(out_trav_np, trav)
    # trav2, seg_img2 = preGroundSeg(trav, seg_img)
    temp_image, virtual_lane_available = GroundSeg(out_trav_np2, trav2)

    t3 = time.time()

    t4 = time.time()
    # handling lane
    cv2.line(temp_image, (0, UNAVAILABLE_THRES), (ROW, UNAVAILABLE_THRES), (0, 255, 0), 2)

    cv2.namedWindow('WVN MORP', cv2.WINDOW_NORMAL)
    cv2.imshow('WVN MORP', temp_image)
    # print("IMG PROC {}".format(t1-t_loop), 'TRV EST {}'.format(t2-t1), 'MORP {}'.format(t3-t2), 'CSV {}'.format(t4-t3))
    # print("NAV TIME {}".format(float(sim_time)-t0))
    #cv2.imshow('RealSense_depth', depth_image)
    if cv2.waitKey(1) == 27: #esc
        cv2.destroyAllWindows()
        rospy.signal_shutdown("esc")
        sys.exit(1)
    # print(virtual_lane_available)

def listener():
    #rospy.init_node('node_name')
    # bridge = CvBridge()
    rospy.Subscriber('/camera/color/image_raw', Image, img_callback)
    rospy.Subscriber("/odom", Odometry, state_callback)
    rospy.Subscriber("/clock", Clock, time_callback)
    rospy.Subscriber("/cmd_vel", Twist, cmd_callback)
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


new_checkpoint = OrderedDict()
for k, v in checkpoint['model'].items():
    name = k[7:]
    new_checkpoint[name] = v
model.load_state_dict(new_checkpoint)


best_error = checkpoint['error']
start_epoch = checkpoint['epoch']

optimizer = torch.optim.Adam(model.parameters(), lr=0.01)
criterion = nn.MSELoss()

t = time.time()

def main():
    # Configure depth and color streams
    global ROW, COL, GRN_ROI, bridge, sim_time, t0, virtual_lane_available, cmd_vel
    fpsFlag = False
    numFrame = 1
    fps = 0.0

    bridge = CvBridge()
    image_listener = threading.Thread(target=listener)
    image_listener.start()
    dist = 10.0

    obs_flg = 0
    t0 = sim_time

    print(dist)
    history_data = []

    model.eval()
    control_speed = 0
    control_steer = 0

    PI = 3.1415926535897

    easyGo.mvCurve(0.01/(2*PI/360), 0.0/(2*PI/360))

    while(dist > 0.8):
        #t1 = time.time()
        # global depth_image_raw, color_image_raw, robot_state, sim_time
        if type(virtual_lane_available) == type(0):
            sleep(0.1)
            # print(virtual_lane_available)
            # print(cmd_vel)
            print('waiting...')
            continue


        dist = math.sqrt((GOAL_X - robot_state[1])**2 + (-GOAL_Y - robot_state[0])**2)
        if obs_flg == 0 and dist < 10:
            # os.system("sh ./init.sh")
            obs_flg = 1
        
        virtual_lane_available = np.array(virtual_lane_available)
        # virtual_lane_available = UNAVAILABLE_THRES - virtual_lane_available # normalize. 0 means top of the image
        virtual_lane_available_rev = COL - virtual_lane_available
        temp = [(time.time()-t), (GOAL_X - robot_state[1]), (GOAL_Y + robot_state[0]), control_speed, control_steer, robot_state[2]] # yaw -
        temp.extend([x for x in virtual_lane_available_rev])

        if history_data == []:
            history_data = np.array(temp).reshape((1,-1))
            history_data = np.repeat(history_data, HISTORY, axis=0)
        else:
            history_data = np.roll(history_data, -1, axis=0) # data in the front is oldest
            history_data[-1] = np.array(temp)

        goal_x = history_data[:, 1]
        goal_y = history_data[:, 2]

        deadends = history_data[:, 5:] / 360.0

        commands = history_data[:, 3:5]

        dead_ends = np.hstack(deadends)
        commands = np.hstack(commands)
        goal_x = np.hstack(goal_x)
        goal_y = np.hstack(goal_y)

        model_input = list(dead_ends)
        model_input.extend(list(commands))
        model_input.extend(list(goal_x))
        model_input.extend(list(goal_y))
        
        print(model_input)

        with torch.no_grad():
            model_command = model(torch.FloatTensor(model_input))

        model_command = np.array(model_command)
        control_speed = model_command[0]
        control_steer = model_command[1]

        print(control_speed, control_steer)

        if numFrame < 15:
            easyGo.mvCurve(0.26/(2*PI/360), 0.0/(2*PI/360))
            print('numF < 15')
        else:
            easyGo.mvCurve(control_speed/(2*PI/360), control_steer/(2*PI/360))
            print('else')

        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', color_image)
        #print("NAV TIME {}".format(float(sim_time)-t0))
        #cv2.imshow('RealSense_depth', depth_image)
        if cv2.waitKey(1) == 27: #esc
            easyGo.stop()
            cv2.destroyAllWindows()
            rospy.signal_shutdown("esc")
            break
        # FPS
        numFrame += 1
    print("TOTAL TIME {}".format(float(sim_time) - t0))
    easyGo.stop()
    rospy.signal_shutdown("esc")

if __name__ == "__main__":
    rospy.init_node('robot_mvs', anonymous=False)
    
    main()
    exit()
