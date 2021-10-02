#!/usr/bin/env python

import rospy
import sys
import os
import math
import time
import matplotlib.pyplot as plt
from os import listdir
from os.path import isfile, join
from std_msgs.msg import Float32
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
import csv
from cv_bridge import CvBridge, CvBridgeError
from time import sleep
import numpy as np

MODEL_NAME = 'h_10'
HISTORY = 10
parser = argparse.ArgumentParser(description='arguments')
parser.add_argument('--lr', type=float, default=0.0001,
                    help='learning rate')
parser.add_argument('--batch', type=int, default=32, help='batch_size')
parser.add_argument('--name', type=str, default='dagger__',
                    help='checkpoint file name')
parser.add_argument('--epoch', type=int, default=5, help='number of epoch')
parser.add_argument('--workers', type=int, default=1,
                    help='number of parallel data load workers')
parser.add_argument('--keyboard', action='store_true')
parser.add_argument('--control', action='store_true')
parser.add_argument('--plot', action='store_true')
parser.add_argument('--csv', action='store_true')
args = parser.parse_args()

totalTime = 0
count = 0

MAX_SPEED = 15
MAX_STEER = 25

csv_flag = False
DRIVE_INDEX = -1  # last drive index

COL= 480
ROW = 640

seed = 42
torch.manual_seed(seed)
np.random.seed(seed)


def imgmsg_to_cv2(img_msg):
    #if img_msg.encoding != "bgr8":
    #    rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    #dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    #image_opencv = np.ndarray(shape=(COL, ROW, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
    #                dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    #if img_msg.is_bigendian == (sys.byteorder == 'little'):
    #    image_opencv = image_opencv.byteswap().newbyteorder()

    image_opencv = np.zeros((COL, ROW, 3), dtype='uint8')
    return image_opencv

def depthmsg_to_cv2(img_msg):
    #if img_msg.encoding != "32fc1":
    #    rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("float32") # Hardcode to 8 bits...
    #dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 1), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    #if img_msg.is_bigendian == (sys.byteorder == 'little'):
    #    image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv


def depth_callback(data):
    global depth_image_raw
    # depth_image_raw = bridge.imgmsg_to_cv2(data, "32FC1")
    depth_image_raw =depthmsg_to_cv2(data)



from rosgraph_msgs.msg import Clock
sim_time =0.0
def time_callback(data):
    global sim_time
    _sec = data.clock.secs
    _nsec = data.clock.nsecs
    sim_time = _sec + _nsec * 0.000000001

def image_callback(data):
    global color_image_raw
    # color_image_raw = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    color_image_raw = imgmsg_to_cv2(data)

sim_time = 0.0
flg = 0


def joystick_callback(data):
    global joystick
    control_speed = (data.axes[5] + 1) / 2 # Right trigger
    control_speed_back = (data.axes[2] + 1) / 2 # Left trigger
    #control_steer = - data.axes[3] # Right stick
    control_steer = - data.axes[0] # Left stick

    if control_speed_back:
        control_speed = - control_speed_back

    control_speed *= MAX_SPEED
    control_steer *= MAX_STEER

    control_steer = (int)(round(control_steer))
    control_speed = (int)(round(control_speed))
    
    joystick = [control_speed, control_steer]

def state_callback(data):
    global robot_state
    q = data.pose.pose.orientation
    yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
    robot_state = [-data.pose.pose.position.y, data.pose.pose.position.x, -yaw]

def cmd_callback(data):
    global cmd_vel
    cmd_vel = data

def listener():
    #rospy.init_node('node_name')
    bridge = CvBridge()
    rospy.Subscriber("/camera/depth/image_raw", Image, depth_callback)
    rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, image_callback)
    rospy.Subscriber("/odom", Odometry, state_callback)
    rospy.Subscriber("/clock", Clock, time_callback)
    rospy.Subscriber("/cmd_vel", Twist, cmd_callback)
    rospy.Subscriber("joy", Joy, joystick_callback, queue_size=1)
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

#size of images
COL= 480
ROW = 640

#ROBOT MOVE
SPEED = 0.26 # 0.26 (m/s)
ROTATE_SPEED = 25 # 25 (deg/s)
ANGULAR_SPEED = 0.2

# Set goal position
GOAL_X = 0
GOAL_Y = 3

VERTICAL_CORRECTION = 0.35 # 0.15 #0.45  #parameter of correction for parabola to linear
WARP_PARAM = 0.45  #value should be 0.0 ~ 1.0. Bigger get more warped. 0.45
GRN_ROI = 200 #The index of col that want to consider as ground ROI 400 300
ZOOM_PARAM = 0.15 #Force calibrating the depth image to match the color image 0.15 0.205
UNAVAILABLE_THRES = 200 #  #The index of col that is threshold of unavailable virtual lane. defalut 170
ROBOT_WIDTH_LIST = [2,3,4,5]
ROBOT_LEFT = 1
ROBOT_RIGHT = 6
font = cv2.FONT_HERSHEY_SCRIPT_SIMPLEX
fontScale = 1.5
yellow = (0, 255, 255)
depth_image_raw = 0
color_image_raw = 0
robot_state = [0, -8, 0]
cmd_vel = 0
joystick = [0.0, 0.0]

t = time.time()

def euler_from_quaternion(x,y,z,w):
    t3 = 2.0*(w*z+x*y)
    t4 = 1.0-2.0*(y*y+z*z)
    yaw_z = math.atan2(t3,t4)
    return yaw_z

#Topview image. src, dst are numpy array.
def Topview(src):
    global WARP_PARAM, ROW, COL
    # col=720, row=1280
    col, row = src.shape[0], src.shape[1]

    corners = np.float32([[row*WARP_PARAM/2, 0], [row*(1-WARP_PARAM/2), 0], [0, col], [row, col]])
    warp_corners = np.float32([[0, 0], [ROW, 0], [0, COL], [ROW, COL]])

    trans_matrix = cv2.getPerspectiveTransform(corners, warp_corners)

    dst = cv2.warpPerspective(src, trans_matrix, (ROW, COL))
    return dst


#vertically scan ground
def verticalGround(depth_image2, images, numCol, plot):
    global depth_scale, GRN_ROI, ROW, COL

    ###################################################################################
    #############Calibration. CHECK HERE WHEN YOU USE DIFFERENT CAMERA!!###############
    ###################################################################################
    numLine=numCol

    #Force correction. Depth and color pixels don't match.
    numCol=(int)((numCol-150)*(-0.15)+numCol)

    # get [i,640] column
    _640col = [a[numCol] for a in depth_image2]

    abs_x = []
    abs_y = []
    ground_center_idx = []

    # depth_image2[360] = depth_image2[360] * float(depth_scale)
    for idx, temp in enumerate(_640col):
        if _640col[idx] == 0:
            abs_x.append(None)
            abs_y.append(None)
        else:
            #true_idx is a calibrated idx. Because the start point is not zero. Start point of ROI is GRN_ROI.
            true_idx = GRN_ROI + idx*(COL-GRN_ROI)/float(COL)
            #Correction for parabola to linear. In other words, correcting lens distortion using 2nd-order function.
            _640col[idx] = temp * depth_scale * (abs(true_idx -COL/2)**2/float(360**2)*VERTICAL_CORRECTION + 1)
            #58.0 is vertical FOV of the depth IR sensor. abs_x and abs_y are absolute coordinate of one column of depth image.
            abs_x.append(
                _640col[idx] * math.cos(
                    ((float)(58.0 / 2.0 - 58.0 * (float)(true_idx) / COL)) * 3.14 / 180.0))
            abs_y.append(
                _640col[idx] * math.sin((float)(58.0 / 2.0 - 58.0 * (float)(true_idx) / COL) * 3.14 / 180.0))

    idx = 20 #temporary set the point, that we want to consider it would be almost ground.
    try:
        while abs_x[COL - idx] == None:
            idx += 1
        ground_center_idx.append(COL - idx)  #ground_center_idx contains all the indexes of ground.
    except:
        print("TOO CLOSE!!!!!!!!!!!!!")
        ground_center_idx.append(COL - 30)
    i = 0
    groundCount = 0  #Count points that considered as ground
    hurdleCount = 0  #Count points that not considered as ground subsequently. Initialize it to zero when found ground.
    while idx < COL:
        #try:
        if abs_x[COL - idx] == None or abs_y[COL - idx] == None:
            idx += 1
            #print(idx)
            continue
        # (abs(abs_x[ground_center_idx[i]] - abs_x[(720 - idx)]) < 0.4) and (

        #To found ground indexes, we use differential. If variation dy/dx is lower than threshold, append it.
        ####################################################################################################
        #######19/04/26 : I have updated the way of checking gradient. Now I use two gradients##############
        #######from original, and from the current ground pixel. It works better ###########################
        ####################################################################################################
        gradient_from_original = (abs_y[(COL - idx)] - abs_y[ground_center_idx[0]]) / float(abs_x[(COL - idx)] - abs_x[ground_center_idx[0]])
        gradient_from_current = (abs_y[(COL - idx)] - abs_y[ground_center_idx[i]]) / float(abs_x[(COL - idx)] - abs_x[ground_center_idx[i]])

        #print("dist" + str(_640col[COL - idx]))
        if abs(gradient_from_original + 0.13) < 0.2 and abs(gradient_from_current + 0.133) < 0.15:   #These number are carefully selected
            #print("origin: ", gradient_from_original, "current: ", gradient_from_current)
            ground_center_idx.append((COL - idx))
            i += 1
            cv2.circle(images, (numLine, (COL - idx)), 7, (0, 255, 0), 2)
            groundCount += 1
            hurdleCount = 0
            # print(idx)
            idx += 20
        elif hurdleCount > 1:
            break
        else:
            hurdleCount += 1
            idx += 20

    if plot:
        #print(abs_x[ground_center_idx[0]], abs_y[ground_center_idx[0]])
        #print(abs_x[ground_center_idx[-1]], abs_y[ground_center_idx[-1]])
        #print((abs_x[ground_center_idx[-1]]-abs_x[ground_center_idx[0]])/(abs_y[ground_center_idx[-1]]-abs_y[ground_center_idx[0]]))
            # print(ground_center_idx[0])
        plt.plot(abs_x, abs_y)
        plt.scatter(abs_x[ground_center_idx[0]], abs_y[ground_center_idx[0]], color='r',
                    s=20)  # red point on start point of ground
        plt.scatter(abs_x[ground_center_idx[-1]], abs_y[ground_center_idx[-1]], color='r', s=20)
        plt.xlim(0, 10)  #5
        plt.ylim(-2, 2)

        plt.pause(0.05)
        plt.cla()
        plt.clf()
        #print("error")

    if ground_center_idx[-1] > UNAVAILABLE_THRES:
        #dead_end = COL
        dead_end = ground_center_idx[-1]
        cv2.line(images, (numLine, 0), (numLine, ROW), (0, 0, 255), 5)  #Draw a red line when ground indexes is less than we want.
    else:
        dead_end = ground_center_idx[-1]
        cv2.line(images, (numLine, ground_center_idx[-1]), (numLine, COL), (0, 255, 0), 5) #Draw a green line.

    try:

        #pass
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)5
        cv2.circle(images, (numLine, ground_center_idx[0]), 5, (255, 255, 255), 10)
        cv2.putText(images, str(round(abs_x[ground_center_idx[0]],2)) + "m", (numLine, COL - 100), font, fontScale, yellow, 2)
    except:
        print("error2")
        pass

    return images, dead_end


def preGroundSeg(depth_image, color_image):
    global ROW, COL, GRN_ROI
    # FIXME: don't do ZOOM_PARAM in GAZEBO
        # Force calibrating the depth image to match the color image. Interpolation is really important. DO NOT USE INTER_LINEAR. IT MAKES NOISES!!!!
    #depth_image = cv2.resize(depth_image[(int)(COL * ZOOM_PARAM):(int)(COL * (1 - ZOOM_PARAM)), (int)(ROW * ZOOM_PARAM):(int)(ROW * (1 - ZOOM_PARAM))],
    #						 dsize=(ROW, COL), interpolation=cv2.INTER_NEAREST)

    # ROI image
    depth_image = depth_image[GRN_ROI:COL, 0:ROW]
    color_image = color_image[GRN_ROI:COL, 0:ROW]

    # Topview image
    depth_image2 = Topview(depth_image)
    color_image2 = Topview(color_image)
    return depth_image2, color_image2


def GroundSeg(depth_image, color_image, stride=80):
    global ROW
    virtual_lane_available = []
    for i in range(stride, ROW, stride):
        if args.plot and i == ROW/2:
            temp_image, dead_end = verticalGround(depth_image, color_image, i, plot=True)
        else:
            temp_image, dead_end = verticalGround(depth_image, color_image, i, plot=False)
        virtual_lane_available.append(dead_end)
    return temp_image, virtual_lane_available

def bool_straight(virtual_lane_available, unavailable_thres):
    global ROBOT_WIDTH_LIST
    for i in ROBOT_WIDTH_LIST:
        # > means unavailable path
        if virtual_lane_available[i] > unavailable_thres:
            return False
    return True


def sample_trajectory(model, beta=0.75):
    # Configure depth and color streams
    global depth_scale, ROW, COL, GRN_ROI, bridge, sim_time
    fpsFlag = False
    numFrame = 1
    fps = 0.0

    bridge = CvBridge()
    realsense_listener = threading.Thread(target=listener)
    realsense_listener.start()
    depth_scale = 1.0
    startTime = time.time()
    ground_seg_time = 0.0
    lpp_time = 0.0
    dist = 10.0

    obs_flg = 0
    #while sim_time == 0.0:
    #    continue
    t0 = sim_time

    history_data = []

    model.eval()
    control_speed = 0
    control_steer = 0

    PI = 3.1415926535897

    collected_data_x = []
    collected_data_y = []

    while(dist > 0.8):
        #t1 = time.time()
        # global depth_image_raw, color_image_raw, robot_state, sim_time
        global joystick, depth_image_raw, color_image_raw
        if type(depth_image_raw) == type(0) or type(color_image_raw) == type(0):
            sleep(0.1)
            continue


        dist = math.sqrt((GOAL_X - robot_state[1])**2 + (-GOAL_Y - robot_state[0])**2)
        if obs_flg == 0 and dist < 10:
            # os.system("sh ./init.sh")
            obs_flg = 1
        depth_image, color_image = preGroundSeg(depth_image_raw, color_image_raw)
        # last step
        color_image, virtual_lane_available = GroundSeg(depth_image, color_image)
        t2 = time.time()
        # handling lane
        cv2.line(color_image, (0, UNAVAILABLE_THRES), (ROW, UNAVAILABLE_THRES), (0, 255, 0), 2)

        virtual_lane_available = np.array(virtual_lane_available)
        # virtual_lane_available = UNAVAILABLE_THRES - virtual_lane_available # normalize. 0 means top of the image
        virtual_lane_available = COL - virtual_lane_available
        temp = [(time.time()-t), (GOAL_X - robot_state[1]), (GOAL_Y + robot_state[0]), control_speed, control_steer, -robot_state[2]]
        temp.extend([x for x in virtual_lane_available])

        if history_data == []:
            history_data = np.array(temp).reshape((1,-1))
            history_data = np.repeat(history_data, HISTORY, axis=0)
        else:
            history_data = np.roll(history_data, -1, axis=0) # data in the front is oldest
            history_data[-1] = np.array(temp)

        goal_x = history_data[:, 1]
        goal_y = history_data[:, 2]
        yaw = history_data[:, 5]

        deadends = history_data[:, 6:] / 360.0

        commands = history_data[:, 3:5]

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

        # aggregate dataset
        if -robot_state[0] > -8.0:
            print("append data", [joystick[0]*(2*PI/360), joystick[1]*(2*PI/360)])
            collected_data_x.append(model_input)
            collected_data_y.append([joystick[0]*(2*PI/360), joystick[1]*(2*PI/360)])

        model_command = np.array(model_command)

        control_speed = (1-beta)*model_command[0] + beta*joystick[0]*(2*PI/360)
        control_steer = (1-beta)*model_command[1] + beta*joystick[1]*(2*PI/360)
        print(control_speed, control_steer)
        #print("="*20)

        easyGo.mvCurve(control_speed/(2*PI/360), control_steer/(2*PI/360))

        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', color_image)
        #print("NAV TIME {}".format(float(sim_time)-t0))
        #cv2.imshow('RealSense_depth', depth_image)
        if cv2.waitKey(1) == 27: #esc
            easyGo.mvCurve(0.0, 0.0)
            easyGo.stop()
            cv2.destroyAllWindows()
            rospy.signal_shutdown("esc")
            if args.csv:
                f.close()
            sys.exit(1)
            break
        # FPS
        numFrame += 1
    print("TOTAL TIME {}".format(float(sim_time) - t0))
    easyGo.stop()
    #rospy.signal_shutdown("esc")

    return collected_data_x, collected_data_y


skip_data_front = 10 # 

def csv2list(filename):
    raw_data = []
    with open(filename, newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')

        for i, row in enumerate(spamreader):
            raw_data.append(row)

    return np.array(raw_data[1+skip_data_front:], dtype='float32')

def make_history_data(split_data):
    # [deadends_1, deadends_2, ..., goal_y_H] (H history)
    total_history_data = []
    total_answer = []
    split_data = np.array(split_data)
    for data in split_data:
        for i in range(HISTORY-1, len(data)-HISTORY):  # last data only used for answer

            history_data = data[i-HISTORY+1:i+1]

            history_data = np.array(history_data)
            goal_x = history_data[:, 1]
            goal_y = history_data[:, 2]
            yaw = history_data[:, 5]

            deadends = history_data[:, 6:] / 360.0

            commands = history_data[:, 3:5]

            dead_ends = np.hstack(deadends)
            commands = np.hstack(commands)
            goal_x = np.hstack(goal_x)
            goal_y = np.hstack(goal_y)
            yaw = np.hstack(yaw)

            split_history_data = list(dead_ends)
            split_history_data.extend(list(commands))
            split_history_data.extend(list(goal_x))
            split_history_data.extend(list(goal_y))
            split_history_data.extend(list(yaw))
            #print(split_history_data)
            answer = data[i+1][[3, 4]]  # multiple indexing

            total_history_data.append(split_history_data)
            total_answer.append(list(answer))

    return total_history_data, total_answer



class CustomDataset(Dataset):
    def __init__(self, x, y):
        self.x_data = x
        self.y_data = y

    def __len__(self):
        return len(self.x_data)

    def __getitem__(self, item):
        x_ = torch.FloatTensor(self.x_data[item])
        y_ = torch.FloatTensor(self.y_data[item])
        return x_, y_

                          
def train(model, epoch, train_loader):
    # print('Training {} batches, {} data'.format(len(train_loader), len(train_loader)*args.batch))
    model.train()

    for i in range(epoch):
        train_loss = torch.FloatTensor([0])
        for batch_idx, samples in enumerate(train_loader):
            x_train, y_train = samples
            x_train, y_train = x_train.to(device), y_train.to(device)
            prediction = model(x_train)
            loss = criterion(prediction, y_train)
            optimizer.zero_grad(set_to_none=True)  # efficient zero out
            loss.backward()
            optimizer.step()

            train_loss += loss

        # scheduler.step()
        err = math.sqrt(float(train_loss.item() / float(len(train_loader))))
        print('Training ==> Epoch {:2d} / {} Cost: {:.6f}'.format(i, args.epoch,
                                                                err))
    print("Training Finish !")
    return model, err

device = 'cpu'

# load pretrained model

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
model.load_state_dict(checkpoint['model'])
best_error = checkpoint['error']
start_epoch = checkpoint['epoch']

optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
criterion = nn.MSELoss()

def dagger():
    global model
    # load dataset
    train_dataX = []
    train_dataY = []
    data_folder = 'experts_data/'
    files = [f for f in listdir(data_folder) if isfile(join(data_folder, f))]
    print("dataset files list (total ", len(files), " files ): ", files)

    split_data = []
    for filename in files:
        split_data.append(csv2list(data_folder+filename))
    x, y = make_history_data(split_data)
    train_dataX.extend(x)
    train_dataY.extend(y)

    # DAgger
    beta = 0.25
    episode_length = 5
    for i in range(episode_length):
        x, y = sample_trajectory(model, beta)
        train_dataX.extend(x)
        train_dataY.extend(y)
        train_dataset = CustomDataset(train_dataX, train_dataY)
        train_loader = DataLoader(train_dataset, batch_size=args.batch, shuffle=True,
                            num_workers=args.workers, pin_memory=True)  # shuffle every epoch

        model, err = train(model, args.epoch, train_loader)
        state = {
                'model': model.state_dict(),
                'error': err,
                'epoch': i,
        }
        if not os.path.isdir('checkpoint'):
            os.mkdir('checkpoint')
        torch.save(state, './checkpoint/' + args.name + str(i) + '.pth')

        print("Ready for next run ? (Press Anything)")
        input()
    

if __name__ == "__main__":
    rospy.init_node('robot_mvs', anonymous=False)
    
    dagger()
    easyGo.mvCurve(0.0, 0.0)
    easyGo.stop()
    cv2.destroyAllWindows()
    rospy.signal_shutdown("esc")
    if args.csv:
        f.close()
    sys.exit(1)
