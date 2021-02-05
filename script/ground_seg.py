######################################
#######realsense plotting#############
######################################
'''
Working with
    UBUNTU 16.04 LTS
    OPENCV 4.0~
    Python 3.6
    pyrealsense2
    matplotlib
    numpy
    for intel D435i
font
'''

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
import os

import easyGo
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import threading
from time import sleep
from rosgraph_msgs.msg import Clock
import csv

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--control', action='store_true')
parser.add_argument('--plot', action='store_true')
parser.add_argument('--csv', action='store_true')
args = parser.parse_args()

global depth_scale, ROW, COL

if args.csv:
    CSV_NAME = "office_01"

    f= open(CSV_NAME+'.csv','w')
    wr = csv.writer(f)
    wr.writerow(["time", \
                "linear_x", "angular_z", \
                "deadends"])

#size of images
COL= 480
ROW = 640

#ROBOT MOVE
SPEED = 15
ROTATE_SPEED = 25
ANGULAR_SPEED = 0.3

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
        try:
            # print(ground_center_idx[0])
            plt.plot(abs_x, abs_y)
            plt.scatter(abs_x[ground_center_idx[0]], abs_y[ground_center_idx[0]], color='r',
                        s=20)  # red point on start point of ground
            plt.scatter(abs_x[ground_center_idx[-1]], abs_y[ground_center_idx[-1]], color='r', s=20)
            plt.xlim(0, 4)  #5
            plt.ylim(-2, 2)

            plt.pause(0.05)
            plt.cla()
            plt.clf()
        except:
            pass

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

def LaneHandling(virtual_lane_available, unavailable_thres, n):
    center = int(len(virtual_lane_available)/2)

    #If center lane is blocked.
    if virtual_lane_available[center] > unavailable_thres:
        #two lanes are both unavailable
        if n > center:
            print("GO BACK")
            return 4
        if virtual_lane_available[center-n] > unavailable_thres and virtual_lane_available[center+n] > unavailable_thres:
            n+=1
            if n > center:
                print("GO BACK")
                return 4
            else:
                return LaneHandling(virtual_lane_available, unavailable_thres, n)
        elif virtual_lane_available[center-n] > unavailable_thres:
            print("TURN RIGHT")
            return 3
        elif virtual_lane_available[center+n] > unavailable_thres:
            print("TURN LEFT")
            return 2
        else:
            n += 1
            return LaneHandling(virtual_lane_available, unavailable_thres, n)
    #Checking where is future obstable and avoid it.
    else:
        if n > center:
            print("NO OBS")
            return 0 # no obstacle
        if virtual_lane_available[center-n] > unavailable_thres and virtual_lane_available[center+n] > unavailable_thres and n > 2:
            print("GO STRAIGHT")
            return 1 # robot can pass through
        if virtual_lane_available[center-n] > unavailable_thres:
            print("TURN RIGHT")
            return 3
        elif virtual_lane_available[center+n] > unavailable_thres:
            print("TURN LEFT")
            return 2
        else:
            n+=1
            return LaneHandling(virtual_lane_available, unavailable_thres, n)


def GoEasy(direc):
    if direc == 4: # Backward
        easyGo.mvStraight(- SPEED, -1)
    elif direc == 1: # Go straight
        easyGo.mvStraight(SPEED, -1)
    elif direc == 2: # turn left
        easyGo.mvRotate(ROTATE_SPEED, -1, False)
    elif direc == 3: # turn right
        easyGo.mvRotate(ROTATE_SPEED, -1, True)

def depth_callback(data):
    global depth_image_raw
    depth_image_raw = bridge.imgmsg_to_cv2(data, "32FC1")

def image_callback(data):
    global color_image_raw
    color_image_raw = bridge.compressed_imgmsg_to_cv2(data, "bgr8")

sim_time = 0.0
flg = 0

def time_callback(data):
    global sim_time
    _sec = data.clock.secs
    _nsec = data.clock.nsecs
    sim_time = _sec + _nsec * 0.000000001

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
    if args.csv:
        rospy.Subscriber("/cmd_vel", Twist, cmd_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def main():
    # Configure depth and color streams
    global depth_scale, ROW, COL, GRN_ROI, bridge
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
    global sim_time
    while sim_time == 0:
        global sim_time
    t1 = float(sim_time)
    while(dist > 0.8):
        #t1 = time.time()
        global depth_image_raw, color_image_raw, robot_state, sim_time
        if type(depth_image_raw) == type(0) or type(color_image_raw) == type(0):
            sleep(0.1)
            continue
        dist = math.sqrt((GOAL_X - robot_state[1])**2 + (-GOAL_Y - robot_state[0])**2)
        if obs_flg == 0 and dist < 10:
            os.system("sh ./init.sh")
            obs_flg = 1
        depth_image, color_image = preGroundSeg(depth_image_raw, color_image_raw)
        # last step
        color_image, virtual_lane_available = GroundSeg(depth_image, color_image)
        t2 = time.time()
        # handling lane
        cv2.line(color_image, (0, UNAVAILABLE_THRES), (ROW, UNAVAILABLE_THRES), (0, 255, 0), 2)

        if args.csv:
            virtual_lane_available = np.array(virtual_lane_available)
            # virtual_lane_available = UNAVAILABLE_THRES - virtual_lane_available # normalize. 0 means top of the image
            virtual_lane_available = COL - virtual_lane_available
            temp = [(time.time()-t), cmd_vel.linear.x, cmd_vel.angular.z]
            temp.extend([x for x in virtual_lane_available])
            wr.writerow(temp)

        t3 = time.time()
        direc = LaneHandling(virtual_lane_available, UNAVAILABLE_THRES, 1)
        if args.control:
            if direc == 0:
                diff_angle = (-robot_state[2] + math.atan2(GOAL_X - robot_state[1], -GOAL_Y - robot_state[0]))
                if diff_angle > 0:
                    v_ang = ANGULAR_SPEED * min(diff_angle/(math.pi/2), 1)
                else:
                    v_ang = ANGULAR_SPEED * max(diff_angle/(math.pi/2), -1)
                easyGo.mvCurve(SPEED, -v_ang)
            else:
                GoEasy(direc) # FIXME
        t4 = time.time()
        ground_seg_time += t2-t1
        lpp_time += t4-t3

        '''
        print("ground_seg took: {} sec".format(t2-t1))
        print("MORP took: {} sec".format(t4-t3))
        print("Average took: {} sec, {} sec, numFrame {}".format(ground_seg_time/numFrame, lpp_time/numFrame, numFrame))
        print("Distance to the Goal: {}".format(dist))
        print("flg: {}".format(obs_flg))
        '''

        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', color_image)
        print("NAV TIME {}".format(float(sim_time)-t1))
        #cv2.imshow('RealSense_depth', depth_image)
        if cv2.waitKey(1) == 27: #esc
            easyGo.stop()
            cv2.destroyAllWindows()
            rospy.signal_shutdown("esc")
            if args.csv:
                f.close()
            sys.exit(1)
            break
        # FPS
        numFrame += 1
    print(sim_time, t1)
    print("NAV TIME {}".format(float(sim_time)-t1))
    easyGo.stop()
    rospy.signal_shutdown("esc")

if __name__ == "__main__":
    rospy.init_node('robot_mvs', anonymous=False)
    main()
    if args.csv:
        f.close()
    exit()
