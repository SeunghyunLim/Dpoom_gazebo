# ECSC on the opencv image to quit

import numpy as np
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import time
import sys

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, CompressedImage, PointField
from nav_msgs.msg import Odometry

from std_msgs.msg import String, Header
from cv_bridge import CvBridge, CvBridgeError
import threading
from time import sleep
import csv

global depth_scale, ROW, COL
global currentStatus

rospy.init_node('pc2obs', anonymous=False)

#size of images
COL= 480
ROW = 640

#ROBOT MOVE
SPEED = 15
ROTATE_SPEED = 25

currentStatus = ""
font = cv2.FONT_HERSHEY_SCRIPT_SIMPLEX
fontScale = 1.5
yellow = (0, 255, 255)
handle_easy = True
points_raw = 0
color_image_raw = 0
cmd_vel = 0
robot_state = 0

t = time.time()


def euler_from_quaternion(x,y,z,w):
    t3 = 2.0*(w*z+x*y)
    t4 = 1.0-2.0*(y*y+z*z)
    yaw_z = math.atan2(t3,t4)
    return yaw_z
#Topview image. src, dst are numpy array.
#########################Move LAVACON TO EACH EDGES AND TRY AGAIN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
def Topview(src):
    global WARP_PARAM, ROW, COL
    # col=720, row=1280
    col, row = src.shape[0], src.shape[1]

    corners = np.float32([[row*WARP_PARAM/2, 0], [row*(1-WARP_PARAM/2), 0], [0, col], [row, col]])
    warp_corners = np.float32([[0, 0], [ROW, 0], [0, COL], [ROW, COL]])

    trans_matrix = cv2.getPerspectiveTransform(corners, warp_corners)

    dst = cv2.warpPerspective(src, trans_matrix, (ROW, COL))
    return dst


def preGroundSeg(depth_image, color_image):
    global ROW, COL, GRN_ROI
    # FIXME: don't do ZOOM_PARAM in GAZEBO
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

def points_callback(data):
    global points_raw
    points_raw = data

from rosgraph_msgs.msg import Clock
sim_time =0.0
def time_callback(data):
    global sim_time
    _sec = data.clock.secs
    _nsec = data.clock.nsecs
    sim_time = _sec + _nsec * 0.000000001

def image_callback(data):
    global color_image_raw
    color_image_raw = bridge.compressed_imgmsg_to_cv2(data, "bgr8")

def cmd_callback(data):
    global cmd_vel
    cmd_vel = data

def state_callback(data):
    global robot_state
    q = data.pose.pose.orientation
    yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
    yaw = yaw-math.pi/2 if yaw-math.pi/2 < math.pi else yaw+3*math.pi/2
    robot_state = [data.pose.pose.position.x, data.pose.pose.position.y, -yaw]

from rosgraph_msgs.msg import Clock
sim_time = 0.0
def time_callback(data):
    global sim_time
    _sec = data.clock.secs
    _nsec = data.clock.nsecs
    sim_time = _sec + _nsec * 0.000000001

def listener():
    rospy.Subscriber("/camera/depth/points", PointCloud2, points_callback)
    #rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, image_callback)
    # rospy.Subscriber("/gazebo/model_states", ModelStates, state_callback)
    rospy.Subscriber("/odom", Odometry, state_callback)
    rospy.Subscriber("/clock", Clock, time_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

direc = 0

def pc2obs_init():
    # Configure depth and color streams
    global bridge, pub

    bridge = CvBridge()

    realsense_listener = threading.Thread(target=listener)
    realsense_listener.start()
    pub = rospy.Publisher("obs_center", PointCloud2, queue_size=1)

fields = [PointField('x',0,PointField.FLOAT32,1),
            PointField('y',4,PointField.FLOAT32,1),
            PointField('z',8,PointField.FLOAT32,1)]
header = Header()
header.frame_id = "map"

def pc2obs(voxel_size = 0.3, plot=False, ros=True):
    global points_raw, color_image_raw, robot_state, bridge, currentStatus, handle_easy, pub, sim_time
    #print(points_raw)
    # if type(points_raw) == type(0) or type(color_image_raw) == type(0):
    if type(points_raw) == type(0) or sim_time == 0.0:
        print("NOT CONNECTED")
        sleep(0.1)
        return False, False, False

    t1 = time.time()
    points = pc2.read_points(points_raw, skip_nans=True, field_names=("x", "y", "z"))
    points = np.array(list(points), dtype=np.float32)
    if len(points) == 0:
        return False, False, False

    t2 = time.time()
    #print("length pre-processed points: {}".format(len(points)))
    np_vox = np.ceil((np.max(points, axis=0) - np.min(points, axis=0)) / voxel_size)
    non_empty_voxel_keys, inverse, nb_pts_per_voxel = np.unique(((points - np.min(points, axis=0)) // voxel_size).astype(int), axis=0, return_inverse=True, return_counts=True)
    idx_pts_sorted = np.argsort(inverse)
    voxel_grid = {}
    grid_barycenter, grid_candidate_center = [], []
    last_seen = int(0)

    for idx, vox in enumerate(non_empty_voxel_keys):
        voxel_grid[tuple(vox)] = points[idx_pts_sorted[int(last_seen):int(last_seen + nb_pts_per_voxel[idx])]]
        grid_barycenter.append(np.mean(voxel_grid[tuple(vox)], axis=0))
        grid_candidate_center.append(voxel_grid[tuple(vox)][np.linalg.norm(voxel_grid[tuple(vox)] - np.mean(voxel_grid[tuple(vox)], axis=0), axis=1).argmin()])
        last_seen += nb_pts_per_voxel[idx]
    points = np.array(list(filter(lambda x: x[0] != 0, list(grid_candidate_center))))

    t3 = time.time()
    points_layer = []
    for i, p in enumerate(points):
        # When the pointcloud has z-foward axis, the xyz-coordinate order should be [p[0], p[2], -p[1]].
        #if -p[1] > 0.1 and -p[1] < 0.6:
        #    points_layer.append([p[0], p[2], -p[1]])

        # When the pointcloud has x-foward axis, the xyz-coordinate order should be [-p[1], p[0], p[2]].
         if p[2] > 0.1 and p[2] < 0.6:
             points_layer.append([-p[1], p[0], p[2]])
    samples = np.array(points_layer)

    if plot:
        print("time took")
        print(t2-t1)
        print(t3-t2)
        print(time.time() - t3)

        plt.scatter(points[:,0], points[:,2], label='voxel grid filtering')
        if len(samples):
            plt.scatter(samples[:,0], samples[:,1], label='height filtering')
        plt.xlim(-1.5,1.5)
        plt.ylim(0,6)
        plt.legend()
        plt.title("Top view points after filter processing")
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.pause(0.05)
        plt.cla()
        plt.clf()

    color_image = color_image_raw
    # Show images
    #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    #cv2.imshow('RealSense', color_image)
    #cv2.imshow('RealSense_depth', depth_image)
    if cv2.waitKey(1) == 27: #esc
        cv2.destroyAllWindows()
        rospy.signal_shutdown("esc")
        if args.csv:
            f.close()
        sys.exit(1)

    if ros:
        pub_pc2 = pc2.create_cloud(header, fields, samples)
        pub_pc2.header.stamp = rospy.Time.now()
        pub.publish(pub_pc2)

    return samples, robot_state, sim_time

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--control', action='store_true')
    parser.add_argument('--plot', action='store_true')
    parser.add_argument('--csv', action='store_true')
    args = parser.parse_args()

    if args.csv:
        CSV_NAME = "office_01"

        f= open(CSV_NAME+'.csv','w')
        wr = csv.writer(f)
        wr.writerow(["time", \
                    "linear_x", "angular_z", \
                    "deadends"])

    pc2obs_init()
    while True:
        samples = pc2obs(voxel_size = 0.3, plot=args.plot)
        # print(samples)
    f.close()
    exit()
