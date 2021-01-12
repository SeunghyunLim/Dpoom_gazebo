
import numpy as np
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import time
import sys

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, CompressedImage
from geometry_msgs.msg import Twist

import easyGo
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import threading
from time import sleep
import csv

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--control', action='store_true')
parser.add_argument('--plot', action='store_true')
parser.add_argument('--csv', action='store_true')
args = parser.parse_args()

global depth_scale, ROW, COL
global currentStatus

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

currentStatus = ""
font = cv2.FONT_HERSHEY_SCRIPT_SIMPLEX
fontScale = 1.5
yellow = (0, 255, 255)
handle_easy = True
points_raw = 0
color_image_raw = 0
cmd_vel = 0

t = time.time()

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


def GoEasy(direc):
	if direc == 4:
		easyGo.mvStraight(- SPEED, -1)
	elif direc == 0 or direc == 1:
		#print("COME HERE")
		easyGo.mvStraight(SPEED, -1)
	elif direc == 2:
		#print("COME HERE2")
		easyGo.mvRotate(ROTATE_SPEED, -1, False)
	elif direc == 3:
		easyGo.mvRotate(ROTATE_SPEED, -1, True)

def points_callback(data):
	global points_raw
	points_raw = data

def image_callback(data):
	global color_image_raw
	color_image_raw = bridge.compressed_imgmsg_to_cv2(data, "bgr8")


def cmd_callback(data):
	global cmd_vel
	cmd_vel = data

def listener():
	rospy.Subscriber("/camera/depth/points", PointCloud2, points_callback)
	rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, image_callback)
	if args.csv:
		rospy.Subscriber("/cmd_vel", Twist, cmd_callback)
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

direc = 0

def main():
	# Configure depth and color streams
	global depth_scale, ROW, COL, GRN_ROI, bridge, direc
	fpsFlag = False
	numFrame = 0
	fps = 0.0

	bridge = CvBridge()

	realsense_listener = threading.Thread(target=listener)
	realsense_listener.start()


	#COL=720, ROW=1280
	# depth_scale = 0.0010000000474974513
	# depth_scale = 1.0
	startTime = time.time()
	while True:
		# first step
		global points_raw, color_image_raw, currentStatus, handle_easy
		#print(points_raw)
		if type(points_raw) == type(0) or type(color_image_raw) == type(0):
			print("NOT CONNECTED")
			sleep(0.1)
			continue

		points = pc2.read_points(points_raw, skip_nans=True, field_names=("x", "y", "z"))
		points_plane_x = []
		points_plane_y = []
		points_plane_z = []
		for i, p in enumerate(points):
			#if i % 500 == 0:
			if abs(-p[1] - 0.3) < 0.02: # points are at 0.2m higher height than depth camera
				# forward:x  . right:y,  up:z
				points_plane_x.append(p[0])
				points_plane_y.append(p[2])
				points_plane_z.append(-p[1])
		print(len(points_plane_x))
		#fig = plt.figure()
		#ax = fig.gca(projection='3d')
		#ax.scatter3D(points_plane_x, points_plane_y, points_plane_z)
		#ax.set_xlabel("x")
		#ax.set_ylabel("y")
		#ax.set_zlabel("z")

		plt.scatter(points_plane_x, points_plane_y)
		plt.pause(0.05)
		plt.cla()
		plt.clf()
		
		color_image = color_image_raw

		if args.csv:
			virtual_lane_available = np.array(virtual_lane_available)
			# virtual_lane_available = UNAVAILABLE_THRES - virtual_lane_available # normalize. 0 means top of the image
			virtual_lane_available = COL - virtual_lane_available
			temp = [(time.time()-t), cmd_vel.linear.x, cmd_vel.angular.z]
			temp.extend([x for x in virtual_lane_available])
			wr.writerow(temp)
			# sleep(n secs)

		if direc == 0:
			currentStatus = "NO OBSTACLE"
			rospy.set_param('/point_init', True)
		else:
			currentStatus = "YES OBSTACLE"
		#print(direc)
		if handle_easy:
			easyGo.stopper=handle_easy
			if args.control:
				GoEasy(direc) # FIXME
			#print('Morp easyGo stoppper :: ' + str(easyGo.stopper))
		#LaneHandling(virtual_lane_available, UNAVAILABLE_THRES, 1)

		# Stack both images horizontally
		# images = np.hstack((color_image, depth_colormap))

		# Show images
		cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
		cv2.imshow('RealSense', color_image)
		#cv2.imshow('RealSense_depth', depth_image)
		if cv2.waitKey(1) == 27: #esc
			easyGo.stop()
			cv2.destroyAllWindows()
			rospy.signal_shutdown("esc")
			if args.csv:
				f.close()
			sys.exit(1)
		#	break

		# FPS
		numFrame += 1

		#if cv2.waitKey(1) == ord('f'):
		#	endTime = time.time()
		#	fps = round((float)(numFrame) / (endTime - startTime), 2)
		#	print("###FPS = " + str(fps) + " ###")

		#except Exception:
		#	print("Error")
			#pipeline.stop()
			#easyGo.stop()

	# Stop streaming
	#pipeline.stop()
	easyGo.stop()

if __name__ == "__main__":
	rospy.init_node('robot_mvs', anonymous=False)
	main()

	f.close()
	exit()