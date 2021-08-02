#!/usr/bin/env python

import rospy
import sys
import os
import math
import time
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
import argparse

import easyGo
import keyCap

parser = argparse.ArgumentParser()
parser.add_argument('--keyboard', action='store_true')
args = parser.parse_args()


rospy.init_node("Joy2Cmd")
#speedpub = rospy.Publisher(
#    '/cmros/car/speed', Float32, queue_size=1)

# Publishing Rate.
# rate = rospy.Rate(20)  # 30hz
print("Connecting")
totalTime = 0
count = 0

MAX_SPEED = 15
MAX_STEER = 25

csv_flag = False
CSV_NAME = "LaneChange_new"
DRIVE_INDEX = -1  # last drive index

'''
f= open(CSV_NAME+'.csv','w', encoding='utf-8', newline='')
wr = csv.writer(f)
wr.writerow(["drive_index", "time", \
            "longitudinal_acc", "lateral_acc", "roll_rate", "yaw_rate", "vhcl_steering_angle", "longitudinal_speed", \
            "roll_angle", "slip_angle", "lateral_speed", \
            "gas", "brake", "driver_steering_angle", "gearNo", \
            "car_yaw"])  
'''

def callback(data):
    global totalTime, count, csv_flag, DRIVE_INDEX
    
    
    '''
    print()
    print("Car speed: " + str(car_speed.data * 3.6) + " km/h")
    print("Car Yaw: " + str(car_yaw.data))
    print("Steering Angle: " + str(steering_angle.data))
    print("Simulation status: " + ("Running" if sim_status.data >=
                                    0 else cm.status_dic.get(sim_status.data)))
    '''

    '''
    if longitudinal_speed.data == 0:
        print(slip_angle.data, 0)
    else:
        print(slip_angle.data, math.atan2(lateral_speed.data, longitudinal_speed.data))
    
    floatmsg = Float32()
    floatmsg.data = car_speed.data * 3.6
    speedpub.publish(floatmsg)
    floatmsg.data = car_yaw.data
    yawpub.publish(floatmsg)
    floatmsg.data = steering_angle.data
    steerpub.publish(floatmsg)
    '''

    # csv_save = data.buttons[0] # A button
    ## CM vehicle control 
    control_speed = (data.axes[5] + 1) / 2 # Right trigger
    control_speed_back = (data.axes[2] + 1) / 2 # Left trigger
    #control_steer = - data.axes[3] # Right stick
    control_steer = - data.axes[0] # Left stick

    control_brake = data.buttons[5] # Right Button RB

    #if abs(control_steer) < 0.15:  # give a threshold due to the poor joystick
    #    control_steer = 0

    if control_speed_back:
        control_speed = - control_speed_back

    control_speed *= MAX_SPEED
    # control_brake *= MAX_BRAKE
    control_steer *= MAX_STEER

    start = data.buttons[4]

    control_steer = (int)(round(control_steer))
    control_speed = (int)(round(control_speed))
    
    if control_brake:
        easyGo.stop()
    else:
        easyGo.mvCurve(control_speed, control_steer)

    '''
    elif control_speed != 0:
        easyGo.mvStraight(control_speed, -1)
    elif control_steer > 0:
        easyGo.mvRotate(control_steer, -1, True)
    elif control_steer < 0:
        easyGo.mvRotate(-control_steer, -1, False)
    else:
        easyGo.stop()
    '''

    print("speed: ", control_speed, " , brake: ", control_brake, ", steer: ", control_steer)

    #rate.sleep()

    '''
    if sim_status.data == 0: # sim running
        if csv_flag == False:
            csv_flag = True
            DRIVE_INDEX += 1
        wr.writerow([DRIVE_INDEX, cm_time.data, \
            longitudinal_acc.data, lateral_acc.data, roll_rate.data, yaw_rate.data, vhcl_steering_angle.data, longitudinal_speed.data, \
            roll_angle.data, slip_angle.data, lateral_speed.data, \
            gas.data, brake.data, driver_steering_angle.data, gearNo.data, \
            car_yaw.data])
            

    else:
        if csv_flag ==True:
            csv_flag = False

    if csv_save:
        print("You Preseed A Button, File IO CLOSE !!!!!!!!!!!!!! ")
        f.close()
    '''
    


if __name__ == "__main__":

    cm_cmd = rospy.Subscriber("joy", Joy, callback, queue_size=1)

    rospy.init_node("Joy2Cmd")
    rospy.spin()
