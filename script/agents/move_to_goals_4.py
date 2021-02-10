import rospy
import rospkg
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from utils import *

DPoom_init = [0.0, 3.0+4.0, 0.2, 1.57]
#DPoom_init = [0.0, 3.0, 0.2, 1.57]

init_list = ['', '', '', '', '']

# init_list[0] = [-0.5, -3.7, 0.2, 0.0]
# init_list[1] = [-2.5, 0, 0.2, -3.14]
# init_list[2] = [-2.5, -2, 0.2, -1.57]
# init_list[3] = [2.5, -2.5, 0.2, 1.0]
# init_list[4] = [1.5, 0.1, 0.2, 2.0]
init_list[0] = [-0.5, -3.7+4.0, 0.2, 0.0]
init_list[1] = [-2.5, 0+4.0, 0.2, -3.14]
init_list[2] = [-2.5, -2+4.0, 0.2, -1.57]
init_list[3] = [2.5, -2.5+4.0, 0.2, 1.0]
init_list[4] = [1.5, 0.1+4.0, 0.2, 2.0]

def main():
    rospy.init_node('set_pose')
    # Init pose for DPoom
    state_msg = ModelState()
    state_msg.model_name = 'dpoom'
    state_msg.pose.position.x = DPoom_init[0]
    state_msg.pose.position.y = DPoom_init[1]
    state_msg.pose.position.z = DPoom_init[2]
    (x, y, z, w) = eu2qut(yaw = DPoom_init[3])
    state_msg.pose.orientation.x = x
    state_msg.pose.orientation.y = y
    state_msg.pose.orientation.z = z
    state_msg.pose.orientation.w = w
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

    # Init pose for agents
    for i in range(5):
        state_msg = ModelState()
        state_msg.model_name = 'tb3_%s' % i
        state_msg.pose.position.x = init_list[i][0]
        state_msg.pose.position.y = init_list[i][1]
        state_msg.pose.position.z = init_list[i][2]
        (x, y, z, w) = eu2qut(yaw = init_list[i][3])
        state_msg.pose.orientation.x = x
        state_msg.pose.orientation.y = y
        state_msg.pose.orientation.z = z
        state_msg.pose.orientation.w = w

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass




#
# rosservice call /gazebo/set_model_state '{model_state: { model_name: tb3_0, pose: { position: { x: -1.74811633133, y: -0.996556746752 ,z: 0.188792623379 }, orientation: {x: -1.97054213262e-05, y: 0.00158948611954, z: 0.0114769113202, w: 0.999932874573 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'
#
#
# rosservice call /gazebo/set_model_state '{model_state: { model_name: tb3_0, pose: { position: { x: 1.74811633133, y: -0.996556746752 ,z: 0.188792623379 }, orientation: {x: -1.97054213262e-05, y: 0.00158948611954, z: 0.0114769113202, w: 0.999932874573 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'

    # position:
    #   x: -1.74811633133
    #   y: -0.996556746752
    #   z: 0.188792623379
    # orientation:
    #   x: -1.97054213262e-05
    #   y: 0.00158948611954
    #   z: 0.0114769113202
    #   w: 0.999932874573
