# Dpoom_gazebo
Gazebo simulation of Dpoom, based on turtlebot3 simulation.

## Dependencies
- Ubuntu 18.04
- ROS melodic
- Gazebo 9
- linux-headers-generic
- ros-melodic-turtlebot3-*

## Installing
On your catkin_ws,
```
git clone https://github.com/SeunghyunLim/Dpoom_gazebo.git
```
and catkin_make.

## Run
```
export TURTLEBOT3_MODEL=dpoom
roslaunch dpoom_gazebo dpoom.launch
```
```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
# for Rviz visualization
```
rosrun rviz rviz -d dpoom_rviz.rviz
```
## Reference
https://github.com/ROBOTIS-GIT/turtlebot3_simulations
