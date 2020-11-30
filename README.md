# Dpoom_gazebo
* This reopsitory contains __Gazebo simulation of Dpoom__, based on turtlebot3 simulation.
* You can visit our indoor SLAM robot in this page, [Project Dpoom](https://shinkansan.github.io/2019-UGRP-DPoom/)

<center><img src="https://github.com/SeunghyunLim/Dpoom_gazebo/blob/master/gif/dpoom_gazebo.gif" alt="drawing" width="720"/></center>

| Dpoom in real world | Dpoom in Gazebo |
|---|---|
|<center><img src="https://github.com/SeunghyunLim/Dpoom_gazebo/blob/master/img/dpoom_real.png" alt="drawing" width="285"/></center>|<center><img src="https://github.com/SeunghyunLim/Dpoom_gazebo/blob/master/img/dpoom_gazebo.png" alt="drawing" width="385"/></center>|


## Dependencies
- Ubuntu 18.04
- ROS melodic
- Gazebo 9
- linux-headers-generic
- ros-melodic-turtlebot3-*

## Installing
On your catkin_ws/src,
```
git clone https://github.com/SeunghyunLim/Dpoom_gazebo.git
```
and 'cd ~/catkin_ws && catkin_make'

## Run
for launching Gazebo,
```
export TURTLEBOT3_MODEL=dpoom
roslaunch dpoom_gazebo dpoom.launch
```
for teleop keyboard,
```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
for Rviz visualization,
```
rosrun rviz rviz -d dpoom_rviz.rviz
```
## Reference
https://github.com/ROBOTIS-GIT/turtlebot3_simulations
