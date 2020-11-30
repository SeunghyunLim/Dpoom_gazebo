# Dpoom_gazebo
Gazebo simulation of Dpoom, based on turtlebot3 simulation.

<center><img src="https://github.com/SeunghyunLim/Dpoom_gazebo/blob/master/gif/dpoom_gazebo.gif" alt="drawing" width="720"/></center>

| Dpoom in real world | Dpoom in Gazebo |
|---|---|
|<center><img src="https://github.com/SeunghyunLim/Dpoom_gazebo/blob/master/img/dpoom_real.png" alt="drawing" width="360"/></center>|<center><img src="https://github.com/SeunghyunLim/Dpoom_gazebo/blob/master/img/dpoom_gazebo.png" alt="drawing" width="480"/></center>|


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
