# Dpoom_gazebo
* This reopsitory contains __Gazebo simulation of Dpoom__, based on turtlebot3 simulation
* Covered urdf, ros topic publication, gazebo
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

## Topic
* Type : PointCloud2
* ROS topic : /camera/depth/points, /camera/depth/image_raw

## RGB-D Sensor Tilting
* You can tilt the sensor by changing pitch value of the _camera_rgb_joint_ in [dpoom.urdf.xacro](https://github.com/SeunghyunLim/Dpoom_gazebo/blob/master/urdf/dpoom.urdf.xacro)
```
  <joint name="camera_rgb_joint" type="fixed">
    <!--You can tilt the sensor by changing the value of pitch
        Ex) <origin xyz="0.003 0.011 0.009" rpy="0 0.3 0"/>-->
    <origin xyz="0.003 0.011 0.009" rpy="0 0 0"/>
    <parent link="camera_link"/>
    <child link="camera_rgb_frame"/>
  </joint>
```

| Default | Tilted by 0.3 rad |
|---|---|
|<center><img src="https://github.com/SeunghyunLim/Dpoom_gazebo/blob/master/img/original.png" alt="drawing" width="335"/></center>|<center><img src="https://github.com/SeunghyunLim/Dpoom_gazebo/blob/master/img/tilt.png" alt="drawing" width="335"/></center>|

## References
* https://github.com/ROBOTIS-GIT/turtlebot3_simulations
* http://gazebosim.org/tutorials?tut=ros_depth_camera&cat=connect_ros
