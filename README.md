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

## RGB-D Sensor Height change
* You can change the sensor height by changing z origin value of the _camera_joint_in [dpoom.urdf.xacro](https://github.com/SeunghyunLim/Dpoom_gazebo/blob/master/urdf/dpoom.urdf.xacro)
```
  <joint name="camera_joint" type="fixed">
    <!--You can change the hight of the sensor by changing the value of origin z. 
        Total view-height is sum of this value and "camera_link"'s origin z value, which is 0.013
        Also, for visualization, the change in this part should be considered in "camera_rgb_frame".
        (Original height is 0.12m, so z should be 0.12 - 0.013 = 0.107)

     Ex) <origin xyz="0.073 -0.011 0.107" rpy="0 0 0"/>

     -->
    <origin xyz="0.073 -0.011 0.107" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="camera_link"/>
  </joint>
```
* For proper mesh visualization of the sensor, the change from above should be considered in _camera_rgb_frame_ of [dpoom.urdf.xacro](https://github.com/SeunghyunLim/Dpoom_gazebo/blob/master/urdf/dpoom.urdf.xacro)
```
  <link name="camera_rgb_frame">
    <!--Original value of visual origin is xyz="0 0 -0.01"
        if you changed the sensor height in "camera_joint", z should be deducted by the
        height change.

    EX) if you change the sensor height to 13cm(0.13m), which means that you changed 
        "camera_joint" z from 0.107 to 0.117, the visual z origin should be changed 
        from -0.01 to -0.02

    -->
    <visual>
      <origin xyz="0 0 -0.01" rpy="-1.57 0 -1.57"/>
      <geometry>
        <mesh filename="package://dpoom_gazebo/meshes/sensors/d435.dae" scale="1 1 1"/>
      </geometry>
    </visual>
  </link>
```
* Original sensor height should be fixed to 12cm(from bottom to the center of the sensor).
* The cylinder below has 12cm height, so you can check the sensor height is fixed to 12cm.
<center><img src="https://github.com/SeunghyunLim/Dpoom_gazebo/blob/master/img/rviz.png" alt="drawing" width="720"/></center>

## References
* https://github.com/ROBOTIS-GIT/turtlebot3_simulations
* http://gazebosim.org/tutorials?tut=ros_depth_camera&cat=connect_ros
