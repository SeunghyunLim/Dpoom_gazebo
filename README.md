# dpoom_gazebo

## Dependencies
linux-headers-generic
ros-melodic-turtlebot3-*

## Run
```
export TURTLEBOT3_MODEL=dpoom
roslaunch dpoom_gazebo dpoom.launch

export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

rosr rviz rviz -d dpoom_rviz.rviz ^C
```
