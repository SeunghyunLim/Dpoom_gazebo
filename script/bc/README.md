## How to make dataset
### Start
``` bash
roslaunch dpoom_gazebo dppom_obs.launch

python3 myjoystick.py

python2 joy2cmd.py

<option> python2 ground_seg.py (to verify required topics)
```

### rosbag record
``` bash
cd <bag_record_directory>
#rosbag record -O subset <topic1> <topic2>
rosbag record -O subset /camera/depth/image_raw /camera/color/image_raw/compressed /cmd_vel /odom
# Ctrl+C to stop
```

### start dynamic obstacle
in 'script/agent' folder:
```bash
bash init.sh
```

### initialie pose after experiments
To initialize agent's positions
``` bash
python init_agent.py
```

## How to train model
``` bash
python train_morp_bc.py

```
## How to test trained model
``` bash
roslaunch dpoom_gazebo dppom_obs.launch

python2 bc2cmd.py

```