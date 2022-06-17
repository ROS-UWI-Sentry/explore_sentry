# explore_sentry readme

We are in the process of migrating from ROS to ARC-OS but due to time constraints we will be using ROS for communication in our navigation task. The required folders and scripts are:
ard_com/scripts/ard_com_rec.py
explore_sentry/scripts/explore_sentry.py
explore_sentry/src/zed_depth.cpp
zed-ros-wrapper/zed_wrapper/launch/zed2.launch

the commands to launch are:
# new terminal
roscore

## new terminal
rosrun ard_com ard_com_rec.py

##  new terminal
rosrun explore_sentry state_machine.py

##  new terminal
rosrun explore_sentry system_model.py

##  new terminal
rosrun explore_sentry controls_act.py

## new terminal
```
cd catkin_ws
catkin_make
rosrun explore_sentry nav_sensor
```

## for zed camera
## new terminal
roslaunch zed_wrapper zed2.launch

## for lidar
### ensure that the static tf publisher is commented out in the launch file and the following params are: 
```
<param name="port"         type="string" value="/dev/ttyUSB0"/> 
<param name="angle_min"    type="double" value="-157.5" /> 
<param name="angle_max"    type="double" value="-22.5" />
```
```
open new terminal
cd ydlidar_ws
source ./devel/setup.sh
roslaunch ydlidar_ros_driver lidar.launch
```


## Network setup:
Find the IP of the robot's PC and the companion PC
In this example `@192.168.1.146` is the robot's PC and `192.168.1.108` is the companion PC
Change these accordingly in the code below:
In a new terminal:
```
ssh uwi-sentry-agx@192.168.1.103
export ROS_MASTER_URI=http://192.168.1.103:11311
export ROS_IP=192.168.1.103
```

On companion PC terminal:
```
export ROS_MASTER_URI=http://192.168.1.103:11311
export ROS_IP=192.168.1.105
```
For a permanent fix you can set these values in the PC's .bashrc file.(not the ssh command just the export)

