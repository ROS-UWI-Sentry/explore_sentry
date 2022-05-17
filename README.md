# explore_sentry readme

We are in the process of migrating from ROS to ARC-OS but due to time constraints we will be using ROS for communication in our navigation task. The required folders and scripts are:
ard_com/scripts/ard_com_rec.py
explore_sentry/scripts/explore_sentry.py
explore_sentry/src/zed_depth.cpp
zed-ros-wrapper/zed_wrapper/launch/zed2.launch

the commands to launch are:

*new terminal
rosrun ard_com ard_com_rec.py

*new terminal
rosrun explore_sentry explore_sentry.py

*new terminal
cd catkin_ws
catkin_make
rosrun explore_sentry zed_depth

*new terminal
roslaunch zed_wrapper zed2.launch
