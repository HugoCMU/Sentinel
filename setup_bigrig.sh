#!/bin/bash

# Make catkin folder
catkin_make -C /home/bigrig/Sentinel

# Source ROS and Catkin setup files
. /opt/ros/jade/setup.bash
. /home/bigrig/Sentinel/devel/setup.bash

# Set appropriate paths
export ROS_MASTER_URI=http://192.168.0.21:11311
export ROS_PACKAGE_PATH=/home/bigrig/Sentinel/src/rosserial:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=/home/bigrig/Sentinel/src/image_pipeline:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=/home/bigrig/Sentinel/src/cv_camera:$ROS_PACKAGE_PATH

# Set up namespace for stereo camera
#ROS_NAMESPACE=my_stereo rosrun stereo_image_proc stereo_image_proc
