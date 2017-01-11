#!/bin/bash
source /home/kevin/rosbuild_ws/setup.bash
rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10 &
rosrun tf static_transform_publisher 0 0 0 0 1 -1 0 map PointCloudFrame 10 &
rosrun tf static_transform_publisher 0 0 0 0 1 -1 0 map camera_depth_optical_frame 10 &
rosrun rviz rviz &
