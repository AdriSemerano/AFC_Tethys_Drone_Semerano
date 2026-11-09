#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 run ros_gz_sim create -world vfh_training_world -file tethys_equipped.sdf -name tethys -x 117.38 -y -117.94 -z 64.30 -Y 2.35
