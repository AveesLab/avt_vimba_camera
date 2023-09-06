#!/bin/bash
# configure.sh

mkdir ~/ros2_ws/data
mkdir ~/ros2_ws/weights

cd ~/ros2_ws/src/clustering_sensor/avt_vimba_camera/launch
sed -i "s/{NODE_ID}/${AVEES_CLUSTERING_NODE_ID}/g" computing_node.py