#!/bin/bash

echo  "Start"
roslaunch lidar_nodes lidar.launch &
sleep 3
roslaunch velodyne_pointcloud VLP16_points.launch &
sleep 3
rosrun rviz rviz -f velodyne &
sleep 3
rosrun rqt_graph rqt_graph &
sleep 3
echo "Finish"
