#!/bin/bash

echo  "Start"
rosrun rviz rviz -f velodyne &
sleep 3
rosrun rqt_graph rqt_graph &
sleep 3
echo "Finish"
