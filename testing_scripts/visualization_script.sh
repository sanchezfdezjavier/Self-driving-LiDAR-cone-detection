s
#!/bin/bash

echo  "Tools for visualization started"
roslaunch velodyne_pointcloud VLP16_points.launch &
rosrun rviz rviz -f velodyne &
rosrun rqt_graph rqt_graph &
echo "Ended"
