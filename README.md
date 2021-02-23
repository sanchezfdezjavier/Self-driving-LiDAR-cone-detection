# Lidar

## How to start using this repo?
* This code is supposed to work in ROS melodic with catkin. If you don't have it already, you must install it and come back later.
* The root folder of this repository is supposed to be your catkin workspace.

### Installation
1. __Clone__ the repository and either source the new folder called "lidar" or change the name to your existing catkin workspace directory name.
2. __Install the necessary dependencies__:
  * https://github.com/lorenwel/linefit_ground_segmentation.git
  * https://github.com/catkin/catkin_simple.git
  * https://github.com/ethz-asl/glog_catkin.git
  
 To __install the dependencies__, you only need to __clone them into the src folder__.
 If you want to use __catkin simple__(recommended) instead of ` catkin_make `.
 Compile using your favorite catkin build tool (i.e ` catkin build `).

### Launch instructions

The ground segmentation ROS node can be launch by executing `roslaunch linefit_ground_segmentation_ros segmentation.launch`.
Input and output topic names can be specified in the same file.

Getting up and running with your own point cloud source should be as simple as:

1. Change the `input_topic` parameter in `segmentation.launch` to your topic.
2. Adjust the `sensor_height` parameter in `segmentation_params.yaml` to the height where the sensor is mounted on your robot (e.g. KITTI Velodyne: 1.8m)

### Parameter description

Parameters are set in `linefit_ground_segmentation_ros/launch/segmentation_params.yaml`

This algorithm works on the assumption that you known the height of the sensor above ground. 
Therefore, **you have to adjust the `sensor_height`** to your robot specifications, otherwise, it will not work.

The default parameters should work on the KITTI dataset.

### Ground Condition
- **sensor_height**  Sensor height above ground.
- **max_dist_to_line**  maximum vertical distance of point to line to be considered ground.
- **max_slope**  Maximum slope of a line.
- **max_fit_error**  Maximum error a point is allowed to have in a line fit.
- **max_start_height**  Maximum height difference between new point and estimated ground height to start a new line.
- **long_threshold**  Distance after which the max_height condition is applied.
- **max_height**  Maximum height difference between line points when they are farther apart than *long_threshold*.
- **line_search_angle**  How far to search in angular direction to find a line. A higher angle helps fill "holes" in the ground segmentation.

### Segmentation

- **r_min**  Distance at which segmentation starts.
- **r_max**  Distance at which segmentation ends.
- **n_bins**  Number of radial bins.
- **n_segments**  Number of angular segments.

### Other

- **n_threads**  Number of threads to use.
- **latch**  Latch output point clouds in ROS node. 
- **visualize** Visualize the segmentation result. **ONLY FOR DEBUGGING.** Do not set true during online operation.
