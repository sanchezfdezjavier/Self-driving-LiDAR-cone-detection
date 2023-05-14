# Lidar

## Requirements

You should have:

- Ubuntu 18.04
- ROS melodic installed

## How to start using this repo?

- The root folder of this repository is supposed to be your catkin workspace.

### Installation and Setup

1. **Clone** the repository and either source the new folder called "lidar" or change the name to your existing catkin workspace directory name.
2. **Install the necessary ros packages** by cloning them into the src folder:

- https://github.com/catkin/catkin_simple.git
- https://github.com/ethz-asl/glog_catkin.git

3. **Install glog**: `sudo apt-get install libgoogle-glog-dev`
4. **Install velodyne depencencies for ROS**: `sudo apt-get install ros-melodic-velodyne`
5. **Install ros-numpy**: `sudo apt-get install ros-melodic-ros-numpy`
6. **Install scikit-learn**: `pip install -U scikit-learn`
7. **Build** with `catkin_make`
8. **Source the setup** file by running: `source ~/<catkin_workspace_name>/devel/source.bash`. If you are using zsh instead of bash run: `source ~/<catkin_workspace_name/devel/source.zsh`

### Launch instructions

To launch the whole environment run: `roslaunch lidar_nodes lidar.launch`.

Getting up and running with **your own point cloud** source should be as simple as:

1. Change the `input_topic` parameter in `segmentation.launch` to your topic.**(Already configured, change only if you want to run your custom setup)**
2. Adjust the `sensor_height` parameter in `segmentation_params.yaml` to the height where the sensor is mounted on your robot (e.g. KITTI Velodyne: 1.8m)

### Visualize the data

1. Open RVIZ with the frame set to _velodyne_: `rosrun rviz rviz -f velodyne`
2. Click the **add** button and choose the topic you want to listen to:

- `/velodyne_points`: raw points captured by the lidar.
- `/prefiltered_points`: Remaining points after filters(depth, azimuth, etc.)
- `/ground_cloud`: Ground points.
- `/obstacle_cloud`: Cone pointcloud.
- `/visualization_marker`: Detected cones.

### Ground segmentation parameter description

Parameters are set in `linefit_ground_segmentation_ros/launch/segmentation_params.yaml`

This algorithm works on the assumption that you known the height of the sensor above ground.
Therefore, **you have to adjust the `sensor_height`** to your robot specifications, otherwise, it will not work.

The default parameters should work on the KITTI dataset.

### Ground Condition

- **sensor_height** Sensor height above ground.
- **max_dist_to_line** maximum vertical distance of point to line to be considered ground.
- **max_slope** Maximum slope of a line.
- **max_fit_error** Maximum error a point is allowed to have in a line fit.
- **max_start_height** Maximum height difference between new point and estimated ground height to start a new line.
- **long_threshold** Distance after which the max_height condition is applied.
- **max_height** Maximum height difference between line points when they are farther apart than _long_threshold_.
- **line_search_angle** How far to search in angular direction to find a line. A higher angle helps fill "holes" in the ground segmentation.

### Segmentation

- **r_min** Distance at which segmentation starts.
- **r_max** Distance at which segmentation ends.
- **n_bins** Number of radial bins.
- **n_segments** Number of angular segments.

### Other

- **n_threads** Number of threads to use.
- **latch** Latch output point clouds in ROS node.
- **visualize** Visualize the segmentation result. :warning: **ONLY FOR DEBUGGING.** Do not activate during operation.
