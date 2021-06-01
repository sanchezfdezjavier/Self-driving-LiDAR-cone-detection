#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import ros_numpy as rnp

import numpy as np

import uuid

from sklearn.cluster import DBSCAN
from sklearn import metrics
from sklearn.datasets import make_blobs
from sklearn.preprocessing import StandardScaler
import random


# TODO: Finish, z? how to remove starray
def get_PoseArray(idxs_by_cluster, nparray_obstacle_xy, starray_obstacle_xyz, timestamp):

    clusters_keys = idxs_by_cluster.keys()

    pose_array = PoseArray()

    pose_array.header.stamp = timestamp
    pose_array.header.frame_id = "/velodyne"

    for _, idx in enumerate(clusters_keys):
        pose = Pose()

        # Cone position
        pose.position.x = nparray_obstacle_xy[idxs_by_cluster[idx][-1]][0]
        pose.position.y = nparray_obstacle_xy[idxs_by_cluster[idx][-1]][1]
        pose.position.z = starray_obstacle_xyz['z'][idxs_by_cluster[idx][-1]]

        # Not useful
        # pose.orientation.w = 0
        # pose.orientation.x = 0
        # pose.orientation.y = 0
        # pose.orientation.z = 0

        pose_array.poses.append(pose)

    return pose_array

def get_cone_marker(timestamp, id, x, y, z, r, g, b, a):
    lidar_marker = Marker()
    lidar_marker.header.frame_id = "/velodyne"
    lidar_marker.header.stamp = timestamp
    lidar_marker.ns = "aux_marker"
    lidar_marker.action = Marker.ADD
    lidar_marker.id = id
    lidar_marker.type = lidar_marker.CYLINDER
    lidar_marker.scale.x = 0.3
    lidar_marker.scale.y = 0.3
    lidar_marker.scale.z = 0.3
    lidar_marker.color.a = a
    lidar_marker.color.r = r
    lidar_marker.color.g = g
    lidar_marker.color.b = b
    lidar_marker.pose.orientation.w = 1.0
    lidar_marker.pose.position.x = x
    lidar_marker.pose.position.y = y
    lidar_marker.pose.position.z = z

    return lidar_marker

def get_LiDAR_marker(timestamp):
    '''
     Returns the marker representing the LiDAR itself in RVIZ
    '''
    lidar_marker = Marker()
    lidar_marker.header.frame_id = "/velodyne"
    lidar_marker.header.stamp = timestamp
    lidar_marker.ns = "cone_markers"
    lidar_marker.action = Marker.ADD
    lidar_marker.id = 250
    lidar_marker.type = lidar_marker.CYLINDER
    lidar_marker.scale.x = 0.3
    lidar_marker.scale.y = 0.3
    lidar_marker.scale.z = 0.3
    lidar_marker.color.a = 1
    lidar_marker.color.r = 0
    lidar_marker.color.g = 1
    lidar_marker.color.b = 1
    lidar_marker.pose.orientation.w = 1.0
    lidar_marker.pose.position.x = 0
    lidar_marker.pose.position.y = 0
    lidar_marker.pose.position.z = 0

    return lidar_marker

# TODO: Efficiency can be improved by better params
def publish_markers_to_RVIZ(idxs_by_cluster, nparray_obstacle_xy, starray_obstacle_xyz):
    clusters_keys = idxs_by_cluster.keys()
    marker_array = MarkerArray()
    timestamp = rospy.Time.now()
    marker_array.markers.append(get_LiDAR_marker(timestamp))
	
    marker_array.markers.append(get_cone_marker(timestamp, 100, 5, 0, 0, 1, 0, 0, 0.5))
    marker_array.markers.append(get_cone_marker(timestamp, 101, 10, 0, 0, 1, 0, 0, 0.5))
    marker_array.markers.append(get_cone_marker(timestamp, 111, 8, 0, 0, 1, 0, 0, 0.5))
    marker_array.markers.append(get_cone_marker(timestamp, 102, 0, 5, 0, 1, 0, 0, 0.5))
    marker_array.markers.append(get_cone_marker(timestamp, 103, 0, -5, 0, 1, 0, 0, 0.5))
    marker_array.markers.append(get_cone_marker(timestamp, 104, 5, 0, 0, 1, 0, 0, 0.5))

    # id: iteration element -> cluster key
    # idx: iteration index 
    for id, idx in enumerate(clusters_keys):
        # TODO: Generate marker standalone function
        marker = Marker()
        marker.header.frame_id = "/velodyne"
        marker.header.stamp = timestamp
        marker.ns = "cone_markers"
        marker.lifetime = rospy.Duration(0.1)
        marker.action = Marker.ADD
        marker.id = id
        marker.type = marker.CYLINDER
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.4
        marker.color.a = 1
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = nparray_obstacle_xy[idxs_by_cluster[idx][-1]][0]
        marker.pose.position.y = nparray_obstacle_xy[idxs_by_cluster[idx][-1]][1]
        marker.pose.position.z = 0

        marker_array.markers.append(marker)

    publisher_rviz_markers.publish(marker_array)  # Publishes detected cones to RVIZ

def remove_field_num(a, i):
    '''Removes field from numpy structured array'''
    names = list(a.dtype.names)
    new_names = names[:i] + names[i+1:]
    return a[new_names]

def preprocessing(msg):
    ''''''
    timestamp = msg.header.stamp

    starray_obstacle_xyz = rnp.point_cloud2.pointcloud2_to_array(msg)
    starray_obstacle_xy = remove_field_num(starray_obstacle_xyz, 2)
    nparray_obstacle_xy = starray_obstacle_xy.view(np.float32).reshape(starray_obstacle_xy.shape + (-1,))
    return timestamp, nparray_obstacle_xy, starray_obstacle_xyz

# TODO: Finish
def publish_to_perception(pose_array):
    publisher_perception.publish(pose_array)

#TODO: Remove increment_snapshot_counter call
def clustering(msg):

    increment_snapshot_counter()

    timestamp, nparray_obstacle_xy, starray_obstacle_xyz = preprocessing(msg)

    # Compute DBSCAN
    db = DBSCAN(eps=0.3, min_samples=5).fit(nparray_obstacle_xy)
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_

    # Number of clusters in labels, ignoring noise if present
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    #n_noise_ = list(labels).count(-1)

    idxs_by_cluster = {i:[] for i in range(n_clusters_)}

    for idx, l in enumerate(labels):
        if(l != -1):
            idxs_by_cluster[l].append(idx)

    clusters_keys = idxs_by_cluster.keys()
    cone_idx_pseudopositions = []

    # Pick the last index in the cluster --> Higher height point?
    for k in clusters_keys:
        cone_idx_pseudopositions.append(idxs_by_cluster[k][-1])

    # TODO: At starray_obstacle_xyz can be dropped to improve efficiency
    publish_markers_to_RVIZ(idxs_by_cluster, nparray_obstacle_xy, starray_obstacle_xyz)
    publish_to_perception(get_PoseArray(idxs_by_cluster, nparray_obstacle_xy, starray_obstacle_xyz, timestamp))

    # Number of points in cluster
    # n_points_cluster = len(idxs_by_cluster[n_clusters_ -1])
    

#TODO: Remove global variable before testing
global_snapshot_counter = 0

def increment_snapshot_counter():
    global global_snapshot_counter
    global_snapshot_counter = global_snapshot_counter + 1
    rospy.loginfo(global_snapshot_counter)

if __name__ == '__main__':
    rospy.init_node('clustering')

    subscriber_obstacle_cloud = rospy.Subscriber("/ground_segmentation/obstacle_cloud", PointCloud2, clustering)
    publisher_rviz_markers = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1)
    publisher_perception = rospy.Publisher("clustered_points", PoseArray, queue_size=1)

    rospy.spin()
