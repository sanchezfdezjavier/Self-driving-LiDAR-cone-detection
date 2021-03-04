#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
import ros_numpy as rnp

import numpy as np

import uuid

from sklearn.cluster import DBSCAN
from sklearn import metrics
from sklearn.datasets import make_blobs
from sklearn.preprocessing import StandardScaler
import random


def publish_markers(cluster_idxs, raw_points):
    # marker storage for each instance
    marker_array = MarkerArray()
    
    print(len(cluster_idxs), "number of clusters")
    # marker creation for each cluster
    timestamp = rospy.Time.now()

    for id, idx in enumerate(cluster_idxs):
        marker = Marker()
        marker.header.frame_id = "/velodyne"
        marker.header.stamp = timestamp
        marker.ns = "cone_markers";
        marker.lifetime = rospy.Duration(0.1)
        marker.action = Marker.ADD
        marker.id = id
        marker.type = marker.CYLINDER
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 0.3
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = raw_points[idx][0]
        marker.pose.position.y = raw_points[idx][1]
        marker.pose.position.z = 0

        marker_array.markers.append(marker)

    print(len(marker_array.markers), "Number of markers created")
    # pubish the marker array to rviz
    publisher_rviz_markers.publish(marker_array)
    marker_array  
    

def remove_field_num(a, i):
    names = list(a.dtype.names)
    new_names = names[:i] + names[i+1:]
    b = a[new_names]

    return b

def preprocessing(msg):
    ndarray_points = rnp.point_cloud2.pointcloud2_to_array(msg)
    new_points = remove_field_num(ndarray_points, 2)
    # np structured array to nparray
    array = new_points.view(np.float32).reshape(new_points.shape + (-1,))
    # print(array[0])
    # print(array.shape)
    return array, ndarray_points


def clustering(msg):
    raw_points, ndarray_points = preprocessing(msg)

    #Compute DBSCAN
    db = DBSCAN(eps=0.4, min_samples=4).fit(raw_points)
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_

    # Number of clusters in labels, ignoring noise if present
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    print(n_clusters_)
    n_noise_ = list(labels).count(-1)

    idxs_by_cluster = {i:[] for i in range(n_clusters_)}

    for idx, l in enumerate(labels):
        if(l != -1):
            idxs_by_cluster[l].append(idx)

    clusters_keys = idxs_by_cluster.keys()
    cone_idx_pseudopositions = []
    # Pick the last index in the cluster --> Higher height point?
    for k in clusters_keys:
        cone_idx_pseudopositions.append(idxs_by_cluster[k][-1])

    publish_markers(clusters_keys, ndarray_points)

    # Number of points in cluster
    #print(len(idxs_by_cluster[n_clusters_ -1]))

    #print(idxs_by_cluster)

if __name__ == '__main__':
    rospy.init_node('clustering')
    bag_subscriber = rospy.Subscriber("/ground_segmentation/obstacle_cloud", PointCloud2, clustering)
    publisher_rviz_markers = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1)
#   bag_publisher = rospy.Publisher("clustered_points", PointCloud2, queue_size=1)

    rate = rospy.Rate(10)
    rospy.spin()