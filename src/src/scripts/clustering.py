#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import ros_numpy as rnp

import numpy as np

from sklearn.cluster import DBSCAN
from sklearn import metrics
from sklearn.datasets import make_blobs
from sklearn.preprocessing import StandardScaler

import matplotlib.pyplot as plt

def remove_field_num(a, i):
    names = list(a.dtype.names)
    new_names = names[:i] + names[i+1:]
    b = a[new_names]
    return b

def callback_view_message(msg):
    raw_points = rnp.point_cloud2.pointcloud2_to_array(msg)
    new_points = remove_field_num(raw_points, 2)
    array = new_points.view(np.float32).reshape(new_points.shape + (-1,))
    rospy.loginfo(array.shape)
    rospy.loginfo(new_points.dtype.names)
    return array


def callback_clustering(msg):
    rospy.loginfo("Message received")
    raw_points = callback_view_message(msg)
    #Compute DBSCAN
    db = DBSCAN(eps=0.4, min_samples=3).fit(raw_points)
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_

    # Number of clusters in labels, ignoring noise if present
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)

    print('Estimated number of clusters: %d' % n_clusters_)
    print('Estimated number of noise points: %d' % n_noise_)

if __name__ == '__main__':
    rospy.init_node('clustering')
    bag_subscriber = rospy.Subscriber("/ground_segmentation/obstacle_cloud", PointCloud2, callback_clustering)
#   bag_publisher = rospy.Publisher("clustered_points", PointCloud2, queue_size=1)

    rate = rospy.Rate(10)
    rospy.spin()