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

def preprocessing(msg):
    raw_points = rnp.point_cloud2.pointcloud2_to_array(msg)
    new_points = remove_field_num(raw_points, 2)
    array = new_points.view(np.float32).reshape(new_points.shape + (-1,))
    return array


def clustering(msg):
    raw_points = preprocessing(msg)
    #Compute DBSCAN
    db = DBSCAN(eps=0.4, min_samples=3).fit(raw_points)
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_

    # Number of clusters in labels, ignoring noise if present
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)

    idxs_by_cluster = {i:[] for i in range(n_clusters_)}
    print(idxs_by_cluster)

    for idx, l in enumerate(labels):
        if(l != -1):
            idxs_by_cluster[l].append(idx)

    clusters_keys = idxs_by_cluster.keys()

    cone_idx_pseudopositions = []
    # Pick the last index in the cluster --> Higher height point?
    for k in clusters_keys:
        cone_idx_pseudopositions.append(idxs_by_cluster[k][-1])

    print(idxs_by_cluster)
    print(cone_idx_pseudopositions)

    # Number of points in cluster
    #print(len(idxs_by_cluster[n_clusters_ -1]))

    #print(idxs_by_cluster)

if __name__ == '__main__':
    rospy.init_node('clustering')
    bag_subscriber = rospy.Subscriber("/ground_segmentation/obstacle_cloud", PointCloud2, clustering)
#   bag_publisher = rospy.Publisher("clustered_points", PointCloud2, queue_size=1)

    rate = rospy.Rate(10)
    rospy.spin()