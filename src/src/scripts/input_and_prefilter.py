#!/usr/bin/env python

import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2

def prefilter(msg):
    rospy.loginfo("Message received")
    #xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
    record_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    scaled_array = record_array.copy()
    scaled_array = scaled_array[scaled_array['x'] < 8]
    scaled_array = scaled_array[scaled_array['x'] > 0]
    scaled_array = scaled_array[scaled_array['y'] < 4]
    scaled_array = scaled_array[scaled_array['y'] > -4]
    scaled_array = scaled_array[scaled_array['z'] < 0.5]
    pc2_to_record_to_pc2 = ros_numpy.point_cloud2.array_to_pointcloud2(scaled_array, frame_id='velodyne')
    bag_publisher.publish(pc2_to_record_to_pc2)
    #rospy.loginfo(msg)



if __name__ == '__main__':
    rospy.init_node('prefiltering')

    bag_subscriber = rospy.Subscriber("/velodyne_points", PointCloud2, prefilter)
    bag_publisher = rospy.Publisher("/prefiltered_points", PointCloud2, queue_size=1)

    rate = rospy.Rate(10)
    rospy.spin()



