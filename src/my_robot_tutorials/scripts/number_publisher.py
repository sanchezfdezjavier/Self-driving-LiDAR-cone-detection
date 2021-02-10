#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64

if __name__ == '__main__':
    rospy.init_node('number_publisher')

    publisher = rospy.Publisher("/number_publisher_topic", Int64, queue_size=10)
    rate = rospy.Rate(5)
    counter = 0
    msg = Int64()
    msg.data = counter

    while not rospy.is_shutdown():
        publisher.publish(msg)
        counter +=1
        rate.sleep()

    rospy.loginfo("Node stopped!")
    
        

