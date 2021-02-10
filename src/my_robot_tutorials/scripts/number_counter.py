#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64

def callback_receive_counter(msg):
    self.counter += 1
    rospy.loginfo(counter)

if __name__ == '__main__':
    rospy.init_node('number_counter')

    counter = 0

    subscriber = rospy.Subscriber("/number_publisher_topic", Int64)
    publishser = rospy.Publisher("/number_count", Int64)

    rospy.spin()