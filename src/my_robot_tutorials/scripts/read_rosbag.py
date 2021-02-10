#!/usr/bin/env python

import rosbag

bag = rosbag.Bag('test.bag')

for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
    print(msg)

bag.close()