#!/usr/bin/python

from rosbag import Bag

with Bag('start_c_conv.bag', 'w') as Y:
    for topic, msg, t in Bag('start_c.bag'):
        Y.write('/commbot/cmd_vel' if topic == '/rosaria/cmd_vel' else topic, msg, t)
