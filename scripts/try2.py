#! /usr/bin/env python

# Generic Packages
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped, Point

# Packages for LSL
import sys
import getopt
import time
from random import random as rand
from pylsl import StreamInfo, StreamOutlet, local_clock


class Listener:
    def __init__(self):
        self.spot_pose = Pose()
        self.hsr_pose = Pose()
        srate = 10 # Hz
        name1 = 'spot_state'
        name2 = 'hsr_state'
        type = 'xy_rxryrzrw'
        n_channels = 6
    
        self.info1 = StreamInfo(name1, type, n_channels, srate, 'float32', 'spot_state')
        self.info2 = StreamInfo(name2, type, n_channels, srate, 'float32', 'hsr_state')

        self.outlet1 = StreamOutlet(self.info1)
        self.outlet2 = StreamOutlet(self.info2)

    def hsr_pose_callback(self, data):
        self.hsr_pose.position.x = data.pose.pose.position.x
        self.hsr_pose.position.y = data.pose.pose.position.y
        self.hsr_pose.orientation.x = data.pose.pose.orientation.x
        self.hsr_pose.orientation.y = data.pose.pose.orientation.y
        self.hsr_pose.orientation.z = data.pose.pose.orientation.z
        self.hsr_pose.orientation.w = data.pose.pose.orientation.w
        print("hsr callkback")
        mysample1 = [rand() for _ in range(6)]
        self.outlet1.push_sample(mysample1)

    def spot_pose_callback(self, data):
        self.spot_pose.position.x = data.pose.pose.position.x
        self.spot_pose.position.y = data.pose.pose.position.y
        self.spot_pose.orientation.x = data.pose.pose.orientation.x
        self.spot_pose.orientation.y = data.pose.pose.orientation.y
        self.spot_pose.orientation.z = data.pose.pose.orientation.z
        self.spot_pose.orientation.w = data.pose.pose.orientation.w
        print("spot callkback")
        mysample2 = [rand() for _ in range(6)]
        self.outlet2.push_sample(mysample2)


def main():
    listener = Listener()

    rospy.init_node('ros_to_labstream')
    r = rospy.Rate(10) # 10hz

    rospy.Subscriber("/hsr/global_pose", PoseWithCovarianceStamped, listener.hsr_pose_callback)
    rospy.Subscriber("/a1_950/global_pose", PoseWithCovarianceStamped, listener.spot_pose_callback)

    while not rospy.is_shutdown():
        r.sleep()

if  __name__ == '__main__':
    main()
