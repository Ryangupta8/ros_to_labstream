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
        self.go1_pose = Pose()
        srate = 10 # Hz
        name1 = 'spot_state'
        name2 = 'go1_state'
        type = 'xy_rxryrzrw'
        n_channels = 6
    
        self.info1 = StreamInfo(name1, type, n_channels, srate, 'float32', 'spot_state')
        self.info2 = StreamInfo(name2, type, n_channels, srate, 'float32', 'go1_state')

        self.outlet1 = StreamOutlet(self.info1)
        self.outlet2 = StreamOutlet(self.info2)

    def go1_pose_callback(self, data):
        self.go1_pose.position.x = data.pose.position.x
        self.go1_pose.position.y = data.pose.position.y
        self.go1_pose.orientation.x = data.pose.orientation.x
        self.go1_pose.orientation.y = data.pose.orientation.y
        self.go1_pose.orientation.z = data.pose.orientation.z
        self.go1_pose.orientation.w = data.pose.orientation.w
        print("go1 callback")
        # mysample1 = [rand() for _ in range(6)]
        mysample1 = [self.go1_pose.position.x, self.go1_pose.position.y,
                self.go1_pose.orientation.x, self.go1_pose.orientation.y,
                self.go1_pose.orientation.z, self.go1_pose.orientation.w]
        self.outlet2.push_sample(mysample1)

    def spot_pose_callback(self, data):
        self.spot_pose.position.x = data.pose.position.x
        self.spot_pose.position.y = data.pose.position.y
        self.spot_pose.orientation.x = data.pose.orientation.x
        self.spot_pose.orientation.y = data.pose.orientation.y
        self.spot_pose.orientation.z = data.pose.orientation.z
        self.spot_pose.orientation.w = data.pose.orientation.w
        print("spot callback")
        # mysample2 = [rand() for _ in range(6)]
        mysample2 = [self.spot_pose.position.x, self.spot_pose.position.y,
                self.spot_pose.orientation.x, self.spot_pose.orientation.y,
                self.spot_pose.orientation.z, self.spot_pose.orientation.w]
        self.outlet1.push_sample(mysample2)


def main():
    listener = Listener()

    rospy.init_node('ros_to_labstream')
    r = rospy.Rate(10) # 10hz

    rospy.Subscriber("/go1/localization_ros", PoseStamped, listener.go1_pose_callback)
    rospy.Subscriber("/gcr_spot/localization_ros", PoseStamped, listener.spot_pose_callback)

    while not rospy.is_shutdown():
        r.sleep()

if  __name__ == '__main__':
    main()
