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


    def go1_pose_callback(self, data):
        self.go1_pose.position.x = data.pose.position.x
        self.go1_pose.position.y = data.pose.position.y
        self.go1_pose.orientation.x = data.pose.orientation.x
        self.go1_pose.orientation.y = data.pose.orientation.y
        self.go1_pose.orientation.z = data.pose.orientation.z
        self.go1_pose.orientation.w = data.pose.orientation.w
        print("go1 callback")

    def spot_pose_callback(self, data):
        self.spot_pose.position.x = data.pose.position.x
        self.spot_pose.position.y = data.pose.position.y
        self.spot_pose.orientation.x = data.pose.orientation.x
        self.spot_pose.orientation.y = data.pose.orientation.y
        self.spot_pose.orientation.z = data.pose.orientation.z
        self.spot_pose.orientation.w = data.pose.orientation.w
        print("spot callback")


def main():
    listener = Listener()

    srate = 10 # Hz
    name1 = 'spot_state'

    name2 = 'go1_state'
    type = 'xy_rxryrzrw'
    n_channels = 6
    
    info1 = StreamInfo(name1, type, n_channels, srate, 'float32', 'spot_state')
    info2 = StreamInfo(name2, type, n_channels, srate, 'float32', 'go1_state')

    outlet1 = StreamOutlet(info1)
    outlet2 = StreamOutlet(info2)

    rospy.init_node('ros_to_labstream')
    r = rospy.Rate(10) # 10hz


    rospy.Subscriber("/go1/localization_ros", PoseStamped, listener.go1_pose_callback)
    rospy.Subscriber("/gcr_spot/localization_ros", PoseStamped, listener.spot_pose_callback)

    curr_time = time.clock()
    prev_time = time.clock()

    while not rospy.is_shutdown():

        curr_time = time.clock()

        # mysample1 = [rand() for _ in range(n_channels)]
        mysample1 = [listener.spot_pose.position.x, listener.spot_pose.position.y,
                listener.spot_pose.orientation.x, listener.spot_pose.orientation.y,
                listener.spot_pose.orientation.z, listener.spot_pose.orientation.w]
        # print("mysample = ", mysample1)
        # mysample2 = [rand() for _ in range(n_channels)]

        mysample2 = [listener.go1_pose.position.x, listener.go1_pose.position.y,
                listener.go1_pose.orientation.x, listener.go1_pose.orientation.y,
                listener.go1_pose.orientation.z, listener.go1_pose.orientation.w]

        if curr_time - prev_time >= 0.08:
            rospy.loginfo("output")
            outlet1.push_sample(mysample1)
            outlet2.push_sample(mysample2)
            prev_time = curr_time


if  __name__ == '__main__':
    main()
