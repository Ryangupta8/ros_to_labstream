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

    def hsr_pose_callback(self, data):
        self.hsr_pose.position.x = data.pose.pose.position.x
        self.hsr_pose.position.y = data.pose.pose.position.y
        self.hsr_pose.orientation.x = data.pose.pose.orientation.x
        self.hsr_pose.orientation.y = data.pose.pose.orientation.y
        self.hsr_pose.orientation.z = data.pose.pose.orientation.z
        self.hsr_pose.orientation.w = data.pose.pose.orientation.w
        print("hsr callkback")

    def spot_pose_callback(self, data):
        self.spot_pose.position.x = data.pose.pose.position.x
        self.spot_pose.position.y = data.pose.pose.position.y
        self.spot_pose.orientation.x = data.pose.pose.orientation.x
        self.spot_pose.orientation.y = data.pose.pose.orientation.y
        self.spot_pose.orientation.z = data.pose.pose.orientation.z
        self.spot_pose.orientation.w = data.pose.pose.orientation.w
        print("spot callkback")


def main():
    listener = Listener()

    srate = 10 # Hz
    name1 = 'spot_state'
    name2 = 'hsr_state'
    type = 'xy_rxryrzrw'
    n_channels = 6
    
    info1 = StreamInfo(name1, type, n_channels, srate, 'float32', 'spot_state')
    info2 = StreamInfo(name2, type, n_channels, srate, 'float32', 'hsr_state')

    outlet1 = StreamOutlet(info1)
    outlet2 = StreamOutlet(info2)

    rospy.init_node('ros_to_labstream')
    r = rospy.Rate(10) # 10hz

    rospy.Subscriber("/hsr/global_pose", PoseWithCovarianceStamped, listener.hsr_pose_callback)
    rospy.Subscriber("/a1_950/global_pose", PoseWithCovarianceStamped, listener.spot_pose_callback)
    curr_time = time.clock()
    prev_time = time.clock()

    while not rospy.is_shutdown():

        curr_time = time.clock()

        # mysample1 = [rand() for _ in range(n_channels)]
        mysample1 = [listener.spot_pose.position.x, listener.spot_pose.position.y,
                listener.spot_pose.orientation.x, listener.spot_pose.orientation.y,
                listener.spot_pose.orientation.z, listener.spot_pose.orientation.w]
        # print("mysample = ", mysample1)
        mysample2 = [rand() for _ in range(n_channels)]
        # mysample2 = [listener.hsr_pose.position.x, listener.hsr_pose.position.y,
        #         listener.hsr_pose.orientation.x, listener.hsr_pose.orientation.y,
        #         listener.hsr_pose.orientation.z, listener.hsr_pose.orientation.w]

        if curr_time - prev_time >= 0.08:
            print("output")
            outlet1.push_sample(mysample1)
            outlet2.push_sample(mysample2)
            prev_time = curr_time


if  __name__ == '__main__':
    main()
