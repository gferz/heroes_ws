#!/usr/bin/env python3

import rospy
from omni4_odometry.rviz import OdomController


if __name__== '__main__':
    rospy.init_node('rviz')
    controller = OdomController()

    rospy.spin()