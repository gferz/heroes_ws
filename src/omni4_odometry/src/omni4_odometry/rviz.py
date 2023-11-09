#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_conversions
import math


class OdomController(object):

    def __init__(self):
        rospy.loginfo("Program Start")
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.x_sub_ = rospy.Subscriber("/x_pos", Float64, self.xCallback)
        self.y_sub_ = rospy.Subscriber("/y_pos", Float64, self.yCallback)
        self.theta_sub_ = rospy.Subscriber("/theta_pos", Float64, self.thetaCallback)
        self.odom_pub_ = rospy.Publisher("/odom", Odometry, queue_size=10)
        
        # Fill the Odometry message with invariant parameters
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        # Fill the TF message
        self.br_ = TransformBroadcaster()
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"

        self.prev_time_ = rospy.Time.now()
        
        # Compose and publish the odom message
        

    def xCallback(self, msg):
        self.x_ = msg.data
        rospy.loginfo("x : %f", self.x_)
    
    def yCallback(self, msg):
        self.y_ = msg.data
        rospy.loginfo("y : %f", self.y_)

    def thetaCallback(self, msg):
        self.theta_ = msg.data
        rospy.loginfo("theta : %f", self.theta_)
        
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg_.header.stamp = rospy.Time.now()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.twist.twist.linear.x = 0
        self.odom_msg_.twist.twist.angular.z = 0
        self.odom_pub_.publish(self.odom_msg_)

        # TF
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = rospy.Time.now()
        self.br_.sendTransform(self.transform_stamped_)

if __name__== '__main__':
    rospy.init_node('rviz')
    controller = OdomController()

    rospy.spin()