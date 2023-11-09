#!/usr/bin/env python3
import rospy
import roslib
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion, Point
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int16

#############################################################################
class DiffTf:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("diff_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        #### parameters #######
        self.rate = rospy.get_param('~rate',10.0)  # the rate at which to publish the transform
        self.ticks_meter = float(rospy.get_param('ticks_meter', 250))  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(rospy.get_param('~base_width', 1.3)) # The wheel base width in meters
        
        self.base_frame_id = rospy.get_param('~base_frame_id','base_link') # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame
        
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        self.x = 0                  # position in xy plane 
        self.y = 0
        self.theta = 0
        self.then = rospy.Time.now()
        
        rospy.Subscriber("x_pos", Int16, self.x_posCallback)
        rospy.Subscriber("y_pos", Int16, self.y_posCallback)
        rospy.Subscriber("theta_pos", Int16, self.y_posCallback)
        rospy.Subscriber("/vision/coordinate", Point, self.vision_callback)
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()
        
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
       
     
    #############################################################################
    def update(self):
            now = rospy.Time.now()
            # publish the odom information
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin( self.theta / 2 )
            quaternion.w = cos( self.theta / 2 )
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
                )
            
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = 0
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = 0
            self.odomPub.publish(odom)
            
            


    def x_posCallback(self, msg):
        rospy.loginfo(rospy.get_caller_id() + "x : ", msg.data)
        self.x = msg.data
    
    def y_posCallback(self, msg):
        rospy.loginfo(rospy.get_caller_id() + "y : ", msg.data)
        self.y = msg.data
        
    def theta_posCallback(self, msg):
        rospy.loginfo(rospy.get_caller_id() + "theta : ", msg.data)
        self.theta = msg.data

    def vision_callback(self, msg):
        x = msg.x
        y = msg.y

        pass
#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
