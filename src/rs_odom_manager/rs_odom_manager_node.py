#!/usr/bin/env python
from __future__ import print_function

#import roslib; roslib.load_manifest('BINCADDY')
import rospy
#import roslib
import tf
import tf_conversions
import tf2_ros
from tf.transformations import *

import std_msgs.msg
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import std_srvs.srv

#roslib.load_manifest('diagnostic_updater')
import diagnostic_updater, diagnostic_msgs.msg

import time
import math
import traceback
import Queue

import json 
import numpy as np

class ROSLogger(object):
    """Imitate a standard Python logger, but pass the messages to rospy logging.
    """
    def debug(self, msg):    rospy.logdebug(msg)  #  print(msg) #
    def info(self, msg):     rospy.loginfo(msg)   #  print(msg) #
    def warn(self, msg):     rospy.logwarn(msg)   #  print(msg) #
    def error(self, msg):    rospy.logerr(msg)    #  print(msg) #
    def critical(self, msg): rospy.logfatal(msg)  #  print(msg) #
# 1 m/s = 3.6 km/hr

def get_param(name, default):
    val = rospy.get_param(name, default)
    rospy.loginfo('  %s: %s', name, str(val))
    return val

class RSOdomManager(object):  

    main_rate = 10
    odom_in_msg = Odometry()
    odom_msg = Odometry()
    tf_msg = TransformStamped()

    def __init__(self):
        self.odom_in_topic   = get_param('~odom_in_topic', "/in_odom")
        self.odom_out_topic  = get_param('~odom_out_topic', "/odom")
        self.odom_out_frame  = get_param('~odom_out_frame', "odom")
        self.base_frame      = get_param('~base_frame', "base_link")
        self.odom_calc_hz    = get_param('~odom_calc_hz', 10)
        self.rs_offset       = json.loads(get_param('~rs_offset', '{"x": 0., "y": 0., "z": 0.}'))

        rospy.on_shutdown(self.terminate)

        self.camera_offset = [self.rs_offset["x"], self.rs_offset["y"] , self.rs_offset["z"] ]   # Values in m
        
        self.odom_publisher  = rospy.Publisher(self.odom_out_topic, Odometry, tcp_nodelay=True, queue_size=2)
        self.odom_subscriber = rospy.Subscriber(self.odom_in_topic, Odometry, self.rs_odom_callback)

        # setup qaternion
        self.qtMapToOdom = np.array([0,0,0,1])
        self.qtCamToBase = np.array([1,0,0,0])

        # setup message
        self.odom_msg.header.frame_id = self.odom_out_frame
        self.odom_msg.child_frame_id = self.base_frame
        self.odom_msg.pose.pose.position.x = 0.0
        self.odom_msg.pose.pose.position.y = 0.0
        self.odom_msg.pose.pose.position.z = 0.0    # always on the ground, we hope
        self.odom_msg.pose.pose.orientation.x = 0.0 # always vertical
        self.odom_msg.pose.pose.orientation.y = 0.0 # always vertical
        self.odom_msg.pose.pose.orientation.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 1.0
        self.odom_msg.twist.twist.linear.x = 0.0
        self.odom_msg.twist.twist.linear.y = 0.0  # no sideways
        self.odom_msg.twist.twist.linear.z = 0.0  # or upwards... only forward
        self.odom_msg.twist.twist.angular.x = 0.0 # or roll
        self.odom_msg.twist.twist.angular.y = 0.0 # or pitch... only yaw
        self.odom_msg.twist.twist.angular.z = 0.0
        
        # setup transform
        self.tf_publisher = tf2_ros.TransformBroadcaster()
        #self.tf_msg = TransformStamped()
        self.tf_msg.header.frame_id = self.odom_out_frame
        self.tf_msg.child_frame_id  = self.base_frame
        self.tf_msg.transform.translation.x = 0.0
        self.tf_msg.transform.translation.y = 0.0
        self.tf_msg.transform.translation.z = 0.0
        self.tf_msg.transform.rotation.x = 0.0
        self.tf_msg.transform.rotation.y = 0.0
        self.tf_msg.transform.rotation.w = 0.0
        self.tf_msg.transform.rotation.z = 1.0

        self.first_frame_received = False

    def main_loop(self):
        # Main control, handle startup and error handling
        # while a ROS timer will handle the high-rate (~50Hz) comms + odometry calcs
        self.main_rate = rospy.Rate(10) # hz
        # Start timer to run high-rate comms
        self.fast_timer = rospy.Timer(rospy.Duration(1/float(self.odom_calc_hz)), self.fast_timer)

        
    def fast_timer(self, timer_event):
        time_now = rospy.Time.now()
        self.pub_odometry(time_now)

    
    def terminate(self):
        self.fast_timer.shutdown()


    def rs_odom_callback(self, data):
        self.first_frame_received = True
        #rospy.loginfo(rospy.get_caller_id() + "I got RS odom!")
        self.odom_in_msg.header.frame_id = data.header.frame_id
        self.odom_in_msg.child_frame_id = data.child_frame_id
        self.odom_in_msg.pose.pose.position.x = data.pose.pose.position.x
        self.odom_in_msg.pose.pose.position.y = data.pose.pose.position.y
        self.odom_in_msg.pose.pose.position.z = data.pose.pose.position.z
        self.odom_in_msg.pose.pose.orientation.x = data.pose.pose.orientation.x
        self.odom_in_msg.pose.pose.orientation.y = data.pose.pose.orientation.y
        self.odom_in_msg.pose.pose.orientation.z = data.pose.pose.orientation.z
        self.odom_in_msg.pose.pose.orientation.w = data.pose.pose.orientation.w
        self.odom_in_msg.twist.twist.linear.x = data.twist.twist.linear.x
        self.odom_in_msg.twist.twist.linear.y = data.twist.twist.linear.y
        self.odom_in_msg.twist.twist.linear.z = data.twist.twist.linear.z
        self.odom_in_msg.twist.twist.angular.x = data.twist.twist.angular.x
        self.odom_in_msg.twist.twist.angular.y = data.twist.twist.angular.y
        self.odom_in_msg.twist.twist.angular.z = data.twist.twist.angular.z


    def pub_odometry(self, time_now):

        if(self.first_frame_received == True):
            now = time_now
            self.odom_msg.header.stamp = now
            self.tf_msg.header.stamp = now


            self.odom_msg.header.frame_id = self.odom_out_frame
            self.odom_msg.child_frame_id = self.base_frame

            # Twist/velocity:
            self.odom_msg.twist.twist.linear.x = self.odom_in_msg.twist.twist.linear.x
            self.odom_msg.twist.twist.angular.z = self.odom_in_msg.twist.twist.linear.z

            self.qtOdomToPose = np.array([self.odom_in_msg.pose.pose.orientation.x,
                                        self.odom_in_msg.pose.pose.orientation.y,
                                        self.odom_in_msg.pose.pose.orientation.z,
                                        self.odom_in_msg.pose.pose.orientation.w])

            self.qtMapToRobot = tf.transformations.quaternion_multiply(self.qtMapToOdom, self.qtOdomToPose) #qtMapToOdom * qtOdomToPose * qtCamToBase
            #self.qtMapToRobot = tf.transformations.quaternion_multiply(self.qtMapToRobot, self.qtCamToBase)

            self.rpy = tf.transformations.euler_from_quaternion(self.qtOdomToPose)

            self.transl_x = self.odom_in_msg.pose.pose.position.x - ((self.rs_offset["x"] * math.cos(self.rpy[2])) - (-self.rs_offset["y"] * math.sin(self.rpy[2])))
            self.transl_y = self.odom_in_msg.pose.pose.position.y - ((self.rs_offset["x"] * math.sin(self.rpy[2])) + (-self.rs_offset["y"] * math.cos(self.rpy[2])))

            self.pose_x = self.transl_x #+ self.rs_offset["x"]
            self.pose_y = self.transl_y #+ self.rs_offset["y"]

            self.odom_msg.pose.pose.position.x = self.pose_x
            self.odom_msg.pose.pose.position.y = self.pose_y
            self.odom_msg.pose.pose.orientation.x = self.qtMapToRobot[0]
            self.odom_msg.pose.pose.orientation.y = self.qtMapToRobot[1]
            self.odom_msg.pose.pose.orientation.z = self.qtMapToRobot[2]
            self.odom_msg.pose.pose.orientation.w = self.qtMapToRobot[3]
        
            self.tf_msg.transform.translation.x = self.transl_x
            self.tf_msg.transform.translation.y = self.transl_y
            self.tf_msg.transform.rotation.x = self.qtMapToRobot[0]
            self.tf_msg.transform.rotation.y = self.qtMapToRobot[1]
            self.tf_msg.transform.rotation.z = self.qtMapToRobot[2]
            self.tf_msg.transform.rotation.w = self.qtMapToRobot[3]

            # ... and publish!
            self.tf_publisher.sendTransform(self.tf_msg)
            self.odom_publisher.publish(self.odom_msg)

  
def start_manager():
    rospy.init_node('rs_odom_mngr')
    rs_manager = RSOdomManager()
    rs_manager.main_loop()
    while not rospy.is_shutdown():
        rs_manager.main_rate.sleep()
    

if __name__ == '__main__':
    try:
        start_manager()
    except rospy.ROSInterruptException:
        pass

