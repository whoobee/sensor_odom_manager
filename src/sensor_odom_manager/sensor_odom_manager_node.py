#!/usr/bin/env python
from __future__ import print_function
import time
# ros std imports
import rospy
# tf imports
import tf
import tf_conversions
import tf2_ros
from tf.transformations import *
# msg imports
import std_msgs.msg
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
# param parser imports
import json
# generic math imports
import math
import numpy as np
from dual_quaternions import DualQuaternion
from dual_quaternions_ros import *

# helper class for ROS lovver
class ROSLogger(object):
    #Imitate a standard Python logger, but pass the messages to rospy logging.
    def debug(self, msg):    rospy.logdebug(msg)  #  print(msg) #
    def info(self, msg):     rospy.loginfo(msg)   #  print(msg) #
    def warn(self, msg):     rospy.logwarn(msg)   #  print(msg) #
    def error(self, msg):    rospy.logerr(msg)    #  print(msg) #
    def critical(self, msg): rospy.logfatal(msg)  #  print(msg) #

# helper function to read parameters from launch files
def get_param(name, default):
    val = rospy.get_param(name, default)
    rospy.loginfo('  %s: %s', name, str(val))
    return val

# Odom conversion class
class SensorOdomManager(object):  
    # ros node frequency
    main_rate = 10
    # input message definition for sensor
    odom_in_msg = Odometry()
    # output message definition
    odom_msg = Odometry()
    # tf message definition
    tf_msg = TransformStamped()

    # initialization function
    def __init__(self):
        # param fetch
        self.odom_in_topic   = get_param('~odom_in_topic', "/in_odom")
        self.odom_out_topic  = get_param('~odom_out_topic', "/odom")
        self.odom_out_frame  = get_param('~odom_out_frame', "odom")
        self.base_frame      = get_param('~base_frame', "base_link")
        self.odom_calc_hz    = get_param('~odom_calc_hz', 10)
        self.sensor_offset   = json.loads(get_param('~sensor_offset', '{"x": 0., "y": 0., "z": 0.}'))
        self.sensor_rotation = json.loads(get_param('~sensor_rotation', '{"x": 0., "y": 0., "z": 0.}'))
        self.ignore_x        = get_param('~ignore_x', 'False')
        self.ignore_y        = get_param('~ignore_y', 'False')
        self.ignore_z        = get_param('~ignore_z', 'False')
        # shutdown handling  
        rospy.on_shutdown(self.terminate)
        # ros publishers
        self.odom_publisher  = rospy.Publisher(self.odom_out_topic, Odometry, tcp_nodelay=True, queue_size=2)
        self.odom_subscriber = rospy.Subscriber(self.odom_in_topic, Odometry, self.sensor_odom_callback)
        # setup qaternion
        self.qtAdjustedRot = np.array([0,0,0,1])
        # setup input message from sensor
        self.odom_msg.header.frame_id = self.odom_out_frame
        self.odom_msg.child_frame_id = self.base_frame
        self.odom_msg.pose.pose.position.x = 0.0
        self.odom_msg.pose.pose.position.y = 0.0
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 1.0
        self.odom_msg.twist.twist.linear.x = 0.0
        self.odom_msg.twist.twist.linear.y = 0.0
        self.odom_msg.twist.twist.linear.z = 0.0
        self.odom_msg.twist.twist.angular.x = 0.0
        self.odom_msg.twist.twist.angular.y = 0.0
        self.odom_msg.twist.twist.angular.z = 0.0
        # setup transform
        self.tf_publisher = tf2_ros.TransformBroadcaster()
        self.tf_msg.header.frame_id = self.odom_out_frame
        self.tf_msg.child_frame_id  = self.base_frame
        self.tf_msg.transform.translation.x = 0.0
        self.tf_msg.transform.translation.y = 0.0
        self.tf_msg.transform.translation.z = 0.0
        self.tf_msg.transform.rotation.x = 0.0
        self.tf_msg.transform.rotation.y = 0.0
        self.tf_msg.transform.rotation.w = 0.0
        self.tf_msg.transform.rotation.z = 1.0
        # bool for flagging the first msg received from sensor
        self.first_frame_received = False

    # calback function of the sensor input topic
    def sensor_odom_callback(self, data):
        # mark the flag that the sensor message was received
        self.first_frame_received = True
        # get data from the sensor msg
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

    # odometry publisher function
    def pub_odometry(self, time_now):
        # in case we received at least one frame from the sensor
        if(self.first_frame_received == True):
            # update timestapm
            now = time_now
            self.odom_msg.header.stamp = now
            self.tf_msg.header.stamp = now
            # update frame ids
            self.odom_msg.header.frame_id = self.odom_out_frame
            self.odom_msg.child_frame_id = self.base_frame
            # Twist/velocity:
            self.odom_msg.twist.twist.linear.x = self.odom_in_msg.twist.twist.linear.x
            self.odom_msg.twist.twist.angular.z = self.odom_in_msg.twist.twist.linear.z
            # get sensor pose quaternion
            self.qtAdjustedRot = np.array([self.odom_in_msg.pose.pose.orientation.x,
                                        self.odom_in_msg.pose.pose.orientation.y,
                                        self.odom_in_msg.pose.pose.orientation.z,
                                        self.odom_in_msg.pose.pose.orientation.w])
            # in case there is no configured rotation to the sensor, ignore the quaternion calc
            if(self.sensor_rotation["x"] != 0. or self.sensor_rotation["y"] != 0. or self.sensor_rotation["z"] != 0.):
                # calc quaternion rotation tool (angles are negative because we need to adjust the sensor from the config data )
                qtSensorAdjustmentTool = tf.transformations.quaternion_from_euler(  np.radians(-self.sensor_rotation["x"]), 
                                                                                np.radians(-self.sensor_rotation["y"]), 
                                                                                np.radians(-self.sensor_rotation["z"]))
                # rotate sensor tf
                qtRot = tf.transformations.quaternion_multiply(qtSensorAdjustmentTool, self.qtAdjustedRot)
                self.qtAdjustedRot = tf.transformations.quaternion_multiply(qtRot, tf.transformations.quaternion_inverse(qtSensorAdjustmentTool))
            # get the euler angles from pose quaternion
            self.rpy = tf.transformations.euler_from_quaternion(self.qtAdjustedRot)
            # calculate translations
            # check if we need to calculate or ignore x coordonate
            if (self.ignore_x == False):
                # calculate x coordonate
                self.transl_x = self.odom_in_msg.pose.pose.position.x - ((self.sensor_offset["x"] * math.cos(self.rpy[2])) - (-self.sensor_offset["y"] * math.sin(self.rpy[2])))
            else:
                # ignore x coordonate of the sensor, therefore set it to 0
                self.transl_x = 0
            # check if we need to calculate or ignore y coordonate
            if (self.ignore_y == False):
                # calculate y coordonate
                self.transl_y = self.odom_in_msg.pose.pose.position.y - ((self.sensor_offset["x"] * math.sin(self.rpy[2])) + (-self.sensor_offset["y"] * math.cos(self.rpy[2])))
            else:
                # ignore y coordonate of the sensor, therefore set it to 0
                self.transl_y = 0
            # check if we need to calculate or ignore z coordonate
            if (self.ignore_z == False):
                # calculate z coordonate
                self.transl_z = self.odom_in_msg.pose.pose.position.z - self.sensor_offset["z"]
            else:
                # ignore z coordonate of the sensor, therefore set it to 0
                self.transl_z = 0
            # update odom message
            self.odom_msg.pose.pose.position.x = self.transl_x
            self.odom_msg.pose.pose.position.y = self.transl_y
            self.odom_msg.pose.pose.position.z = self.transl_z
            self.odom_msg.pose.pose.orientation.x = self.qtAdjustedRot[0]
            self.odom_msg.pose.pose.orientation.y = self.qtAdjustedRot[1]
            self.odom_msg.pose.pose.orientation.z = self.qtAdjustedRot[2]
            self.odom_msg.pose.pose.orientation.w = self.qtAdjustedRot[3]
            # update tf
            self.tf_msg.transform.translation.x = self.transl_x
            self.tf_msg.transform.translation.y = self.transl_y
            self.tf_msg.transform.translation.z = self.transl_z
            self.tf_msg.transform.rotation.x = self.qtAdjustedRot[0]
            self.tf_msg.transform.rotation.y = self.qtAdjustedRot[1]
            self.tf_msg.transform.rotation.z = self.qtAdjustedRot[2]
            self.tf_msg.transform.rotation.w = self.qtAdjustedRot[3]
            # ... and publish!
            self.tf_publisher.sendTransform(self.tf_msg)
            self.odom_publisher.publish(self.odom_msg)

    # shutdown hook handler
    def terminate(self):
        # shutdown ros timer
        self.fast_timer.shutdown()

    # scheduling timer handler
    def fast_timer(self, timer_event):
        time_now = rospy.Time.now()
        self.pub_odometry(time_now)

    # main handler
    def main_loop(self):
        # Main control, handle startup and error handling
        # while a ROS timer will handle the high-rate (~50Hz) comms + odometry calcs
        self.main_rate = rospy.Rate(10) # hz
        # Start timer to run high-rate comms
        self.fast_timer = rospy.Timer(rospy.Duration(1/float(self.odom_calc_hz)), self.fast_timer)

# sensor manager start function
def start_manager():
    # init the ros node
    rospy.init_node('sensor_odom_mngr')
    # init the SensorOdomManager class
    sensor_manager = SensorOdomManager()
    # cyclic handling
    sensor_manager.main_loop()
    # execution rate calculation
    while not rospy.is_shutdown():
        sensor_manager.main_rate.sleep()
    
# entry point
if __name__ == '__main__':
    try:
        start_manager()
    except rospy.ROSInterruptException:
        pass
