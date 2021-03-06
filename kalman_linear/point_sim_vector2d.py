#!/usr/bin/env python
from __future__ import division
"""
Simulates the movements of points in space
"""
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, TwistStamped, Twist, PoseWithCovariance, PoseWithCovarianceStamped, TwistWithCovariance
import random
import tf

ODOM_INDEX = 0
PUB_INDEX = 1
NOISE_STD_INDEX = 2

class PointSim:

    def __init__(self):
        rospy.loginfo("Point Sim and Controller Initialized")
        self.load_vehicle()
        self.update_period = 0.1
        self.pose_timer = rospy.Timer(rospy.Duration(self.update_period), self.update_poses)
        sensor_pub_period = 1 / rospy.get_param("kalman/measurements/update_rate")
        self.sensor_timer = rospy.Timer(rospy.Duration(sensor_pub_period), self.publish_sensors)
        self.seq = 0

    def load_vehicle(self):
        self.auvs = {} # each auv has an odom
        auv = "akon"
        start_odom = Odometry()
        start_odom.pose.pose = self.get_start_pose()
        start_odom.header.seq = 0
        start_odom.header.stamp = rospy.get_rostime()
        start_odom.header.frame_id = 'world'
        start_odom.child_frame_id = auv + '/base_link'
        pub = rospy.Publisher(auv + '/pose_gt', Odometry, queue_size=10)
        self.auvs[auv] = [start_odom, pub]

        t = Twist() # initial velocity
        t.linear.x = rospy.get_param("kalman/odom/twist/x")
        t.linear.y = rospy.get_param("kalman/odom/twist/y")
        t.linear.z = rospy.get_param("kalman/odom/twist/z")
        self.auvs[auv][ODOM_INDEX].twist.twist = t
        self.odom_sensor_pub = rospy.Publisher(auv + "/odom_measurement", Odometry, queue_size=10)

    def get_start_pose(self):
        pose = Pose()
        pose.position.x = rospy.get_param("kalman/odom/initial_position/x")
        pose.position.y = rospy.get_param("kalman/odom/initial_position/y")
        pose.position.z = rospy.get_param("kalman/odom/initial_position/z")
        yaw = 0
        roll, pitch = 0,0
        quat_list = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = quat_list[0]
        pose.orientation.y = quat_list[1]
        pose.orientation.z = quat_list[2]
        pose.orientation.w = quat_list[3]
        return pose

    def update_poses(self, msg):
        auv = "akon"
        odom = self.auvs[auv][ODOM_INDEX]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(( odom.pose.pose.orientation.x, \
                                                                        odom.pose.pose.orientation.y, \
                                                                        odom.pose.pose.orientation.z, \
                                                                        odom.pose.pose.orientation.w))
                                    
        roll += odom.twist.twist.angular.x * self.update_period
        pitch += odom.twist.twist.angular.y * self.update_period
        yaw += odom.twist.twist.angular.z * self.update_period
        roll = self.correct_angles(roll)
        pitch = self.correct_angles(pitch)
        yaw = self.correct_angles(yaw)

        quat_list = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        odom.pose.pose.orientation.x = quat_list[0]
        odom.pose.pose.orientation.y = quat_list[1]
        odom.pose.pose.orientation.z = quat_list[2]
        odom.pose.pose.orientation.w = quat_list[3]
        
        odom.pose.pose.position.x += odom.twist.twist.linear.x * self.update_period
        odom.pose.pose.position.y += odom.twist.twist.linear.y * self.update_period
        odom.pose.pose.position.z += odom.twist.twist.linear.z * self.update_period

        odom.header.stamp = rospy.get_rostime()

        self.auvs[auv][ODOM_INDEX] = odom
        self.auvs[auv][PUB_INDEX].publish(odom)

    def publish_sensors(self, msg):
        """ publishes noisy position values for each robot """
        pwc = PoseWithCovariance()
        twc = TwistWithCovariance()
        auv = "akon"

        # X Meas
        rospy.loginfo_once("Normal Error on X")
        sigma = rospy.get_param("kalman/measurements/x_sigma")
        dot_sigma = rospy.get_param("kalman/measurements/x_dot_sigma")
        pwc.pose.position.x = np.random.normal( self.auvs[auv][ODOM_INDEX].pose.pose.position.x, sigma)
        pwc.covariance[0] = sigma ** 2
        twc.twist.linear.x = np.random.normal( self.auvs[auv][ODOM_INDEX].twist.twist.linear.x, dot_sigma)
        twc.covariance[0] = dot_sigma ** 2
        
        # Y Meas
        rospy.loginfo_once("Normal Error on Y")
        sigma = rospy.get_param("kalman/measurements/y_sigma")
        dot_sigma = rospy.get_param("kalman/measurements/y_dot_sigma")
        pwc.pose.position.y = np.random.normal( self.auvs[auv][ODOM_INDEX].pose.pose.position.y, sigma)
        pwc.covariance[7] = sigma ** 2
        twc.twist.linear.y = np.random.normal( self.auvs[auv][ODOM_INDEX].twist.twist.linear.y, dot_sigma)
        twc.covariance[7] = dot_sigma ** 2

        # Z Meas
        rospy.loginfo_once("Normal Error on Z")
        sigma = rospy.get_param("kalman/measurements/z_sigma")
        dot_sigma = rospy.get_param("kalman/measurements/z_dot_sigma")
        pwc.pose.position.z = np.random.normal( self.auvs[auv][ODOM_INDEX].pose.pose.position.z, sigma)
        pwc.covariance[14] = sigma ** 2
        twc.twist.linear.z = np.random.normal( self.auvs[auv][ODOM_INDEX].twist.twist.linear.z, dot_sigma)
        twc.covariance[14] = dot_sigma ** 2

        pwc.pose.orientation = self.auvs[auv][ODOM_INDEX].pose.pose.orientation

        odom_msg = Odometry()
        odom_msg.header.seq = self.seq
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "base_link"
        odom_msg.child_frame_id = odom_msg.header.frame_id
        odom_msg.pose = pwc
        odom_msg.twist = twc
        self.odom_sensor_pub.publish(odom_msg)

        self.seq += 1

    def correct_angles(self, angle):
        """ Map all angles between -pi to pi """
        while angle < -np.pi or angle > np.pi:
            if angle < -np.pi:
                angle += 2 * np.pi
            else:
                angle -= 2 * np.pi
        return angle

def main():
    rospy.init_node('point_sim_contoller')
    ps = PointSim()
    rospy.spin()

if __name__ == "__main__":
    main()
