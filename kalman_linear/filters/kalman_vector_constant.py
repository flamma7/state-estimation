#!/usr/bin/env python
from __future__ import division
import rospy
from geometry_msgs.msg import PoseWithCovariance
import numpy as np

"""
This program should estimate the value of a constant vector, printing out its mean covariance with each measurement
"""

class Kalman:
    def __init__(self):
        rospy.Subscriber("akon/pose_measurement", PoseWithCovariance, self.meas_callback)
        self.P_posteriori = np.eye(3,3) * 1e9
        self.x_posteriori = np.zeros([3,1])
        self.meas_matrix = np.eye(3,3)

    def meas_callback(self, msg):
        y = np.array([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]])
        y_var = np.eye(3,3) * np.array([[msg.covariance[0]], [msg.covariance[7]], [msg.covariance[14]]]) # broadcast multiply
        # print("Meas: \n" + str(y) + "\n" + str(y_var))
        # print("\n---")

        # Run the KF
        # Time update, nothing should change
        P_apriori = self.P_posteriori # No system noise
        K = np.dot(P_apriori, self.meas_matrix.T)
        tmp = np.dot( np.dot(self.meas_matrix, P_apriori), self.meas_matrix.T) + y_var
        K = np.dot(K, np.linalg.inv(tmp))
        x_apriori = self.x_posteriori + 0 # No control input or system noise
        # print("P-: " + str(P_apriori) + " | x-: " + str(x_apriori) + " | K: " + str(K))

        # Meas Update
        innovation = y - np.dot(self.meas_matrix, x_apriori)
        self.x_posteriori = x_apriori + np.dot(K, innovation)
        tmp = np.eye(3,3) - np.dot(K, self.meas_matrix)
        self.P_posteriori = np.dot(tmp, P_apriori)
        # self.P_posteriori = (1 - K)*P_apriori*(1-K) + K*y_var*K

        print("P+: \n" + str(self.P_posteriori) + "\n" + str(self.x_posteriori))
        print("\n---")

        

if __name__ == "__main__":
    rospy.init_node("kalman_filter")
    k = Kalman()
    rospy.spin()