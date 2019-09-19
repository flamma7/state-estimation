#!/usr/bin/env python
from __future__ import division
import rospy
from geometry_msgs.msg import PoseWithCovariance
import numpy as np

"""
This program should estimate the value of a constant scalar, printing out its mean covariance with each measurement
"""

class Kalman:
    def __init__(self):
        rospy.Subscriber("akon/pose_measurement", PoseWithCovariance, self.meas_callback)
        self.P_posteriori = 1e9 # high uncertainty
        self.x_posteriori = 0 # initial value

    def meas_callback(self, msg):
        y = msg.pose.position.x # noisy measurement
        y_var = msg.covariance[0]
        # print("Meas: " + str(y) + " | Var: " + str(y_var))

        # Run the KF(
        # Time update, nothing should change
        P_apriori = self.P_posteriori # No system noise
        K = P_apriori * (1 / (P_apriori + y_var) )
        x_apriori = self.x_posteriori + 0 # No control input or system noise

        # print("P-: " + str(P_apriori) + " | x-: " + str(x_apriori) + " | K: " + str(K))

        # Meas Update
        innovation = y - x_apriori
        self.x_posteriori = x_apriori + K * innovation
        self.P_posteriori = (1 - K)*P_apriori*(1-K) + K*y_var*K

        print("P+: " + str(self.P_posteriori) + " | x+: " + str(self.x_posteriori))

        

if __name__ == "__main__":
    rospy.init_node("kalman_filter")
    k = Kalman()
    rospy.spin()