#!/usr/bin/env python
from __future__ import division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from threading import Lock # locks can only be unlocked by their current thread
"""
This program should estimate the value of a dynamic scalar, printing out its mean covariance with each measurement

What it's modeling:
Single scalar increments +1 unit per second


Steps
X 1) Edit point sim to publish measurements at 1/4 Hz
X 2) Edit this file to update the filter at an arbitrary frequency
X 3) Add dynamics to this file
See if our covariance increases ad infinitum without measurements
4) See how well the filter performs with measurements

The variable self.last_propagation_step is used for synchronization

Time updates happen at an inputted frequency
Meas updates happen on an adhoc basis and call a new time update step
A lock / binary semaphore is used to ensure that the posteriors values do not get overwritten in the middle of an update by either the time or measurement update
- (even though native python is not truly multithreaded, this is good practice)

NOTE: This filter is not totally synchronous, it runs the measurement update step at the timestamp from the msg, but runs the 
time update at the current step. (so our time update could happen at more recent time than our meas update). This was done for simplicity
"""

class Kalman:
    def __init__(self):
        self.system_noise = 0.25 # arbitrary, how much do we trust our dynamics?
        self.P_posteriori = 1 # low uncertainty
        # self.P_posteriori = 1e9 # high uncertainty
        self.x_posteriori = 0 # initial value
        self.lock = Lock()
        self.last_propagation_step = rospy.Time.now()
        self.time_update_callback(False) # initalize the filter
        propagation_frequency = 5 # Hz
        rospy.Timer(rospy.Duration(1 / propagation_frequency), self.time_update_callback)
        rospy.Subscriber("akon/pose_measurement", PoseWithCovarianceStamped, self.meas_callback)

    def time_update_callback(self, msg):
        self.lock.acquire(True) # block on acquiring the Lock
        self.time_update(False)
        self.last_propagation_step = rospy.Time.now()
        self.lock.release()

    def time_update(self, meas_update=False):
        """
        Computes the time update using the time since the last time update
        """
        time_now = rospy.Time.now()
        delta_t = time_now.to_sec() - self.last_propagation_step.to_sec()
        F = delta_t
        Q = self.system_noise

        # Time update
        x_apriori = self.x_posteriori + F
        P_apriori = (self.P_posteriori + F**2) + Q

        self.last_propagation_step = time_now # reset the last time

        if meas_update:
            return P_apriori, x_apriori
        else:
            self.x_posteriori = x_apriori
            self.P_posteriori = P_apriori
            print("P+: " + str(self.P_posteriori) + " | x+: " + str(self.x_posteriori))

    def meas_callback(self, msg):
        print("Measurement Update")
        t_meas = rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs)
        self.lock.acquire(True)
        P_apriori, x_apriori = self.time_update(True) # Perform time update to synchronize the measurement update
        # Perform Meas Update
        y = msg.pose.pose.position.x # noisy measurement
        y_var = msg.pose.covariance[0]
        R = y_var
        

        # Calculate K
        H = 1
        K = P_apriori * H * ( 1 / ( H*P_apriori*H + R) )

        innovation = y - H*x_apriori
        self.x_posteriori = x_apriori + K * innovation
        self.P_posteriori = (1 - K*H) * P_apriori
        print("P+: " + str(self.P_posteriori) + " | x+: " + str(self.x_posteriori))
        self.lock.release()
        


if __name__ == "__main__":
    rospy.init_node("kalman_filter")
    k = Kalman()
    rospy.spin()

"""
target_time = rospy.Time(data_matrix[0][row].header.stamp.secs, data_matrix[0][row].header.stamp.nsecs).to_sec()
                for col in range(1, association_matrix.shape[1]):
                    while True: # not pretty this alg
                        last_msg_index_for_col = int(previous_row[0, col])
                        last_time = rospy.Time(data_matrix[col][last_msg_index_for_col].header.stamp.secs, \
                                            data_matrix[col][last_msg_index_for_col].header.stamp.nsecs).to_sec()
                        next_time = rospy.Time(data_matrix[col][last_msg_index_for_col+1].header.stamp.secs, \
                                            data_matrix[col][last_msg_index_for_col+1].header.stamp.nsecs).to_sec()
                        last_diff = abs(target_time - last_time)
                        next_diff = abs(target_time - next_time)
                        if last_diff < next_diff:
                            association_matrix[row,col] = last_msg_index_for_col
                            break
                        else:
                            previous_row[0,col] = last_msg_index_for_col + 1
"""