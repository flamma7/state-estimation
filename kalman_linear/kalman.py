#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovariance

class Kalman:
    def __init__(self):
        rospy.Subscriber("akon/pose_measurement", PoseWithCovariance, self.meas_callback)

    def meas_callback(self, msg):
        print(msg.pose.position.x)

if __name__ == "__main__":
    rospy.init_node("kalman_filter")
    k = Kalman()
    rospy.spin()