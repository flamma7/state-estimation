"""
Attempts to track the turtle

Publishes estimated odom on /estimate
"""
import rospy
from turtlesim.msg import Pose

# steps
# subscribe to the pose information
# attempt to start tracking


class Kalman():
    def __init__(self):
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)

    def pose_callback(self, msg):
        print(msg)

if __name__ == "__main__":
    rospy.init_node("kalman")
    k = Kalman()
    rospy.spin()