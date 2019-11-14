"""
Attempts to track the turtle

Publishes estimated odom on /estimate
"""
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Float32MultiArray

# steps
# subscribe to the pose information
# attempt to start tracking
# Just uses GPS


class Kalman():
    def __init__(self):
        rospy.Subscriber("/turtle1/gps", Float32MultiArray, self.gps_callback)
        # rospy.Subscriber("/turtle1/cmd_vel", Twist , self.cmd_vel_callback)
        self.pub = rospy.Publisher("/turtle1/kf_estimate", Pose, queue_size=10) # TODO maybe publish a covariance as well?

        # self.control_inpt = None
        self.estimate = None # 4 x 1 column vector
        self.uncertainty = None
        self.last_update_time = None
        self.motion_noise = np.array([[0.2,0,0,0], [0,0.2,0,0],[0,0,0.1,0],[0,0,0,0.1]])
        self.meas_noise = np.array([[5,0],[0,5]])

    def gps_callback(self, msg):

        if self.last_update_time == None:
            self.estimate = np.array([[msg.data[0], msg.data[1], 0, 0]]).T
            self.uncertainty = np.eye(4)
            self.last_update_time = rospy.get_time()
        else:
            delta_t = rospy.get_time() - self.last_update_time
            self.last_update_time = rospy.get_time()
            mu_old = self.estimate
            sigma_old = self.uncertainty
            z = np.array([[msg.data[0]], [msg.data[1]]])
        
            A = np.array([[1, 0, delta_t, 0], [0, 1, 0, delta_t], [0,0,1,0], [0,0,0,1]])
            C = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
            mu_bar = np.dot(A, mu_old)
            simga_bar = np.dot(np.dot(A, sigma_old), A.T) + self.motion_noise

            tmp = np.dot( np.dot(C, simga_bar), C.T ) 
            K = np.dot( np.dot(simga_bar,C.T), np.linalg.inv(tmp + self.meas_noise) )

            tmp = z - np.dot(C, mu_bar)
            self.estimate = mu_old + np.dot(K, tmp)
            self.uncertainty = np.dot( np.eye(4) - np.dot(K, C), simga_bar)

        p = Pose()
        p.x = self.estimate[0]
        p.y = self.estimate[1]
        self.pub.publish(p)
        print(self.uncertainty)

    # def cmd_vel_callback(self, msg):
        # self.control_inpt = msg

if __name__ == "__main__":
    rospy.init_node("kalman")
    k = Kalman()
    rospy.spin()