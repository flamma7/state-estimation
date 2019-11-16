from __future__ import division
"""
Attempts to track {x,y,theta,xdot,ydot}

Publishes estimated odom on /ekf_estimate
"""
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Float32MultiArray
import threading

# steps
# test the locking stuff
# test subscribing and queuing control inputs + measurements
# implement the prediction stuff w/o measurements
# implement correction step

THETA_INDEX = 2
SPEED_INDEX = 3
THETA_DOT_INDEX = 4

class EKF():
    def __init__(self):
        # rospy.Subscriber("/turtle1/meas", Float32MultiArray, self.meas_callback)
        # rospy.Subscriber("/turtle1/cmd_vel", Twist, self.control_callback)
        # self.pub = rospy.Publisher("/turtle1/kf_estimate", Float32MultiArray, queue_size=10) # TODO maybe publish a covariance as well?

        # self.control_inpt = None
        # self.estimate = None
        # self.uncertainty = None
        # self.last_update_time = None
        # self.motion_noise = np.array([[0.2,0,0,0,0,0],\
        #                               [0,0.2,0,0,0,0],\
        #                               [0,0,0.1,0,0,0],\
        #                               [0,0,0,0.05,0,0],\
        #                               [0,0,0,0,0.05,0],\
        #                               [0,0,0,0,0,0.01]])
        # self.meas_noise = np.eye(5) * np.array([[1,1,0.1,0.1,0.01]]).T # broadcast
        # print(self.meas_noise)
        # self.last_control_time = rospy.get_time()
        self.control_queue = []
        self.meas_queue = []
        self.lock = threading.Lock()

    def control_callback(self, msg):
        self.lock.acquire()
        self.last_control_time = rospy.get_time()
        s = msg.linear.x
        theta_dot = msg.angular.z
        self.control_queue = [s, theta_dot]
        self.lock.release()

    def run_filter(self):
        self.lock.acquire(True)
        print(self.lock.locked())
        print("Acquired lock")
        self.lock.release()
        print("releasing lock")

        # Check if control input
        # if not self.control_queue: # no control input
        #     pass
        # else:
        #     [u_s, u_theta_dot] = self.control_queue
        #     # do something...
        # if not self.meas_queue:
        #     pass
        # else:
            # do stuff
            
        
        # Check if measurement

        # if self.last_update_time == None: # initialize with first measurement
        #     x = msg.data[0]
        #     y = msg.data[1]
        #     theta = msg.data[THETA_INDEX]
        #     x_dot = msg.data[SPEED_INDEX] * np.cos(msg.data[THETA_INDEX])
        #     y_dot = msg.data[SPEED_INDEX] * np.sin(msg.data[THETA_INDEX])
        #     theta_dot = msg.data[THETA_DOT_INDEX]
        #     self.estimate = np.array([[x, y, theta, x_dot, y_dot, theta_dot]]).T # x, y, theta, x_dot, y_dot, theta_dot
        #     self.uncertainty = np.eye(6) * 100
        #     self.last_update_time = rospy.get_time()
        #     print("Initialized ekf with first meas")
        # else:
        #     z = np.array([[msg.data[0]],\
        #                   [msg.data[1]],\
        #                   [msg.data[2]],\
        #                   [msg.data[3]],\
        #                   [msg.data[4]]])
        #     time = rospy.get_time()
        #     dt = time - self.last_update_time
        #     self.last_update_time = time
        #     G = np.array([[1,0,0,dt,0,0], \
        #                   [0,1,0,0,dt,0],\
        #                   [0,0,1,0,0,dt],\
        #                   [0,0,0,1,0,0],\
        #                   [0,0,0,0,1,0],\
        #                   [0,0,0,0,0,1]]) # No nonlinearities here..., all our nonlinearity is in the update step

        #     # Prediction
        #     mu_bar = np.dot(G, self.estimate) # linear dynamics
        #     sigma_bar = np.dot(np.dot(G, self.uncertainty), G.T)
        #     # self.debug_prediction(G, self.estimate, self.uncertainty, mu_bar, sigma_bar)

        #     # Extract Relevant prediction data
        #     x = float(mu_bar[0])
        #     y = float(mu_bar[1])
        #     theta = float(mu_bar[2])
        #     x_dot = float(mu_bar[3])
        #     y_dot = float(mu_bar[4])
        #     theta_dot = float(mu_bar[5])

        #     if abs(theta- np.pi/2) < 0.1: # theta = pi/2
        #         print("theta near pi/2")
        #         alpha = 1 / np.sin(theta)
        #         beta = -y_dot * (1 / np.sin(theta)) * (1/np.tan(theta))
        #         H = np.array([[1,0,0,0,0,0],\
        #                   [0,1,0,0,0,0],\
        #                   [0,0,1,0,0,0],\
        #                   [0,0,beta,0,alpha,0],\
        #                   [0,0,0,0,0,1]])
        #         predicted_s = y_dot / np.sin(theta)
        #         h = np.array([[x],\
        #                       [y],\
        #                       [theta],\
        #                       [predicted_s],\
        #                       [theta_dot]])
        #     else:
        #         print("theta away from pi/2")
        #         alpha = 1 / np.cos(theta)
        #         beta = x_dot * (1 / np.cos(theta)) * np.tan(theta)
        #         H = np.array([[1,0,0,0,0,0],\
        #                   [0,1,0,0,0,0],\
        #                   [0,0,1,0,0,0],\
        #                   [0,0,beta,alpha,0,0],\
        #                   [0,0,0,0,0,1]])
        #         predicted_s = x_dot / np.cos(theta)
        #         h = np.array([[x],\
        #                       [y],\
        #                       [theta],\
        #                       [predicted_s],\
        #                       [theta_dot]])
        #     # Correction
        #     tmp = np.dot(np.dot(H,sigma_bar),H.T) + self.meas_noise
        #     tmp_inv = np.linalg.inv(tmp)
        #     sigma_bar_HT_prod = np.dot(sigma_bar,H.T)
        #     K = np.dot( sigma_bar_HT_prod, tmp_inv)
        #     innovation = np.subtract(z, h)

        #     tmp2 = np.dot(K, innovation)
        #     self.estimate = mu_bar + tmp2
        #     self.uncertainty = np.dot( (np.eye(6) - np.dot(K,H)), sigma_bar)
        #     # self.debug_correction(H, h, z, mu_bar, sigma_bar, tmp, tmp_inv, sigma_bar_HT_prod,K, innovation, tmp2, self.estimate, self.uncertainty)
        #     # print(self.estimate)
        #     diag = np.reshape(np.diagonal(self.uncertainty), (self.uncertainty.shape[0], 1))
        #     print(diag)
        #     print("---")

    def meas_callback(self, msg):
        self.lock.acquire()
        self.meas_queue = msg.data
        self.lock.release()

    def debug_prediction(self, G, mu_old, sigma_old, mu_bar, sigma_bar):
        print("------------------------------------------------")
        print("Debug Prediction Step:")
        print("G: " + str(G.shape))
        print(G)
        print("mu_old: " + str(mu_old.shape))
        print(mu_old)
        print("sigma_old: " + str(sigma_old.shape))
        print(sigma_old)
        print("mu_bar: " + str(mu_bar.shape))
        print(mu_bar)
        print("sigma_bar: " + str(sigma_bar.shape))
        print(sigma_bar)

    def debug_correction(self, H, h, z, mu_bar, sigma_bar, tmp, tmp_inv, sigma_bar_HT_prod, K, innovation, tmp2, mu, sigma):
        print("------------------------------------------------")
        print("Debug Correction Step:")
        print("H: " + str(H.shape))
        print(H)
        print("h: " + str(h.shape))
        print(h)
        print("z: " + str(z.shape))
        print(z)
        print("mu_bar: " + str(mu_bar.shape))
        print(mu_bar)
        print("sigma_bar: " + str(sigma_bar.shape))
        print(sigma_bar)

        print("tmp: " + str(tmp.shape))
        print(tmp)

        print("tmp_inv: " + str(tmp_inv.shape))
        print(tmp_inv)

        print("sigma_bar_HT_prod: " + str(sigma_bar_HT_prod.shape))
        print(sigma_bar_HT_prod)

        print("K: " + str(K.shape))
        print(K)

        print("innovation: " + str(innovation.shape))
        print(innovation)

        print("tmp2: " + str(tmp2.shape))
        print(tmp2)

        print("mu: " + str(mu.shape))
        print(mu)

        print("sigma: " + str(sigma.shape))
        print(sigma)


if __name__ == "__main__":
    rospy.init_node("ekf")
    ekf = EKF()
    r = rospy.Rate(25)
    while not rospy.is_shutdown():
        ekf.run_filter()
        r.sleep()