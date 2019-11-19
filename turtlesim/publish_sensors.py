"""
Takes in the turtle's pose and outputs a measurement

Sensor configurations in config/turtle.yaml
"""
import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32MultiArray
import numpy as np
import threading

def pose_callback_all(msg):
    lock.acquire(True)
    fma.data = []
    # fma.data.append(msg.x)
    # fma.data.append(msg.y)
    # fma.data.append(msg.theta)

    fma.data.append(msg.x + np.random.normal(0,1))
    fma.data.append(msg.y + np.random.normal(0,1))
    fma.data.append(msg.theta + np.random.normal(0,0.5))
    # fma.data.append(msg.linear_velocity + np.random.normal(0,0.1))
    # fma.data.append(msg.angular_velocity + np.random.normal(0,0.01))
    lock.release()

lock = threading.Lock()
fma = Float32MultiArray()
rospy.init_node("sensors")
pub_all = rospy.Publisher("/turtle1/meas", Float32MultiArray, queue_size=10)
rospy.Subscriber("/turtle1/pose", Pose, pose_callback_all)
rospy.wait_for_message("/turtle1/pose", Pose)
rospy.sleep(2)

r = rospy.Rate(10) # 2hz
while not rospy.is_shutdown():
    lock.acquire(True)
    pub_all.publish(fma)
    lock.release()
    r.sleep()