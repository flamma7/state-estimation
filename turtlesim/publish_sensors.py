"""
Takes in the turtle's pose and outputs a measurement

Sensor configurations in config/turtle.yaml
"""
import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32MultiArray
import numpy as np

def pose_callback_gps(msg):
    fma = Float32MultiArray()
    fma.data.append(msg.x + np.random.normal(0,1))
    fma.data.append(msg.y + np.random.normal(0,1))
    pub_gps.publish(fma)

def pose_callback_all(msg):
    locked = True
    fma.data = []
    fma.data.append(msg.x + np.random.normal(0,1))
    fma.data.append(msg.y + np.random.normal(0,1))
    fma.data.append(msg.theta + np.random.normal(0,0.1))
    fma.data.append(msg.linear_velocity + np.random.normal(0,0.1))
    fma.data.append(msg.angular_velocity + np.random.normal(0,0.01))
    locked = False

locked = False
fma = Float32MultiArray()
rospy.init_node("sensors")
pub_gps = rospy.Publisher("/turtle1/gps", Float32MultiArray, queue_size=10)
pub_all = rospy.Publisher("/turtle1/meas", Float32MultiArray, queue_size=10)
rospy.Subscriber("/turtle1/pose", Pose, pose_callback_gps)
rospy.Subscriber("/turtle1/pose", Pose, pose_callback_all)
rospy.wait_for_message("/turtle1/pose", Pose)
rospy.sleep(2)

r = rospy.Rate(2) # 2hz
while not rospy.is_shutdown():
    while locked:
        time.sleep(0.1)
    pub_all.publish(fma)
    r.sleep()