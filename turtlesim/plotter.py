"""
This file takes in an input .bag and plots the data
from the defined topics

"""
import rosbag
import matplotlib.pyplot as plt
import argparse
import numpy as np

TOPIC1 = "/turtle1/pose"
TOPIC2 = "/turtle1/ekf_mean"
TOPIC3 = "/turtle1/kf_estimate"

def plot_bags(filename):
    bag = rosbag.Bag(filename)
    legend = []
    num_msgs = bag.get_message_count(topic_filters=TOPIC1)
    topic1 = np.zeros((2,num_msgs))
    i = 0
    legend.append("gnd truth")
    for topic, msg, t in bag.read_messages(topics=[TOPIC1]):
        topic1[0][i] = msg.x
        topic1[1][i] = msg.y
        i += 1

    num_msgs = bag.get_message_count(topic_filters=TOPIC2)
    topic2 = np.zeros((2,num_msgs))
    i = 0
    legend.append("ekf")
    for topic, msg, t in bag.read_messages(topics=[TOPIC2]):
        topic2[0][i] = msg.data[0]
        topic2[1][i] = msg.data[1]
        i += 1

    num_msgs = bag.get_message_count(topic_filters=TOPIC3)
    topic3 = np.zeros((2,num_msgs))
    i = 0
    legend.append("kf")
    for topic, msg, t in bag.read_messages(topics=[TOPIC3]):
        topic3[0][i] = msg.data[0]
        topic3[1][i] = msg.data[1]
        i += 1
    
    plt.scatter(topic1[0,:], topic1[1,:],color='b')
    plt.scatter(topic2[0,:], topic2[1,:],color='g')
    plt.scatter(topic3[0,:], topic3[1,:],color='k')
    plt.legend(legend)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("EKF vs KF for Turtlesim")
    plt.show()
    bag.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plots bag files')
    parser.add_argument("filename", metavar="f", type=str, nargs="+",
        help="rosbag filename")

    args = parser.parse_args()
    plot_bags(args.filename[0])