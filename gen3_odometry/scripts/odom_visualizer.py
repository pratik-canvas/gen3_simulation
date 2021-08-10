#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import message_filters
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class Odomplotter():

    def __init__(self):

        self.actual_x = []
        self.actual_y = []
        self.odom_x = []
        self.odom_y = []

        self.odom_sub = message_filters.Subscriber('/odom',Odometry)
        self.actual_sub = message_filters.Subscriber('/ground_truth',Odometry)


        self.ts = message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.actual_sub],10,1)
        self.ts.registerCallback(self.callback)

    def callback(self,odom,actual):

        self.odom_x.append(odom.pose.pose.position.x)
        self.odom_y.append(odom.pose.pose.position.y)
        self.actual_x.append(actual.pose.pose.position.x)
        self.actual_y.append(actual.pose.pose.position.y)

def animate(i,pose):

    plt.clf()
    plt.ylim(10,-1)
    plt.xlim(10,-1)
    plt.plot(pose.actual_x, pose.actual_y, 'g-',label='Actual')
    plt.plot(pose.odom_x, pose.odom_y, 'b--',label='Predicted')
    plt.legend(loc='upper left')
    plt.tight_layout()


if __name__ == '__main__':

    rospy.init_node('odom_plotter')
    pose = Odomplotter()

    ani = FuncAnimation(plt.gcf(), animate,fargs=(pose,), interval=100)

    plt.tight_layout()

    plt.show()
    rospy.spin()
