#!/usr/bin/env python

import numpy as np
import math
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import geometry_msgs.msg
import tf2_ros

class OdomNode:

    """docstring for OdomNode"""

    def __init__(self,d):

        self.alpha_l = 0
        self.alpha_r = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        self.d = d

        self.left_vel=0
        self.right_vel=0
        self.joint_sub = rospy.Subscriber( '/joint_states', JointState,self.odomCallback)
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)

    def wrap_theta(self, theta):

        if theta > math.pi:
            theta += -math.pi * 2
        elif theta < -math.pi:
            theta += math.pi * 2
        return theta

    def update(self):

        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()
        x_increment = ((self.right_vel *(1-self.alpha_r) + self.left_vel *(1-self.alpha_l))*math.cos(self.theta))/2
        y_increment = ((self.right_vel *(1-self.alpha_r) + self.left_vel *(1-self.alpha_l))*math.sin(self.theta))/2
        heading_increment = ((self.right_vel *(1-self.alpha_r) - self.left_vel *(1-self.alpha_l)))/(2*self.d)

        self.x = self.x + x_increment*dt
        self.y = self.y + y_increment*dt
        self.theta = self.wrap_theta(self.theta + heading_increment*dt)

        self.inst_speed = (self.right_vel *(1-self.alpha_r) + self.left_vel *(1-self.alpha_l))/2
        self.inst_rot_speed = (self.right_vel *(1-self.alpha_r) - self.left_vel *(1-self.alpha_l))/(2*self.d)



    def odomCallback(self,data):

        self.alpha_l=rospy.get_param("alpha_l")
        self.alpha_r=rospy.get_param("alpha_r")

        self.left_vel=round(data.velocity[0]*0.105,4)
        self.right_vel=round(data.velocity[1]*0.105,4)

        odom_broadcaster = tf2_ros.TransformBroadcaster()

        self.update( )

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = quaternion_from_euler(0, 0, self.theta)


        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = self.current_time
        t.header.frame_id = "base_link"
        t.child_frame_id = "odom"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x =odom_quat[0]
        t.transform.rotation.y = odom_quat[1]
        t.transform.rotation.z = odom_quat[2]
        t.transform.rotation.w = odom_quat[3]

        odom_broadcaster.sendTransform(t)

        # next, we'll publish the odometry message over ROS
        odom_msg = Odometry()
        odom_msg.header.stamp = self.current_time
        odom_msg.header.frame_id = "odom"

        # set the position
        odom_msg.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom_msg.child_frame_id = "base_link"
        odom_msg.twist.twist = Twist(Vector3(odom.inst_speed, 0, 0), Vector3(0, 0, odom.inst_rot_speed))

        # publish the message
        self.pub_odom.publish(odom_msg)

        self.last_time = self.current_time

if __name__ == '__main__':

    rospy.init_node('pose_estimation', anonymous=True)

    d = 0.32
    fixed_frequency = 10

    odom = OdomNode(d)

    rate = rospy.Rate(fixed_frequency)

    while not rospy.is_shutdown():
        rate.sleep()

    rospy.spin()
