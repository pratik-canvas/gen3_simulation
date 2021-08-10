#!/usr/bin/env python
import rospy
from simplebuffer import Buffer
from nav_msgs.msg import Odometry
import message_filters
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
import numpy as np


def sign(a,b):

    return 1 if a*b>0 else -1 if a*b<0 else 0

class SlipEstimator():

    def __init__(self,d,n):

        self.v_theta = 0
        self.time = 0
        self.left=0
        self.right=0
        self.slip_l=0
        self.slip_r=0
        self.d=d
        self.n=n

        self.a_l=Buffer(100,1)
        self.a_r=Buffer(100,1)

        self.twist_sub = message_filters.Subscriber('/imu',Imu)
        self.velocity_sub = message_filters.Subscriber('/joint_states',JointState)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.twist_sub, self.velocity_sub],10,1)
        self.ts.registerCallback(self.callback)



    def callback(self,imu,joint):

        self.theta_dot = round(imu.angular_velocity.z,4)
        self.left=round(joint.velocity[0],4)*0.105
        self.right=round(joint.velocity[1],4)*0.105

        try:
            K=pow(abs(self.right/self.left),self.n)
            self.slip_r=(self.right-self.left-2*self.d*self.theta_dot)/(self.right +  K*sign(self.left,self.right)*self.left )
            self.slip_l=-sign(self.left,self.right)*self.slip_r*K
        except ZeroDivisionError:
            self.slip_l=0
            self.slip_r=0

        self.a_l.append(self.slip_l)
        self.a_r.append(self.slip_r)


if __name__ == '__main__':

    rospy.init_node('slip_estimator')
    pose = SlipEstimator(0.32,0.88)
    rate = rospy.Rate(10) # 10hz
   
    
    while not rospy.is_shutdown():

        a_l=np.clip(pose.a_l.running_mean()[-1],-1,1).item()
        a_r=np.clip(pose.a_r.running_mean()[-1],-1,1).item()
        rospy.set_param('alpha_l', a_l)
        rospy.set_param('alpha_r', a_r)
        rate.sleep()
        
    rospy.spin()  
