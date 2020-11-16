#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from math import *
import cmath
import numpy as np
import kinematics_scripts
from kinematics_scripts import fwd_kin,inv_kin

def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=50)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(50) # 50hz
    hello_str = JointState()
    hello_str.header = Header()
    hello_str.header.stamp = rospy.Time.now()
    hello_str.name = ['coxa_joint_R1', 'femur_joint_R1','tibia_joint_R1',
                      'coxa_joint_R2', 'femur_joint_R2','tibia_joint_R2',
                      'coxa_joint_R3', 'femur_joint_R3','tibia_joint_R3',
                      'coxa_joint_L1', 'femur_joint_L1','tibia_joint_L1',
                      'coxa_joint_L2', 'femur_joint_L2','tibia_joint_L2',
                      'coxa_joint_L3', 'femur_joint_L3','tibia_joint_L3']

    A = inv_kin(30,150,30)

    hello_str.position = [A[0],A[1],-A[2],
                          0,0,0,
                          0,0,0,
                          0,0,0,
                          0,0,0,
                          0,0,0]
    hello_str.velocity = []
    hello_str.effort = []
    
    while not rospy.is_shutdown():
      hello_str.header.stamp = rospy.Time.now()
      pub.publish(hello_str)
      rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

