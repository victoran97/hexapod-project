#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from math import *
import cmath
import numpy as np
import kinematics_scripts
from kinematics_scripts import fwd_kin,inv_kin

#altura e comprimento do passo, em metros


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


    hello_str.position = [0,0,0,
                          0,0,0,
                          0,0,0,
                          0,0,0,
                          0,0,0,
                          0,0,0]
    hello_str.velocity = []
    hello_str.effort = []


    stepw = 0
    steph = 0
    height = 0.02*1000
    width = 0.05*1000
    y_dist = 0.06*1000
    res = 100
    ciclo = 1

    a = 4*height/(width**2)
    b = 4*height/width
   

    R1ini = fwd_kin(0,0,0)
    R2ini = fwd_kin(0,0,0)
    R3ini = fwd_kin(0,0,0)
    L1ini = fwd_kin(0,0,0)
    L2ini = fwd_kin(0,0,0)
    L3ini = fwd_kin(0,0,0)

    R1 = (0,0,0)
    R2 = (0,0,0)
    R3 = (0,0,0)
    L1 = (0,0,0)
    L2 = (0,0,0)
    L3 = (0,0,0)

    while not rospy.is_shutdown():
      hello_str.header.stamp = rospy.Time.now()

      #Obtem a posicao da perna no momento atual, para o primeiro ciclo
      posR1 = fwd_kin(hello_str.position[0],hello_str.position[1],hello_str.position[2])
      posR2 = fwd_kin(hello_str.position[3],hello_str.position[4],hello_str.position[5])
      posR3 = fwd_kin(hello_str.position[6],hello_str.position[7],hello_str.position[8])
      posL1 = fwd_kin(hello_str.position[9],hello_str.position[10],hello_str.position[11])
      posL2 = fwd_kin(hello_str.position[12],hello_str.position[13],hello_str.position[14])
      posL3 = fwd_kin(hello_str.position[15],hello_str.position[16],hello_str.position[17])      


      if ciclo==1:
        R1x = R1ini[0]+(stepw/width)*width
        R1z = R1ini[2]-a*((stepw/width)*width)**2+b*(stepw/width)*width
        R1 = inv_kin(R1x, y_dist, R1z)
        R3x = R3ini[0]+(stepw/width)*width
        R3z = R3ini[2]-a*((stepw/width)*width)**2+b*(stepw/width)*width
        R3 = inv_kin(R3x, y_dist, R3z)
        print(R1x,R1z,(stepw/width)*width)
        L2x = L2ini[0]+(stepw/width)*width
        L2z = L2ini[2]-a*((stepw/width)*width)**2+b*(stepw/width)*width
        L2 = inv_kin(L2x, y_dist, L2z)
        #R3 = inv_kin(R3[0]+width/res, y_dist, R3ini[2]-a**2*(steph*height/res)+b*(steph*height/res))
        #L2 = inv_kin(L2[0]+width/res, y_dist, L2ini[2]-a**2*(steph*height/res)+b*(steph*height/res))
      else:
        L1x = L1ini[0]+(stepw/width)*width
        L1z = L1ini[2]-a*((stepw/width)*width)**2+b*(stepw/width)*width
        L1 = inv_kin(L1x, y_dist, L1z)
        L3x = L3ini[0]+(stepw/width)*width
        L3z = L3ini[2]-a*((stepw/width)*width)**2+b*(stepw/width)*width
        L3 = inv_kin(L3x, y_dist, L3z)
        R2x = R2ini[0]+(stepw/width)*width
        R2z = R2ini[2]-a*((stepw/width)*width)**2+b*(stepw/width)*width
        R2 = inv_kin(R2x, y_dist, R2z)
      #  R2 = inv_kin(posR2[0]+width/res, y_dist, R2ini[2]-a**2*width/res+b*width)
      #  L1 = inv_kin(posL1[0]+width/res, y_dist, L1ini[2]-a**2*width/res+b*width)
      #  L3 = inv_kin(posL3[0]+width/res, y_dist, L3ini[2]-a**2*width/res+b*width)
      
      if stepw/width == 1 and ciclo==1:
        stepw = 0
        ciclo = 2

      #  ciclo=2
      elif stepw/width == 1 and ciclo==2:
        stepw = 0
        ciclo = 1

      hello_str.position = [R1[0],R1[1],R1[2],
                            R2[0],R2[1],R2[2],
                            R3[0],R3[1],R3[2],
                            -L1[0],-L1[1],L1[2],
                            -L2[0],-L2[1],L2[2],
                            -L3[0],-L3[1],L3[2]]

      hello_str.velocity = []
      hello_str.effort = []
      
      stepw = stepw + width/res
      steph = steph + height/res 
      
      pub.publish(hello_str)

      rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass