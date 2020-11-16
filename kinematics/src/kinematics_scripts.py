#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
#import tf.transformations as tf
from math import *
import cmath
#from geometry_msgs.msg import Pose, Quaternion

coxa = 30
femur = 80
tibia = 110

user_t1 = 0
user_t2 = 0
user_t3 = 0

def fwd_kin(theta1,theta2,theta3):

    X = tibia*sin(theta1)*cos(theta2)*cos(theta3-pi/2) - tibia*sin(theta1)*sin(theta2)*sin(theta3-pi/2) + femur*sin(theta1)*cos(theta2) + coxa*sin(theta1)

    Y = tibia*cos(theta1)*cos(theta2)*cos(theta3-pi/2) - tibia*cos(theta1)*sin(theta2)*sin(theta3-pi/2) + femur*cos(theta1)*cos(theta2) + coxa*cos(theta1)

    Z = tibia*sin(theta2)*cos(theta3-pi/2) + tibia*cos(theta2)*sin(theta3-pi/2) + femur*sin(theta2)

    return X,Y,Z


def inv_kin(x,y,z):
    
    H = sqrt(x**2+y**2)-coxa
    L = sqrt(H**2+z**2)
    aux_theta2 = atan2(-z,H)

    theta1 = atan2(x,y)
    theta2 = acos((tibia**2-L**2-femur**2)/(-2*L*femur))-aux_theta2
    theta3 = pi/2 - acos((L**2-femur**2-tibia**2)/(-2*femur*tibia))

    return theta1,theta2, theta3

