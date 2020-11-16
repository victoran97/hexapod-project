import kinematics_scripts
from kinematics_scripts import fwd_kin,inv_kin
import cmath
import numpy
from math import *

print inv_kin(0,110,-110)
print inv_kin(0,110,110)
print inv_kin(80,30,110)
print inv_kin(-80,30,110)
print ""
print fwd_kin(pi/2,pi/2,pi/2)
print fwd_kin(0,0,0)