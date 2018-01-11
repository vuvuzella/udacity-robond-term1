# solves for the forward kinematics
from kinematics_functions import *

## Obtain individual transformation matrices from the calculated DH parameters table
# Create symbols
# Create modified dh parameters
# Define modified dh transformation matrix
# Create individual transformation matrices
# Extract rotation matrices from the transformation matrices

## Calculate the joint angles based on the position and orientation of the end-effecor
# Extract end-effector position and orientation from request
# Compensate for the rotation discrepancy between the DH parameters and Gazebo
# Calculate joint angles using geometric IK method

import numpy as np
from numpy import array
from sympy.matrices import Matrix
from sympy import symbols, atan2, sqrt, pi, cos, sin, atan2, simplify, pprint

from kinematics_functions import *

# Create symbols for variables
#
q1, q2, q3, q4, q5, q6, q7 = symbols("q1:8") # theta_i
d1, d2, d3, d4, d5, d6, d7 = symbols("d1:8") # d_i
a0, a1, a2, a3, a4, a5, a6 = symbols("a0:7") # a_i
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols("alpha0:7")

#
# Create Modified DH parameters
#
s = {
    alpha0:     0, a0:      0, d1:  0.75, q1:       q1,         # i = 1
    alpha1: -pi/2, a1:   0.35, d2:     0, q2:       q2-pi/2,    # i = 2
    alpha2:     0, a2:   1.25, d3:     0, q3:       q3,         # i = 3
    alpha3: -pi/2, a3: -0.054, d4:  1.50, q4:       q4,         # i = 4
    alpha4:  pi/2, a4:      0, d5:     0, q5:       q5,         # i = 5
    alpha5: -pi/2, a5:      0, d6:     0, q6:       q6,         # i = 6
    alpha6:     0, a6:      0, d7: 0.303, q7:       0           # i = 7
}

#
# Define Modified DH Transformation matrix
#
# dhHtm() is defined in kinematics_functions.py
print("\nGetting individual htm")
T0_1 = dhHtm(alpha0, q1, a0, d1)
T1_2 = dhHtm(alpha1, q2, a1, d2)
T2_3 = dhHtm(alpha2, q3, a2, d3)
T3_4 = dhHtm(alpha3, q4, a3, d4)
T4_5 = dhHtm(alpha4, q5, a4, d5)
T5_6 = dhHtm(alpha5, q6, a5, d6)
T6_G = dhHtm(alpha6, q7, a6, d7)

print("T0_1")
pprint(T0_1)
print("T1_2")
pprint(T1_2)
print("T2_3")
pprint(T2_3)
print("T3_4")
pprint(T3_4)
print("T4_5")
pprint(T4_5)
print("T5_6")
pprint(T5_6)
print("T6_G")
pprint(T6_G)

# perform symbolic substitution with the defined symbol values
print("Perform symbolic substitution on individual htm's")
T0_1 = T0_1.subs(s)
T1_2 = T1_2.subs(s)
T2_3 = T2_3.subs(s)
T3_4 = T3_4.subs(s)
T4_5 = T4_5.subs(s)
T5_6 = T5_6.subs(s)
T6_G = T6_G.subs(s)

# Get the rotation for each htm
# R0_1 = T0_1[:3, :3]
# R1_2 = T1_2[:3, :3] 
# R2_3 = T2_3[:3, :3]
# R3_4 = T3_4[:3, :3]
# R4_5 = T4_5[:3, :3]
# R5_6 = T5_6[:3, :3]
# R6_G = T6_G[:3, :3]

#
# Create individual transformation matrices
#
print("Create htm from 0 to G")
T0_2 = simplify(T0_1 * T1_2)
T0_3 = simplify(T0_2 * T2_3)
T0_4 = simplify(T0_3 * T3_4)
T0_5 = simplify(T0_4 * T4_5)
T0_6 = simplify(T0_5 * T5_6)
T0_G = simplify(T0_6 * T6_G)

print("Before simplify")
pprint(T0_G)
T0_G.simplify()
print("After simplify")
pprint(T0_G)

#
# Extract rotation matrices from the transformation matrices
#
#
print("Extract needed rotation matrix")
# R0_1 = T0_1[:3, :3]
# R0_2 = T0_2[:3, :3]
R0_3 = T0_3[:3, :3]
# R0_4 = T0_4[:3, :3]
# R0_5 = T0_5[:3, :3]
# R0_6 = T0_6[:3, :3]
# R0_G = T0_G[:3, :3]
###

roll, pitch, yaw = symbols('roll pitch yaw')

R_corr = rot_z(pi) * rot_y(-pi/2) # reverse the method of compensating mentioned in the video
R_rpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_corr
R3_6 = R0_3.inv("LU") * R_rpy

sub_values = {q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0, q7: 0}

pprint(R3_6.evalf(subs=sub_values))