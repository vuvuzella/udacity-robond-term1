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
q1, q2, q3, q4, q5, q6, q7 = symbols("q1:8") # theta_i
d1, d2, d3, d4, d5, d6, d7 = symbols("d1:8") # d_i
a0, a1, a2, a3, a4, a5, a6 = symbols("a0:7") # a_i
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols("alpha0:7")

### KUKA KR210 DH parameters
### parameters already taken into account the urdf's parent joint based displacement notation
s = {
    alpha0:     0, a0:      0, d1:  0.75, q1:       0,
    alpha1: -pi/2, a1:   0.35, d2:     0, q2:       q2-pi/2,
    alpha2:     0, a2:   1.25, d3:     0, q3:       0,
    alpha3: -pi/2, a3: -0.054, d4:  1.50, q4:       0,
    alpha4:  pi/2, a4:      0, d5:     0, q5:       0,
    alpha5: -pi/2, a5:      0, d6:     0, q6:       0,
    alpha6:     0, a6:      0, d7: 0.303, q7:       0
}

# qX theta value we can give to the transform to get the end effector position
var_subs = {q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}

if __name__ == "__main__":
    # execute the creation of forward kinematics transformation matrices
    ## Create Homogeneous transforms
    T0_1 = dhHtm(alpha0, q1, a0, d1)
    T1_2 = dhHtm(alpha1, q2, a1, d2)
    T2_3 = dhHtm(alpha2, q3, a2, d3)
    T3_4 = dhHtm(alpha3, q4, a3, d4)
    T4_5 = dhHtm(alpha4, q5, a4, d5)
    T5_6 = dhHtm(alpha5, q6, a5, d6)
    T6_G = dhHtm(alpha6, q7, a6, d7)

    T0_1 = T0_1.subs(s)
    T1_2 = T1_2.subs(s)
    T2_3 = T2_3.subs(s)
    T3_4 = T3_4.subs(s)
    T4_5 = T4_5.subs(s)
    T5_6 = T5_6.subs(s)
    T6_G = T6_G.subs(s)

    ## Compose the Homogeneous Transforms
    # simplify(0 to 1), then 1 to 2 etc...
    T0_2 = simplify(T0_1 * T1_2)
    T0_3 = simplify(T0_2 * T2_3)
    T0_4 = simplify(T0_3 * T3_4)
    T0_5 = simplify(T0_4 * T4_5)
    T0_6 = simplify(T0_5 * T5_6)
    T0_G = simplify(T0_6 * T6_G)

    ## Get the rotation matrices in each transform
    R0_1 = T0_1[:3, :3]
    R0_2 = T0_2[:3, :3]
    R0_3 = T0_3[:3, :3]
    R0_4 = T0_4[:3, :3]
    R0_5 = T0_5[:3, :3]
    R0_6 = T0_6[:3, :3]
    R0_G = T0_G[:3, :3]

    ## Numerically evaluate transforms (compare this with output of tf_echo)
    # print("link 0 to 1:")
    # pprint(T0_1.evalf(subs=var_subs))   # link 1
    # print("link 0 to 2:")
    # pprint(T0_2.evalf(subs=var_subs))
    # print("link 0 to 3:")
    # pprint(T0_3.evalf(subs=var_subs))
    # print("link 0 to 4:")
    # pprint(T0_4.evalf(subs=var_subs))
    # print("link 0 to 5:")
    # pprint(T0_5.evalf(subs=var_subs))
    # print("link 0 to 6:")
    # pprint(T0_6.evalf(subs=var_subs))

    # Take into account the gripper correction taken from the urdf,
    # vis-a-vis the DH convention used
    # 1. Apply a body-fixed (Intrinsic) rotation about the z-axix,
    #    then about the y-axis
    #    -> R_corr = the composition of rotations 

    # z_rot, y_rot = symbols("z_rot, rot_y")
    # corr = rot_z(z_rot) * rot_y(y_rot)
    corr = rot_z(pi) * rot_y(-pi/2)
    corr.simplify()
    R_corr = htm(corr, Matrix([0, 0, 0]))

    T_total = simplify(T0_G * R_corr)
    # print("Link 0 to G:")
    # pprint(T_total.evalf(subs=var_subs))

    # Get the wrist-center
    T0_G = T0_G.subs(var_subs)
    p0_eef = T0_G[:3, 3]
    d = sqrt(0.843**2)    # the magnitude of the vector difference between wc and eef
    p0_wc = p0_eef - d * R0_6.subs(var_subs) * Matrix([0, 0, 1])
    pprint(p0_wc)