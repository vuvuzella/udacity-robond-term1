# kinematics_functions.py
# Provides helper functions for solving the forward and inverse kinematics
# problems
import numpy as np
from sympy.matrices import Matrix
from sympy import symbols, atan2, sqrt, pi, cos, sin, atan2, simplify 

# radians to degrees and vice versa multipliers
rtd = 180.0 /pi
dtr = pi / 180.0

# Euler Angle functions
# beta = atan2(-R_XYZ[2, 0], sqrt(R_XYZ[0,0]**2 + R_XYZ[1, 0]**2)) * rtd
# gamma  = atan2(R_XYZ[2, 1], R_XYZ[2, 2]) * rtd
# alpha = atan2(R_XYZ[1, 0], R_XYZ[0, 0]) *rtd

# Define functions for rotation matrices about x, y and z given specific angles
# Rotate on X
def rot_x(q):
    return Matrix([[1,      0,      0  ],
                   [0, cos(q), -sin(q) ],
                   [0, sin(q),  cos(q) ]])

# Rotate on Y
def rot_y(q):
    return Matrix([[cos(q),  0, sin(q)],
                   [      0, 1,      0],
                   [-sin(q), 0, cos(q)]])

# Rotate on Z
def rot_z(q):
    return Matrix([[cos(q), -sin(q), 0], 
                   [sin(q), cos(q), 0],
                   [0, 0, 1]])

# Creates the heterogeneous transform matrix
def htm(rot_mat, trans_mat):
    filler = Matrix([[0, 0, 0, 1]])
    htm = rot_mat.row_join(trans_mat).col_join(filler)
    return htm

# creates DH heterogeneous transform matrix
def dhHtm(alp, thet, a, d):
    return Matrix([[       cos(thet),           -sin(thet),              0,             a],
                   [sin(thet)*cos(alp), cos(thet)*cos(alp),-sin(alp), -sin(alp)*d],
                   [sin(thet)*sin(alp), cos(thet)*sin(alp), cos(alp),  cos(alp)*d],
                   [                    0,                     0,          0,           1]])
