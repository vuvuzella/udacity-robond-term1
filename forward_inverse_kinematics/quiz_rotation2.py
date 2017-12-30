from sympy import symbols, cos, sin, pi, sqrt
from sympy.matrices import Matrix

# Create symbols for joint variables
q1, q2 = symbols("q1:3")

# Create a symbolic matrix representing an extrinsic sequence of rotations 
# about the Z and then Y axes. Let the rotation about the Y axis be described
# by q1 and the rotation about Z by q2. 
####### TO DO ########
# Replace R_y and R_z with the appropriate (symbolic) elementary rotation matrices 
# and then compute ZY_extrinsic. 
R_y = Matrix([[cos(q1), 0, sin(q1)], \
              [0, 1, 0], \
              [-sin(q1), 0, cos(q1)]])
R_z = Matrix([[cos(q2), -sin(q2), 0], \
              [sin(q2), cos(q2), 0], \
              [0, 0, 1]])
# Extrinsic -> fixed axis -> pre multiply
ZY_extrinsic_sym = R_y * R_z
ZY_extrinsic_num = ZY_extrinsic_sym.evalf(subs={q1: 45*180.0/pi, q2: 60*180.0/pi})

####### TO DO ########
# Numerically evaluate ZY_extrinsic assuming:
# q1 = 45 degrees and q2 = 60 degrees. 
# NOTE: Trigonometric functions in Python assume the input is in radians!  
#ZY_extrinsic_sym = 
#ZY_extrinsic_num = ZY_extrinsic_sym.evalf(subs{})

print("ZY_extrinsic_num: ")
print(ZY_extrinsic_num)