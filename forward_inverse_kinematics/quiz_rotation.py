# Two coding quizzes follow. The first requires you to perform an
# intrinsic rotation sequence about the Y and then Z axes. 

# The second quiz requires you to perform an extrinsic rotation about
#  the Z and then Y axes.

from sympy import symbols, cos, sin, pi, sqrt
from sympy.matrices import Matrix

# Create symbols for joint variables
q1, q2 = symbols('q1:3')

dtr = pi / 180.0    # degree to radians
rtd = 180.0 / pi    # radians to degrees

q1 = 45 * dtr
q2 = 60 * dtr

# Create a symbolic matrix representing an intrinsic sequence of rotations 
# about the Y and then Z axes.
# Let the rotation about the Y axis be described
# by q1 and the rotation about Z by q2. 
####### TO DO ########
# Replace R_y and R_z with the appropriate (symbolic) elementary rotation matrices 
# and then compute YZ_intrinsic. 
R_y = Matrix([[cos(q1), 0, sin(q1)], \
              [0, 1, 0], \
              [-sin(q1), 0, cos(q1)]])
R_z = Matrix([[cos(q2), -sin(q2), 0], \
              [sin(q2), cos(q2), 0], \
              [0, 0, 1]])
YZ_intrinsic_sym = R_y.evalf(subs={q1:q1 }) * R_z.evalf(subs={q2:q2})

####### TO DO ########
# Numerically evaluate YZ_intrinsic assuming:
# q1 = 45 degrees and q2 = 60 degrees. 
# NOTE: Trigonometric functions in Python assume the input is in radians!  

# Intrinsic -> moving axis -> post multiply
# YZ_intrinsic_num = 1 #YZ_intrinsic_sym.evalf(subs={})
YZ_intrinsic_num = YZ_intrinsic_sym.evalf(subs={})

# Alternatively:
# YZ_intrinsic_sym = R_y * R_z
# YZ_intrinsic_num = YZ_intrinsic_sym.evalf(subs={q1: pi/4, q2: pi/3})

print("YZ_intrinsic_num: ")
print(YZ_intrinsic_num)