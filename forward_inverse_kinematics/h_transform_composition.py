# homogeneuous transform composition
# From Frame A to B to E:
# 
# Frame A: Located at [0, 0, 0]
# Frame B: Rotate Frame A about a_y by -90 degrees. Translate A by [-2, 2, 4]
# Frame E: Rotate Frame B about b_x by 90 degrees. Translate B by [0, 2, 0]
# From Frame A to C to D to E:
# 
# Frame C: Translate A by [4, 4, 0]
# Frame D: Rotate Frame C about c_x by 90 degrees. Translate C by [-3, 3, 2]
# Frame E: Rotate Frame D about d_Z by 90 degrees. Translate D by [-3, 2, 3]
from sympy import symbols, cos, sin, pi, sqrt, simplify
from sympy.matrices import Matrix

# Create symbols for joint variables
q1, q2, q3, q4 = symbols("q1:5")

# radians to degree conversion and vice versa
rtd = 180.0 / pi
dtr = pi / 180.0

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

## Rotations between frames
# Initial rotation matrix for frame A
Ra = Matrix([[1, 0, 0],
             [0, 1, 0],
             [0, 0, 1]])

# Rotations performed on Frames A->B->E
Rb_a = rot_y((360 - 90) * dtr)
Re_b = rot_x(90 * dtr)

# Rotations performed on Frames A->C->D->E
Rc_a = Ra   # no rotation happened
Rd_c = rot_x(90 * dtr)
Re_d = rot_z(90 * dtr)

# Define Translations between frames
tb_a = Matrix([-2, 2, 4])
te_b = Matrix([0, 2, 0])

tc_a = Matrix([4, 4, 0])
td_c = Matrix([-3, 3, 2])
te_d = Matrix([-3, 2, 3])

## Define homogeneous transformation matrices
# NOTE: Check sympy's docu for row_join and col_join
filler = Matrix([[0, 0, 0, 1]])
Ta = Ra.col_join(Matrix([[0, 0, 0]])).row_join(Matrix([0, 0, 0, 1]))

Tb_a = Rb_a.row_join(tb_a)
Tb_a = Tb_a.col_join(filler)
Te_b = Re_b.row_join(te_b)
Te_b = Te_b.col_join(filler)

Tc_a = Rc_a.row_join(tc_a).col_join(filler)
Td_c = Rd_c.row_join(td_c).col_join(filler)
Te_d = Re_d.row_join(te_d).col_join(filler)

## Composition of Transformations
Te_a_1 = simplify(Ta * Tb_a * Te_b)
Te_a_2 = simplify(Ta * Tc_a * Td_c * Te_d)

## Calculate orientation and position for E
E_1 = Te_a_1.evalf(subs={q1: 0, q2: 0}, chop=True)
E_2 = Te_a_2.evalf(subs={q3: 0, q4: 0}, chop=True)

print("Transformation Matrix for A->B->E:")
print(E_1)

print("Transformation Matrix for A->C->D->E:")
print(E_2)