from sympy import symbols, cos, sin, pi, simplify
from sympy.matrices import Matrix
import numpy as np

q1, q2, q3, q4 = symbols('q1:5')

A, R, O, C = symbols('A R O C')

# Conversion factors
rtd = 180 / np.pi   # radians to degrees
dtr = np.pi / 180   # degrees to radians

R_x = Matrix([[1, 0, 0], \
              [0, cos(q1), -sin(q1)], \
              [0, sin(q1), cos(q1)]])

R_y = Matrix([[cos(q2), 0, sin(q2)], \
              [0, 1, 0], \
              [-sin(q2), 0, cos(q2)]])

R_z = Matrix([[cos(q3), -sin(q3), 0], \
              [sin(q3), cos(q3), 0], \
              [0, 0, 1]])

print("Rotations about the x-axis by 45-degrees: ")
print(R_x.evalf(subs={q1: 45*dtr}))
print("Rotations about the y-axis by 45-degrees: ")
print(R_y.evalf(subs={q2: 45*dtr}))
print("Rotations about the z-axis by 30-degrees: ")
print(R_z.evalf(subs={q3: 30*dtr}))