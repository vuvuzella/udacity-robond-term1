from sympy import symbols, cos, sin, pi, simplify, pprint
from sympy.matrices import Matrix
from h_transform_composition import rot_x, rot_y, rot_z

# FK for a SCARA manipulator

# creates DH heterogeneous transform matrix
def dhHtm(alp, thet, a, d):
    return Matrix([[       cos(thet),           -sin(thet),              0,             a],
                   [sin(thet)*cos(alp), cos(thet)*cos(alp),-sin(alp), -sin(alp)*d],
                   [sin(thet)*sin(alp), cos(thet)*sin(alp), cos(alp),  cos(alp)*d],
                   [                    0,                     0,          0,             1]])

if __name__ == "__main__":
    q1, q2, q3, q4 = symbols("q1:5")
    d1, d2, d3, d4 = symbols("d1:5")
    a0, a1, a2, a3 = symbols("a0:4")
    alpha0, alpha1, alpha2, alpha3 = symbols("alpha0:4")

    # constant link distances in meters
    a12 = 0.4500    # link between joint 1 and 2, distance in meters
    a23 = 0.3000    # link between joint 2 and 3, distance in meters

    # DH Parameters dictionary, for each column?
    s = {alpha0: 0, a0: 0,   d1: 0,
         alpha1: 0, a1: a12, d2: 0,
         alpha2: 0, a2: a23, q3: 0,
         alpha3: 0, a3: 0,   d4: 0}

    T0_1 = dhHtm(alpha0, q1, a0, d1)
    T1_2 = dhHtm(alpha1, q2, a1, d2)
    T2_3 = dhHtm(alpha2, q3, a2, d3)
    T3_4 = dhHtm(alpha3, q4, a3, d4)

    T0_1 = T0_1.subs(s)
    T1_2 = T1_2.subs(s)
    T2_3 = T2_3.subs(s)
    T3_4 = T3_4.subs(s)

    T0_4 = simplify(T0_1 * T1_2 * T2_3 * T3_4)

    pprint(T0_4.evalf(subs={q1: 0, q2: 0, d3: 0, q4: 0}))