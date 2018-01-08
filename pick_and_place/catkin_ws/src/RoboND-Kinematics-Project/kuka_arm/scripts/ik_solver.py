# ik_solver.py
# solves the inverse kinematics

# 1. Calculate the location of the sperical wrist center,
#    i.e point O4, O5 and O6 in DH model
#    0rwc/0o = 
# 2. Use trigonometry to solve for the first 3 joint angles
#    (refer to inverse kinematics example)
# 3. The orientation of the end effector is known from ROS.
#    Find joint angles 4, 5, 6 as demonstrated in Euler Angles from
#    rotation matrix