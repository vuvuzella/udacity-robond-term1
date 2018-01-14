#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

from kinematics_functions import *

### Your FK code here
#
# Create symbols
#
q1, q2, q3, q4, q5, q6, q7 = symbols("q1:8") # theta_i
d1, d2, d3, d4, d5, d6, d7 = symbols("d1:8") # d_i
a0, a1, a2, a3, a4, a5, a6 = symbols("a0:7") # a_i
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols("alpha0:7")

#
# Create Modified DH parameters
#
print("Creating DH parameters")
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

#
# perform symbolic substitution with the defined symbol values
#
print("Perform symbolic substitution on individual htm's")
T0_1 = T0_1.subs(s)
T1_2 = T1_2.subs(s)
T2_3 = T2_3.subs(s)
T3_4 = T3_4.subs(s)
T4_5 = T4_5.subs(s)
T5_6 = T5_6.subs(s)
T6_G = T6_G.subs(s)

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
T3_G = simplify(T3_4 * T4_5 * T5_6 * T6_G)
simplify(T3_G)
pprint(T3_G[:3, :3])

#
# Extract rotation matrices from the transformation matrices
#
#
print("Extract needed rotation matrix")
R0_3 = T0_3[:3, :3]
R3_G2 = T3_G[:3, :3]

# Compensate for rotation discrepancy between DH parameters and Gazebo
#
#
r, p, y = symbols('r p y')
print("Creating end effector correction rotation")
R_corr = rot_z(pi) * rot_y(-pi/2) # reverse the method of compensating mentioned in the video
R_rpy = rot_z(y) * rot_y(p) * rot_x(r) * R_corr

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        ###

        # Initialize service response
        print("Total poses: " + str(len(req.poses)))
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	        # Extract end-effector position and orientation from request
	        # px,py,pz = end-effector position
	        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
            pwc_ee = Matrix([px, py, pz])

            ### Your IK code here

            # R_rpy_eval = R_rpy.evalf(subs={r: roll, p: pitch, y: yaw})
            R_rpy_eval = R_rpy.subs({r: roll, p: pitch, y: yaw})
            # Get the wrist-center
            p0_wc = pwc_ee - (s[d7]) * R_rpy_eval[:,2]  # wc in terms of the base link

	        # Calculate joint angles using Geometric IK method
	        #
	        #

            # Using law of cosines to get theta 1, 2 and 3
            # set the length of the sides of the triangle joint2, 3 and 4
            side_a = s[d4]
            side_b = sqrt(pow((sqrt(p0_wc[0]**2 + p0_wc[1]**2) - s[a1]), 2) + pow((p0_wc[2] - s[d1]), 2))  # i dont know why?!
            side_c = s[a2]

            # Get the angles
            angle_a = abs(acos((side_b**2 + side_c**2 - side_a**2)/(2*side_b*side_c)))
            angle_b = abs(acos((side_a**2 + side_c**2 - side_b**2)/(2*side_a*side_c)))
            # angle_c = acos((side_a**2 + side_b**2 - side_c**2)/(2*side_a*side_b))

            # Get theta1, 2 and 3
            # using pythagorean. Draw the kinematics out to understand
            theta1 = atan2(p0_wc[1], p0_wc[0]) 
            # theta2 is a subset of angle_a, so we remove the angle that's not related to theta2
            theta2 = pi/2 - angle_a - atan2(p0_wc[2] - s[d1], sqrt(p0_wc[0]**2 + p0_wc[1]**2) - s[a1])
            # theta3 is a subset of angle_b, so we remove the angle that is not theta3
            sag_angle = atan2(0.054, sqrt(1.501**2 - 0.054**2))
            theta3 = pi/2 - (angle_b + sag_angle)

            # Get the rotation from joint 3 to the ee
            # evaluate the rotation from base link to link3 (joint 0 to joint 3)
            # because we now have the theta for these joints that defines the pose of the wrist center
            R0_3_eval = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            # The inverse of R0_3_eval yields R3_0, R_rpy_eval is 
            # R3_6 = R0_3_eval.inv("LU") * R_rpy_eval
            R3_6 = R0_3_eval.transpose() * R_rpy_eval

            # We get theta4, 5, 6 from R3_G which now contains the evaluated
            # rotation from base link to wrist center
            theta5 = atan2(sqrt(R3_6[0, 2]**2 + R3_6[2, 2]**2), R3_6[1, 2])
            # If sin(theta5) is negative, switch appropriate sign to keep values positive
            if sin(theta5) < 0:
                theta4 = atan2(-R3_6[2, 2], R3_6[0, 2])
                theta6 = atan2(R3_6[1, 1], -R3_6[1, 0])
            else:
                theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
                theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])

            # Generate and print the error from the computed and expected end
            # effector
            # get the computed end effector using the computed thetas
            your_ee = T0_G.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4:
                                       theta4, q5: theta5, q6: theta6})[:3, 3]
            ee_x_e = abs(your_ee[0]-pwc_ee[0])
            ee_y_e = abs(your_ee[1]-pwc_ee[1])
            ee_z_e = abs(your_ee[2]-pwc_ee[2])
            ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
            print ("Computed EE: ")
            pprint(pwc_ee)
            print ("Expected EE:")
            pprint(your_ee)
            print ("End effector error for x position is: %04.8f" % ee_x_e)
            print ("End effector error for y position is: %04.8f" % ee_y_e)
            print ("End effector error for z position is: %04.8f" % ee_z_e)
            print ("Overall end effector offset is: %04.8f units \n" % ee_offset)

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
