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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	    #
        q1, q2, q3, q4, q5, q6, q7 = symbols("q1:8") # theta_i
        d1, d2, d3, d4, d5, d6, d7 = symbols("d1:8") # d_i
        a0, a1, a2, a3, a4, a5, a6 = symbols("a0:7") # a_i
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols("alpha0:7")

	    #
	    # Create Modified DH parameters
	    #
        s = {
            alpha0:     0, a0:      0, d1:  0.75, q1:       0,
            alpha1: -pi/2, a1:   0.35, d2:     0, q2:       q2-pi/2,
            alpha2:     0, a2:   1.25, d3:     0, q3:       0,
            alpha3: -pi/2, a3: -0.054, d4:  1.50, q4:       0,
            alpha4:  pi/2, a4:      0, d5:     0, q5:       0,
            alpha5: -pi/2, a5:      0, d6:     0, q6:       0,
            alpha6:     0, a6:      0, d7: 0.303, q7:       0
        }

	    #
	    # Define Modified DH Transformation matrix
	    #
        T0_1 = dhHtm(alpha0, q1, a0, d1)
        T1_2 = dhHtm(alpha1, q2, a1, d2)
        T2_3 = dhHtm(alpha2, q3, a2, d3)
        T3_4 = dhHtm(alpha3, q4, a3, d4)
        T4_5 = dhHtm(alpha4, q5, a4, d5)
        T5_6 = dhHtm(alpha5, q6, a5, d6)
        T6_G = dhHtm(alpha6, q7, a6, d7)

        # perform symbolic substitution with the defined symbol values
        T0_1 = T0_1.subs(s)
        T1_2 = T1_2.subs(s)
        T2_3 = T2_3.subs(s)
        T3_4 = T3_4.subs(s)
        T4_5 = T4_5.subs(s)
        T5_6 = T5_6.subs(s)
        T6_G = T6_G.subs(s)

        R0_1 = T0_1[:3, :3]
        R1_2 = T1_2[:3, :3] 
        R2_3 = T2_3[:3, :3]
        R3_4 = T3_4[:3, :3]
        R4_5 = T4_5[:3, :3]
        R5_6 = T5_6[:3, :3]
        R6_G = T6_G[:3, :3]

	    #
	    # Create individual transformation matrices
	    #
        T0_2 = simplify(T0_1 * T1_2)
        T0_3 = simplify(T0_2 * T2_3)
        T0_4 = simplify(T0_3 * T3_4)
        T0_5 = simplify(T0_4 * T4_5)
        T0_6 = simplify(T0_5 * T5_6)
        T0_G = simplify(T0_6 * T6_G)

	    #
	    # Extract rotation matrices from the transformation matrices
	    #
	    #
        R0_1 = T0_1[:3, :3]
        R0_2 = T0_2[:3, :3]
        R0_3 = T0_3[:3, :3]
        R0_4 = T0_4[:3, :3]
        R0_5 = T0_5[:3, :3]
        R0_6 = T0_6[:3, :3]
        R0_G = T0_G[:3, :3]

        ###

        # Initialize service response
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
            Rwc_ee = Matrix([roll, 0, 0], [0, pitch, 0], [0, 0, yaw])

            ### Your IK code here
	        # Compensate for rotation discrepancy between DH parameters and Gazebo
	        #
	        #
            R_corr = rot_z(-pi) * rot_y(pi/2) # reverse the method of compensating mentioned in the video
            R_corr.simplify()

            R_rpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_corr

            # Get the wrist-center
            ee_magnitude = pwc_ee + Matrix([.54 + .193, 0, 0])
            l = sqrt(ee_magnitude[0]**2 + ee_magnitude[1]**2 + ee_magnitude[2]**2) # length/magnitude of ee from wc
            p0_wc = (pwc_ee - (s["d6"] + l)) * R_rpy  # wc in terms of the base link

	        # Calculate joint angles using Geometric IK method
	        #
	        #
            ###
            theta1 = atan2(p0_wc[1], p0_wc[0])
            side_a = s["d4"]
            side_b = sqrt(s["a2"]**2 + s["d4"])
            side_c = s["a2"]

            theta2 = acos((side_a**2 - side_c**2 - side_b**)/-(2*side_c*side_b))
            theta3 = acos((side_b**2 - side_c**2 - side_a**2)/-(2*side_c*side_a))

            # R0_1 = R_01.subs({q1: theta1})
            # R0_1.simplify()
            # R1_2 = R1_2.subs({q2: theta2})
            # R1_2.simplify()
            # R2_1 = R2_1.subs({q3: theta3})
            # R2_1.simplify()

            R0_3 = R0_3.subs(q1: thet1, q2: theta2: q3: theta3)
            R0_3.simplify()
            R3_6 = R0_3.inv("LU") * R0_6
            theta4 = atan2(R3_6[1, 0], R3_6[0, 0])
            theta5 = atan(-R3_6[2, 0], sqrt(R3_6[0, 0]**2 + R3_6[1, 0]**2))
            theta6 = atan2(R3_6[2, 1], R3_6[2, 2])

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
