from sympy import *
from time import time
from mpmath import *
import tf
from kinematics_functions import *

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()

    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ### Begin Your FK code here
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
        alpha0:     0, a0:      0, d1:  0.75, q1:       q1,
        alpha1: -pi/2, a1:   0.35, d2:     0, q2:       q2-pi/2,
        alpha2:     0, a2:   1.25, d3:     0, q3:       q3,
        alpha3: -pi/2, a3: -0.054, d4:  1.50, q4:       q4,
        alpha4:  pi/2, a4:      0, d5:     0, q5:       q5,
        alpha5: -pi/2, a5:      0, d6:     0, q6:       q6,
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

    ### End FK code here

    ## (OPTIONAL) YOUR CODE HERE!

    ## End your code input for forward kinematics here!
    ########################################################################################
    
    ########################################################################################
    ## 

    ## Insert IK code here!
    
    theta1 = 0
    theta2 = 0
    theta3 = 0
    theta4 = 0
    theta5 = 0
    theta6 = 0

    px = position.x
    py = position.y
    pz = position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [orientation.x, orientation.y,
            orientation.z, orientation.w])
    pwc_ee = Matrix([px, py, pz]) # end effector based from the wrist center

    ### Your IK code here
	# Compensate for rotation discrepancy between DH parameters and Gazebo
	#
	#
    R_corr = rot_z(pi) * rot_y(-pi/2) # reverse the method of compensating mentioned in the video

    R_rpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_corr
    # R_rpy = R_rpy.evalf()

    # Get the wrist-center
    p0_wc = pwc_ee - (s[d7]) * R_rpy[:,2]  # wc in terms of the base link
    
	# Calculate joint angles using Geometric IK method
	#
	#
    ###
    theta1 = atan2(p0_wc[1], p0_wc[0])

    side_a = s[d4]
    # side_b = sqrt(s[a2]**2 + s[d4])
    side_b = sqrt(pow((sqrt(p0_wc[0]**2 + p0_wc[1]**2) - s[a1]), 2) + pow((p0_wc[2] - s[d1]), 2))  # i dont know why?!
    side_c = s[a2]

    angle_a = acos((side_b**2 + side_c**2 - side_a**2)/(2*side_b*side_c))
    angle_b = acos((side_a**2 + side_c**2 - side_b**2)/(2*side_a*side_c))
    angle_c = acos((side_a**2 + side_b**2 - side_c**2)/(2*side_a*side_b))

    theta2 = pi/2 - angle_a - atan2(p0_wc[2] - s[d1], sqrt(p0_wc[0]**2 + p0_wc[1]**2) - s[a1])
    theta3 = pi/2 - (angle_b + 0.036) 

    # theta2 = acos((side_a**2 - side_c**2 - side_b**2)/-(2*side_c*side_b))
    # theta3 = acos((side_b**2 - side_c**2 - side_a**2)/-(2*side_c*side_a))

    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
    R3_6 = R0_3.inv("LU") * R_rpy

    theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
    theta5 = atan2(sqrt(R3_6[0, 2]**2 + R3_6[2, 2]**2),R3_6[1, 2])
    theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])

    ## 
    ########################################################################################
    

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    # your_wc = [1,1,1] # <--- Load your calculated WC values in this array
    # your_ee = [1,1,1] # <--- Load your calculated end effector value from your forward kinematics
    FK = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
    FK = FK.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
    your_wc = p0_wc # <--- Load your calculated end effector value from your forward kinematics
    your_ee = [FK[0, 3], FK[1, 3], FK[2, 3]]
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    print("Running test: " + str(test_case_number))
    test_code(test_cases[test_case_number])

    test_case_number = 2
    print("Running test: " + str(test_case_number))
    test_code(test_cases[test_case_number])

    test_case_number = 3
    print("Running test: " + str(test_case_number))
    test_code(test_cases[test_case_number])
