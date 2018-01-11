## Project: Kinematics Pick & Place

## [IK_server.py can be found HERE](https://github.com/vuvuzella/udacity-robond-term1/blob/master/pick_and_place/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts/IK_server.py)

---

**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

### A. Placement of the frames

![Kinematic Frames](./writeup_files/kd_frames_xz_0_ee.png)
fig A.1

1. Frame 0

   The z-axis of frame zero (z0) is coincidental with the z1 (z-axis of frame1) by virtue of convention between the base link (w/c frame 0 is) and the first joint connected to the base-link (frame 1). However, the origin will be where x0 and z0 are perpendicular, as shown in fig. 1 labeled as o0 (origin 0). Here, x0 and x1 are not coincidental, because we want to take into account the height, in z0 or z1 direction, of the end-effector (ee)

2. Frame 1

   z1 is coincidental with z0, however, it is skewed with z2 (more info about this on frame 2). z1 is set on the axis of rotation of the revolute joint 1 (the joint whose origin is labeled o1). x1 is parallel to x0 and perpendicular to both z0 and z1.

3. Frame 2

   z2 is set on the axis of rotation of the revolute joint 2 (origin labeled as o2), going "into the page" direction as its positive rotation.

4. Frame 3 
    z3-axis of frame3 (Joint 3, a revolute joint) is parallel with z2-axis, pointing into the page as its positive rotation.
    x3-axis is coincidental with x2-axis. The origin of frame 3, o3, has a link length of a2 from the origin of o2, along x2 axis.

5. Frame 4

   Frame4's revolute joint has its z4-axis to be perpendicular with the z3-axis. The direction of z4-axis is to the left, as its positive rotation. Joint4 is part of a Spherical wrist (along with Joint 5 and 6), hence the x4-axis is set at the same intersection point as x5 and x6, making them have a common intersection with z4, z5 and z6.

6. Frame 5

   Frame5's revolute joint has its z5-axis to be pointing into the page as its positive rotation. x5 is perpendicular to z5, intersects z4, z5 and z6 (Spherical wrists are 3 joints intersecting at one common point) and shares a common origin with o4 and o6

7. Frame 6

   Frame6's revolute joint has its z6-axis to be pointing on the left direction, coincidental with z4-axis and intersecting with z5-axis. x6-axis is coincidental with x4 and x5, hence the same origin as o4 and o5.

8. Frame Gripper

   The gripper frame has the same orientation with Frame 6, but translated along the z-axis, from frame6 by a distance of dg, as depicted in fig A.1.

### B. D-H Parameters

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | d1 | q1
1->2 | -90 | a1 | 0 | -90 + q2
2->3 | 0 | a2 | 0 | q3
3->4 |  -90 | a3 | d4 | q4
4->5 | 90 | 0 | 0 | q5
5->6 | -90 | 0 | 0 | q6
6->EE | 0 | 0 | dG | q7

fig A.2

1. Link 1

   Since z0 is coincidental with z1, there is no difference in angle nor distance in these axes between the base link and joint1.

   On the other hand, we wanted to measure the link offset between link 0 and link 1, which is d1.

      + Link twist alpha0 = 0
      + Link length a0 = 0
      + Link offset d1 = 0.75 (from URDF)
      + Joint angle = theta 1

3. Link 2

   z2 is skewed with z1, hence there is a link length between z1 and z2 along the x1 axis shown as the length a1 in fig. A.1 above. It's DH parameter is defined to be the length a1.

   The angle difference between z2 and z1 on the x1 axis is said to be -90 degrees, since we need to rotate frame1 on the x1 axis by -90 degrees to align z1 to z2.

   The angle difference between x2 and x1 on the z2 axis is said to be the amount of rotation of z2-axis (theta2 or q2 in DH parameter table) plus a fixed offset of -90 degrees, since we need to offset x1 by -90 degrees, in addition to q2, to align x1 with x2

      + Link twist alpha1 = -90 (from fig A.1)
      + Link length a1 = 0.35 (from URDF)
      + Link offset d2 = 0
      + Joint angle q2 = -90 + theta2 (variable value)

4. Link 3

    There is a link length between the z-axes of joint2 and joint3 along the x2-axis, which is defined as a2.
    
    Since z3 and z2 are parallel, there is no link twist.

    Since x3 and x2 are coincidental, there is no link offset. Their joint angle difference changes by theta3

      + Link twist alpha2 = 0
      + Link length a2 = 1.25 (from URDF)
      + Link offset d3 = 0
      + Joint angle q3 = theta3 (variable value)

5. Link 4

    z4 intersects with z3 (diagrammatically it is a skew, but we remove the offset for simpler analysis). Hence, There is a -90 degree in link twist, to align z3 to z4.

    The link length between z4 and z3 is defined by the length a3 in fig. A.1

    Since we made frame4's origin to be coincidental for frame5 and frame6's origins, we consider the link offset between joint3 and joint4 to be the link offset from joint3 to joint5.

      + Link twist alpha3 = -90
      + Link length a3 = -0.54(from URDF)
      + Link offset d4 = 1.500 (from URDF joint 4 + joint 5)
      + Joint angle q4 = theta4 (variable value)

6. Link 5

    z5 intersects with z4, hence we need to rotate frame4 along x4 to align z4 with z5. In doing so, we have a 90 degree link twist.

    There is no difference in link length between z4 and z5 along x4.

    Since we have already accounted for the link offsets from joint3 to joint5 in the link4 analysis, we will not count them again here.

      + Link twist alpha4 = 90
      + Link length a4 = 0
      + Link offset d5 = 0
      + Joint angle q5 = theta5 (variable value)

7. Link 6

   z6 and z5 intersects, hence we need to rotate frame5 along x5 by -90 degrees to align with z6.

   There is no difference in link lengths between z6 and z5 along x5.

   Also, there is no difference in link offsets between x6 and x5 along z6, since they are coincidental axes.

      + Link twist alpha5 = -90
      + Link length a5 = 0
      + Link offset d6 = 0
      + Joint angle q6 = theta5 (variable value)

8. Link 7

    zG and z6 are coincidental axes, so there is no rotation needed for frame6 to align z6 to zG.

    There is also no link length difference in zG and z6 along xG, since the end effector has the same rotation as the joint before it.

    Since end effector frameG is just a translation of frame6, there is difference in xG and x6 along zG, which is defined by dG.

      + Link twist alpha5 = 0
      + Link length a5 = 0
      + Link offset d6 = 0.303 (from URDF)
      + Joint angle q6 = theta5 (variable value)

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

A. Individual Heterogeneous Transformation Matrices

1. Link 0 to Link 1

   ![T0_1](./writeup_files/htm0_1.png)

2. Link 1 to 2

   ![T1_2](./writeup_files/htm1_2.png)

3. Link 2 to 3

   ![T2_3](./writeup_files/htm2_3.png)

4. Link 3 to 4

   ![T3_4](./writeup_files/htm3_4.png)

5. Link 4 to 5

   ![T4_5](./writeup_files/htm4_5.png)

6. Link 5 to 6

   ![T5_6](./writeup_files/htm5_6.png)

7. Link 6 to G

   ![T6_G](./writeup_files/htm6_G.png)

8. Link 0 to G

   ![T0_G](./writeup_files/htm0_G.png)

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

A. Inverse Position

1. Firstly, we calculate for the rotation and position of the wrist center

   + Calculate the wrist center rotation by creating the rotation matrix applied with the correction needed to take into account the difference between the DH-parameter convention and Gazebo values. The correction matrix is the cross product of a 180-degree Z-axis rotation and a -90-degree Y-axis rotation. 

      ```correction_matrix = Rotation_z(180) * Rotation_y(-90)```

   + Then we calculate the rotation matrix from the base link to the wrist center, taking into account the correction matrix. The rotation matrix is retrieved from ROS as roll, pitch and yaw by using the quaternion functions.

      ```Rot_wc = Rotation_z(yaw) * Rotation_y(pitch) * Rotation_x(roll) * correction_matrix```

   + In our wrist center calculation, we only need the projection of zG unto z6, since that is the only rotational difference between frame6 and the gripper

      ```ZprojG_6 = correction_matrix[:,2]```

   + Then we multiply the projection of the rotation to the amount of translation along zG, from x6 to xG. The amount of translation is described by dG in our DH-parameters.

      ```translate_diff = dG * ZprojG_6```

   + Then calculate the wrist center position by subtracting the amount of translation from frame6 to the gripper (since there is no rotation grom frame6 to gripper). 
      
      ```wc_pose = ee_pose - translate_diff```

2. theta1 (q1)

   + theta1 can be solved by applying the pythagorean theorem between joint 1 and the wrist center. We use the tangent function since the triangle formed between joint1 and the wrist center have given values for the y and x values (w/c is the wrist center's pose's y and x's). Then we use the inverse of tangent to get theta

      ```theta1 = atan2(wc_pose[1], wc_pose[0])```

3. theta2 (q2) and theta3 (q3)

   ![proj_xy](./writeup_files/proj_xy.png)

   fig 3.A.1

   + In order to get theta2, we need to find the angles inside the triangle formed by joint2, joint3 and the wrist center.

   + We would use the law of cosines in deriving the angles alpha, Beta and gamma (small case a, b, c in fig 3.A.1). Side A's length is the link offset d4 and side C is the length of link length a2.

   + The length of side B is missing. We can derive its length by projecting the wrist vector unto the xy-plane.

      ```projx = sqrt(wc_pose[0]**2 + wc_pose[1]**2)```

   + We then subtract the lengths link length a1 and link offset d1, to set the vector's magnitude to be from joint 1, instead of from link 0. Then we just use the pythagorean theorem to get the length of side B.

      ```side_b = sqrt((projx-a1)**2 + (wc_pose - d1)**2)```

      ```side_a = d4```

      ```side_c = a2```

   + To find the angles inside the triangle, we now can use the law of cosines

      ```
         angle_a = acos((side_b**2 + side_c**2 - side_a**2)/(2*side_b*side_c))
         angle_b = acos((side_a**2 + side_c**2 - side_b**2)/(2*side_a*side_c))
         angle_c = acos((side_a**2 + side_b**2 - side_c**2)/(2*side_a*side_b))
      ```

   + With the angles derived, theta2 is just 90 minus angle_a, and the remaining angle. The remaining angle is derived by projecting the wrist center pose without a1 and d1, unto the xy plane, then using the resulting vector as the x component of arctangent while the z value of the crist center (without d1) is the x component:

      ```remaining_angle = atan2(pos_wc[2] - d1, projx)```
      ```theta2 = 90 - angle_a - remaining_angle```

   + theta3 is just 90 minus angle_b and some offset

      ```theta3 = 90 - angle_b + 0.036```

B. Onverse Orientation Kinematics

1. First off, we need to get the rotation matrix R0_3, which can be be derived by multuplying the heterogenous transform of T0_1, T1_2, and T2_3:

   ```R0_3 = T0_1 * T1_2 * T2_3```

   Note: Since we now have concrete values for theta1, 2 and 3, we can solve for the R0_3 matrix by substituting them and evaulating into R0_3. Hence, R0_3 will now be containg numbers instead of symbols.

2. Then we get the rotation from R3_6, by using the inverse of R0_3, multiplying it by the rotation towards the wrist center that takes into account the offset between the DH-parameters and Gazebo.

   ```R3_6 = R0_3.inv() * Rot_wc```

3. theta4 (q4)

   + After getting the rotation from joint3 to 6, we now have a matrix which we can derive the next three angle thetas.

   + For theta4, we choose R3_6[2, 2] and -R3_6[0, 2] as the y and x parameters for the inverse tangent function

   + For theta5, we get the square root of the sum of the squares of R3_6[0, 2] and R3_6[2, 2] for the y parameter, and R3_6[1, 2] for the x parameter of the inverse tangent function

   + Finally for theta6, we use -R3_6[1, 1] and R3_6[1, 0] for the y and x parameters for the inverse tangent function


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

   The overall structure of the algorithm is that, for each call to handle_calculate_IK, I would be performing the forward kinematics calculation to generate the hetero-transform matrices from base link to end effector, then for each pose that is received by the function, I would calculate the inverse kinematics to get the theta angles.

   My foremost mistake when implementing the inverse kinematics was that I did not analyze the system quite rigidly due to my lack of understanding of the concept of vector and planar projection. Again, I realized the importance of PROJECTION. PROJECTION is the key in analyzing manipulator arms. Aside from geometry and trigonometry ofcourse.

   Secondly, when running the IK_debug.py, my offset error (end effector) were getting larger for each iteration of the IK loop, BUT NOT for each call to handle_calculate_IK. I realized I did

      ```R0_3 = R0_3.evalf(subs={q1: theta1...})```

   which overwrites the rotation matrix with the evaluated matrix. To fix this, I just assigned the evaluated matrix to another variable. Dont touch the created matrix.

   Another note was that in my DH parameter table, I also included a default value for q1 - q7, which is zero. The result of this changed the output of some evaluations. Instead of putting the default value zero, I put the q1-7 symbolic parameters instead.

   One improvement I did was I commented out the unneccessary computations for individual matrices that were not used later during the IK computation.

   Some improvements could be, since I am dealing with the kuka arm the whole time, I could calculate the forward kinematics once, then use the generated transform matrices for the specific inverse kinematics calculations for every pose on a request.

   End of report.

