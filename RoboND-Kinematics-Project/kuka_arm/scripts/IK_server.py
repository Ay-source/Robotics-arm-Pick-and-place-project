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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols("alpha0:7")
        a0, a1, a2, a3, a4, a5, a6 = symbols("a0:7")
        d1, d2, d3, d4, d5, d6, d7 = symbols("d1:8")
        q1, q2, q3, q4, q5, q6, q7 = symbols("q1:8")

        #
        #
        # Create Modified DH parameters
        s = {
            alpha0: 0, a0: 0, d1: 0.75, q1:q1,
            alpha1: -pi/2, a1: 0.35, d2: 0, q2: q2 + -pi/2,
            alpha2: 0, a2: 1.25, d3: 0, q3: q3,
            alpha3: -pi/2, a3: -0.054, d4: 1.50, q4: q4,
            alpha4: pi/2, a4: 0, d5: 0, q5: q5,
            alpha5: -pi/2, a5: 0, d6: 0, q6: q6,
            alpha6: 0, a6:0, d7: 0.303, q7:0
        }
	#
	#
	# Define Modified DH Transformation matrix
    def DH_Matrix(alpha, a, d, q):
        return Matrix([
            [cos(q), -sin(q), 0, a],
            [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
            [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
            [                   0,                   0,            0,               1]
        ])

    T0_1 = DH_Matrix(alpha0, a0, d1, q1).subs(s)
    T1_2 = DH_Matrix(alpha1, a1, d2, q2).subs(s)
    T2_3 = DH_Matrix(alpha2, a2, d3, q3).subs(s)
    T3_4 = DH_Matrix(alpha3, a3, d4, q4).subs(s)
    T4_5 = DH_Matrix(alpha4, a4, d5, q5).subs(s)
    T5_6 = DH_Matrix(alpha5, a5, d6, q6).subs(s)
    T6_EE = DH_Matrix(alpha6, a6, d7, q7).subs(s)
        
    #
    #3
    # Create individual transformation matrices
    T0_1 = simplify(T0_1)
    T0_2 = simplify(T0_1 * T1_2)
    T0_3 = simplify(T0_2 * T2_3)
    T0_4 = simplify(T0_3 * T3_4)
    T0_5 = simplify(T0_4 * T4_5)
    T0_6 = simplify(T0_5 * T5_6)
    T0_EE = simplify(T0_6 * T6_EE)

    #
    #
    # Extract rotation matrices from the transformation matrices
    R0_1 = T0_1[0:3, 0:3]
    R0_2 = T0_2[0:3, 0:3]
    R0_3 = T0_3[0:3, 0:3]
    R0_4 = T0_4[0:3, 0:3]
    R0_5 = T0_5[0:3, 0:3]
    R0_6 = T0_6[0:3, 0:3]
    R0_EE = T0_EE[0:3, 0:3]


    #print(T0_EE)
    #print(R0_EE)
    #print("Result of the transformation matrix")
    #print(T0_EE.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))

    #print("Result of the rotation matrix")
    #print(R0_EE.evalf(subs={q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}))
        #
	#
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

            ### Your IK code here
            r, p, y = roll, pitch, yaw
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###

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
