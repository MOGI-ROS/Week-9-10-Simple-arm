#!/usr/bin/env python3

import rospy
import math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def inverse_kinematics(coords, gripper_status, gripper_angle = 0):
    '''
    Calculates the joint angles according to the desired TCP coordinate and gripper angle
    :param coords: list, desired [X, Y, Z] TCP coordinates
    :param gripper_status: string, can be `closed` or `open`
    :param gripper_angle: float, gripper angle in woorld coordinate system (0 = horizontal, pi/2 = vertical)
    :return: list, the list of joint angles, including the 2 gripper fingers
    '''
    # link lengths
    ua_link = 0.2
    fa_link = 0.25
    tcp_link = 0.175
    # z offset (robot arm base height)
    z_offset = 0.1
    # default return list
    angles = [0,0,0,0,0,0]

    # Calculate the shoulder pan angle from x and y coordinates
    j0 = math.atan(coords[1]/coords[0])

    # Re-calculate target coordinated to the wrist joint (x', y', z')
    x = coords[0] - tcp_link * math.cos(j0) * math.cos(gripper_angle)
    y = coords[1] - tcp_link * math.sin(j0) * math.cos(gripper_angle)
    z = coords[2] - z_offset + math.sin(gripper_angle) * tcp_link

    # Solve the problem in 2D using x" and z'
    x = math.sqrt(y*y + x*x)

    # Let's calculate auxiliary lengths and angles
    c = math.sqrt(x*x + z*z)
    alpha = math.asin(z/c)
    beta = math.pi - alpha
    # Apply law of cosines
    gamma = math.acos((ua_link*ua_link + c*c - fa_link*fa_link)/(2*c*ua_link))

    j1 = math.pi/2.0 - alpha - gamma
    j2 = math.pi - math.acos((ua_link*ua_link + fa_link*fa_link - c*c)/(2*ua_link*fa_link)) # j2 = 180 - j2'
    delta = math.pi - (math.pi - j2) - gamma # delta = 180 - j2' - gamma

    j3 = math.pi + gripper_angle - beta - delta

    angles[0] = j0
    angles[1] = j1
    angles[2] = j2
    angles[3] = j3

    if gripper_status == "open":
        angles[4] = 0.04
        angles[5] = 0.04
    elif gripper_status == "closed":
        angles[4] = 0.01
        angles[5] = 0.01
    else:
        angles[4] = 0.04
        angles[5] = 0.04

    return angles

rospy.init_node('send_joint_angles_ik')

pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)

controller_name = "arm_controller"
joint_names = rospy.get_param("/%s/joints" % controller_name)

rospy.loginfo("Joint names: %s" % joint_names)

rate = rospy.Rate(10)

trajectory_command = JointTrajectory()

trajectory_command.joint_names = joint_names

point = JointTrajectoryPoint()
#['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_joint', 'left_finger_joint', 'right_finger_joint']
#joint_angles = inverse_kinematics([0.35, 0, 0.05], "open", math.pi/2)
#joint_angles = inverse_kinematics([0.5, 0, 0.05], "open", 0)
#joint_angles = inverse_kinematics([0.4, 0, 0.15], "open", 0)
joint_angles = inverse_kinematics([0.4, 0.2, 0.15], "open", 0)

point.positions = joint_angles
point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
point.time_from_start = rospy.rostime.Duration(1,0)

trajectory_command.points = [point]

while not rospy.is_shutdown():
    trajectory_command.header.stamp = rospy.Time.now()
    pub.publish(trajectory_command)
    rate.sleep()
