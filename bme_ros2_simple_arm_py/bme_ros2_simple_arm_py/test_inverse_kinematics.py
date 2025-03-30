#!/usr/bin/env python3
import math

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

def forward_kinematics(joint_angles):
    '''
    Calculates the TCP coordinates from the joint angles
    :param joint_angles: list, joint angles [j0, j1, j2, j3, ...]
    :return: list, the list of TCP coordinates
    '''
    ua_link = 0.2
    fa_link = 0.25
    tcp_link = 0.175
    z_offset = 0.1

    x = math.cos(joint_angles[0]) * (math.sin(joint_angles[1]) * ua_link + math.sin(joint_angles[1] + joint_angles[2]) * fa_link + math.sin(joint_angles[1] + joint_angles[2] + joint_angles[3]) * tcp_link)
    y = math.sin(joint_angles[0]) * (math.sin(joint_angles[1]) * ua_link + math.sin(joint_angles[1] + joint_angles[2]) * fa_link + math.sin(joint_angles[1] + joint_angles[2] + joint_angles[3]) * tcp_link)
    z = z_offset + math.cos(joint_angles[1]) * ua_link + math.cos(joint_angles[1] + joint_angles[2]) * fa_link + math.cos(joint_angles[1] + joint_angles[2] + joint_angles[3]) * tcp_link

    return [x,y,z]

if __name__ == '__main__':
    joint_angles = inverse_kinematics([0.35, 0, 0.05], "closed", math.pi/2)
    print(forward_kinematics(joint_angles))

    joint_angles = inverse_kinematics([0.5, 0, 0.05], "closed", 0)
    print(forward_kinematics(joint_angles))

    joint_angles = inverse_kinematics([0.4, 0, 0.15], "closed", 0)
    print(forward_kinematics(joint_angles))

    joint_angles = inverse_kinematics([0.4, 0.2, 0.15], "closed", 0)
    print(forward_kinematics(joint_angles))