#!/usr/bin/env python

import math

def forward_kinematics(joint_angles):
    ua_link = 0.2
    fa_link = 0.25
    tcp_link = 0#0.175
    z_offset = 0.075

    x = math.sin(joint_angles[1]) * ua_link + math.sin(joint_angles[1] + joint_angles[2]) * fa_link
    y = 0
    z = z_offset + math.cos(joint_angles[1]) * ua_link + math.cos(joint_angles[1] + joint_angles[2]) * fa_link

    return (x,y,z)


joint_angles = [0.0, 0.29895654509873404, 1.5395412382954012, 6.550886763778825, 0, 0]
print(forward_kinematics(joint_angles))

