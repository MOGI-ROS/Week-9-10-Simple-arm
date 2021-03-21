#!/usr/bin/env python

import rospy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node('send_joint_angles')

pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)

controller_name = "arm_controller"
joint_names = rospy.get_param("/%s/joints" % controller_name)

rospy.loginfo("Joint names: %s" % joint_names)

rate = rospy.Rate(10)

trajectory_command = JointTrajectory()

trajectory_command.joint_names = joint_names
trajectory_command.header.stamp = rospy.Time.now()

point = JointTrajectoryPoint()
point.positions = [1.0, 0.5, 0.5, 0.0, 0.0, 0.0]
point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
point.time_from_start = rospy.rostime.Duration(1,0)

trajectory_command.points = [point]

while not rospy.is_shutdown():
    pub.publish(trajectory_command)
    rate.sleep()
