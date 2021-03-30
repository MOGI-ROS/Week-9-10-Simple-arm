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
#['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_joint', 'left_finger_joint', 'right_finger_joint']
point.positions = [0.0, 0.91, 1.37, -0.63, 0.3, 0.3]
point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
point.time_from_start = rospy.rostime.Duration(1,0)

trajectory_command.points = [point]

while not rospy.is_shutdown():
    pub.publish(trajectory_command)
    rate.sleep()
