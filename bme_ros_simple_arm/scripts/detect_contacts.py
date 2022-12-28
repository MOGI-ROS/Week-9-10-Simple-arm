#!/usr/bin/env python3

import rospy

from gazebo_msgs.msg import ContactsState

def get_contacts (msg):
    if (len(msg.states) == 0):
        rospy.loginfo("No contacts were detected!")
    else:
        if 'left_finger' in msg.states[0].collision1_name:
            rospy.loginfo("Collision detected with %s." % msg.states[0].collision2_name.split("::")[0])
        elif 'left_finger' in msg.states[0].collision2_name:
            rospy.loginfo("Collision detected with %s." % msg.states[0].collision1_name.split("::")[0])
        else:
            rospy.loginfo("Unknown collision")



rospy.init_node('gps_waypoint_follower')

sub_contacts = rospy.Subscriber ('/contact_vals', ContactsState, get_contacts)

rospy.spin()