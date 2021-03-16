#!/usr/bin/env python

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


if __name__ == '__main__':
    rospy.init_node('demo_attach_links')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    # Link them
    rospy.loginfo("Attaching gripper and red box.")
    req = AttachRequest()
    req.model_name_1 = "mogi_bot"
    req.link_name_1 = "left_finger"
    req.model_name_2 = "red_box"
    req.link_name_2 = "link"

    attach_srv.call(req)
    # From the shell:
    """
    rosservice call /link_attacher_node/attach "model_name_1: 'cube1'
    link_name_1: 'link'
    model_name_2: 'cube2'
    link_name_2: 'link'"
    """
