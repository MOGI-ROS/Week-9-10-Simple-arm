#!/usr/bin/env python

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


if __name__ == '__main__':
    rospy.init_node('demo_detach_links')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

    # Link them
    rospy.loginfo("Detaching gripper and red box.")
    req = AttachRequest()
    req.model_name_1 = "mogi_arm"
    req.link_name_1 = "left_finger"
    req.model_name_2 = "red_box"
    req.link_name_2 = "link"

    attach_srv.call(req)

    # From command line:
    """
    rosservice call /link_attacher_node/detach "model_name_1: 'cube1'
    link_name_1: 'link'
    model_name_2: 'cube2'
    link_name_2: 'link'"
    """