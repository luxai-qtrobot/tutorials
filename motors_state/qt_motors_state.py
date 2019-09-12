#!/usr/bin/env python

'''
 Copyright (C) 2018 LuxAI S.A
 Authors: Ali Paikan
 CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
'''

import rospy
from sensor_msgs.msg import JointState


def joint_states_callback(msg):
    strmsg = ""
    for i, joint_name in enumerate(msg.name):
        strmsg += "%s: %.2f, " % (joint_name, msg.position[i])
    rospy.loginfo(strmsg)


# main
if __name__ == '__main__':
    # call the relevant service
    rospy.init_node('qt_motors_state')
    rospy.Subscriber('/qt_robot/joints/state', JointState, joint_states_callback)

    rospy.spin()
