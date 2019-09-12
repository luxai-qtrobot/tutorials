#!/usr/bin/env python

'''
 Copyright (C) 2018 LuxAI S.A
 Authors: Ali Paikan
 CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
'''

import rospy
from qt_gesture_controller.srv import *


def print_help():
    print("QTrobot gesture player example")
    print("Usage:")
    print("      qt_motors_gesture  <name>         play a gesture given by its <name>")
    print("")


# main
if __name__ == '__main__':

    # check the params
    if len(sys.argv) < 2:
        print_help()
        sys.exit(1)

    # call the relevant service
    rospy.init_node('qt_motors_gesture')
    try:
        gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
        res = gesturePlay(sys.argv[1], 1.0)
        if not res.status:
            print("Could not play gesture '%s'." % sys.argv[1])
    except rospy.ServiceException, e:
        print "Service call failed: %s." % e
