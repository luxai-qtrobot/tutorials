#!/usr/bin/env python

'''
 Copyright (C) 2018 LuxAI S.A
 Authors: Ali Paikan
 CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
'''
import sys
import rospy
from std_msgs.msg import Float64MultiArray

if __name__ == '__main__':
    rospy.init_node('qt_motor_command')

    # create a publisher
    right_pub = rospy.Publisher('/qt_robot/right_arm_position/command', Float64MultiArray, queue_size=1)

    # wait for publisher/subscriber connections
    wtime_begin = rospy.get_time()
    while (right_pub.get_num_connections() == 0) :
        rospy.loginfo("waiting for subscriber connections...")
        if rospy.get_time() - wtime_begin > 10.0:
            rospy.logerr("Timeout while waiting for subscribers connection!")
            sys.exit()
        rospy.sleep(1)

    rospy.loginfo("publishing motor commnad...")
    try:
        ref = Float64MultiArray()
        RightShoulderPitch = 90
        RightShoulderRoll = 0
        RightElbowRoll = -70
        ref.data = [RightShoulderPitch, RightShoulderRoll, RightElbowRoll]
        right_pub.publish(ref)
    except rospy.ROSInterruptException:
        rospy.logerr("could not publish motor commnad!")

    rospy.loginfo("motor commnad published")
