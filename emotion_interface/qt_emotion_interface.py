#!/usr/bin/env python

'''
 Copyright (C) 2018 LuxAI S.A
 Authors: Ali Paikan
 CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
'''

import sys
import rospy
from std_msgs.msg import String
from qt_robot_interface.srv import *
from qt_gesture_controller.srv import *

# the following activities will run in parallel on the robot
# with no execution order
def publishAllCuncurent():
    audioPlay_pub.publish("Qt2.wav")
    speechSay_pub.publish("Hello! This is QT talking using text to speech")
    gesturePlay_pub.publish("QT/happy")
    emotionShow_pub.publish("QT/happy")
    behaviorTalkText_pub.publish("I am QT robot! ")
    behaviorTalkAudio_pub.publish("Qt3.wav")

# the following activities will run in sequence on the robot
# one after another
def callAllSequence():
    audioPlay("Qt2.wav", "")
    speechSay("Hello! This is QT talking using text to speech")
    gesturePlay("QT/happy", 0)
    emotionShow("QT/happy")
    behaviorTalkText("I am QT robot!")
    behaviorTalkAudio("Qt3.wav", "");


if __name__ == '__main__':
    rospy.init_node('python_qt_example')

    # create a publisher
    speechSay_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
    audioPlay_pub = rospy.Publisher('/qt_robot/audio/play', String, queue_size=10)
    emotionShow_pub = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=10)
    gesturePlay_pub = rospy.Publisher('/qt_robot/gesture/play', String, queue_size=10)
    behaviorTalkText_pub = rospy.Publisher('/qt_robot/behavior/talkText', String, queue_size=10)
    behaviorTalkAudio_pub = rospy.Publisher('/qt_robot/behavior/talkAudio', String, queue_size=10)

    # wait for publisher/subscriber connections
    wtime_begin = rospy.get_time()
    while (speechSay_pub.get_num_connections() == 0 or
           audioPlay_pub.get_num_connections() == 0 or
           emotionShow_pub.get_num_connections() == 0 or
           gesturePlay_pub.get_num_connections() == 0 or
           behaviorTalkText_pub.get_num_connections() == 0 or
           behaviorTalkAudio_pub.get_num_connections() == 0 ) :

        rospy.loginfo("waiting for subscriber connections")
        if rospy.get_time() - wtime_begin > 5.0:
            rospy.logerr("Timeout while waiting for subscribers connection!")
            sys.exit()
        rospy.sleep(1)

    # create some service clients
    audioPlay = rospy.ServiceProxy('/qt_robot/audio/play', audio_play)
    speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
    gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
    emotionShow = rospy.ServiceProxy('/qt_robot/emotion/show', emotion_show)
    behaviorTalkText = rospy.ServiceProxy('/qt_robot/behavior/talkText', behavior_talk_text)
    behaviorTalkAudio = rospy.ServiceProxy('/qt_robot/behavior/talkAudio', behavior_talk_audio)

    # wait for some services and connection to subscribers
    rospy.loginfo("waiting for services connections")
    rospy.wait_for_service('/qt_robot/gesture/play')
    rospy.wait_for_service('/qt_robot/emotion/show')
    rospy.wait_for_service('/qt_robot/audio/play')
    rospy.wait_for_service('/qt_robot/speech/say')
    rospy.wait_for_service('/qt_robot/behavior/talkText')
    rospy.wait_for_service('/qt_robot/behavior/talkAudio')

    rospy.loginfo("ready...")
    try:
        callAllSequence()
        publishAllCuncurent()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
