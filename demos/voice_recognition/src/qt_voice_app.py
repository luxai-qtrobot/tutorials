#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import time
import threading
from hermes_python.hermes import Hermes

import rospy
from std_msgs.msg import String
from qt_gesture_controller.srv import *
from qt_motors_controller.srv import *
from qt_robot_interface.srv import *


talkText = rospy.ServiceProxy('/qt_robot/behavior/talkText', behavior_talk_text)
gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
gestureList = rospy.ServiceProxy('/qt_robot/gesture/list', gesture_list)
gestureSave = rospy.ServiceProxy('/qt_robot/gesture/save', gesture_save)
setControlMode = rospy.ServiceProxy('/qt_robot/motors/setControlMode', set_control_mode)
gestureRecord = rospy.ServiceProxy('/qt_robot/gesture/record', gesture_record)

#gesturePlay_pub = rospy.Publisher('/qt_robot/gesture/play', String, queue_size=10)
emotionShow_pub = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=10)

action_thread = None 


def emotion_intent_callback(hermes, intent_message):
    #hermes.publish_end_session(intent_message.session_id, None)
    print("in emotion_intent_callback")    
    if intent_message.slots:
        for (slot_value, slot) in intent_message.slots.items():
            if slot_value == "ShowEmotion":
                emotion_name = slot[0].slot_value.value.value
                try:
                    print("Showing emotion %s" % "QT/%s"%emotion_name)
                    talkText("This is my %s face." % emotion_name.replace("_", " "))
                    emotionShow_pub.publish("QT/%s"%emotion_name)
                except rospy.ServiceException, e:
                    print "Service call failed: %s." % e
                    talkText("I have a problem for showing emotion %s" % emotion_name)                                    
    else:
        talkText("I did not understand which emotion.")
        emotionShow_pub.publish("QT/confused")
    print("done!")


def play_intent_callback(hermes, intent_message):
    #hermes.publish_end_session(intent_message.session_id, None)
    # action code goes here...
    print("in play_intent_callback")
    if intent_message.slots:
        for (slot_value, slot) in intent_message.slots.items():
            if slot_value == "GastureName":
                gesture_name = slot[0].slot_value.value.value
                try:
                    res = gestureList()
                    if gesture_name not in res.gestures:
                        talkText("I do not know this gesture. But you can always record your gesture. Just ask me, hey Q.T. . record, new gesture!")
                    else:
			if gesture_name != "my":
				gesture_name = "QT/" + gesture_name
                        print("playing gesture '%s'." % gesture_name)
                        res = gesturePlay(gesture_name, 1.0)
                except rospy.ServiceException, e:
                    print "Service call failed: %s." % e
                    talkText("I have a problem for playing gesture %s" % gesture_name)
    else:
        talkText("I did not understand which gesture.")
        emotionShow_pub.publish("QT/confused")
    print("done!")


def record_intent_callback(hermes, intent_message):
    #hermes.publish_end_session(intent_message.session_id, None)
    print("in record_intent_callback")
    talkText("You can start recording your gesture.")
    res = gestureRecord(["right_arm", "left_arm"], True, 0, 0)
    print("done!")


def stop_intent_callback(hermes, intent_message):
    #hermes.publish_end_session(intent_message.session_id, None)
    print("in stop_intent_callback")    
    res = gestureSave("my", "")    
    if not res.status:
        talkText("I have a problem with recording the gesture")            
    setControlMode(["right_arm", "left_arm"], 1)
    talkText("Gesture recorded!")
    print("done!")

def unknown_intent_callback(hermes, intent_message):
    talkText("I do not know this. But you can ask me to play gesture or show emotions.")
    print("done")


def intent_received(hermes, intent_message):
        hermes.publish_end_session(intent_message.session_id, None)
        coming_intent = intent_message.intent.intent_name
        print(intent_message.intent.confidence_score)
        if intent_message.intent.confidence_score < 0.5:
            action_thread = threading.Thread(target=unknown_intent_callback, args=(hermes, intent_message))
            action_thread.start()
            return
        
        print 
        if coming_intent == 'apaikan:Play':
            action_thread = threading.Thread(target=play_intent_callback, args=(hermes, intent_message))
            action_thread.start()            
        elif coming_intent == 'apaikan:Record':
            action_thread = threading.Thread(target=record_intent_callback, args=(hermes, intent_message))
            action_thread.start()                        
        elif coming_intent == 'apaikan:Emotion':
            action_thread = threading.Thread(target=emotion_intent_callback, args=(hermes, intent_message))
            action_thread.start()                                    
        elif coming_intent == 'apaikan:Stop':
            action_thread = threading.Thread(target=stop_intent_callback, args=(hermes, intent_message))
            action_thread.start()
        else:
            action_thread = threading.Thread(target=unknown_intent_callback, args=(hermes, intent_message))
            action_thread.start()
          

def intent_not_recognized(hermes, intent_nr_message):
    print("not recogniozed")
    action_thread = threading.Thread(target=unknown_intent_callback, args=(hermes, intent_nr_message))
    action_thread.start()


if __name__ == "__main__":
    # call the relevant service
    rospy.init_node('qt_voice_interface', disable_signals=True)
    with Hermes('localhost:1883') as h:  
#        h.subscribe_intent("apaikan:Play", play_intent_callback)
#        h.subscribe_intent("apaikan:Record", record_intent_callback)
#        h.subscribe_intent("apaikan:Stop", stop_intent_callback)        
#        h.subscribe_intent("apaikan:Emotion", emotion_intent_callback)                
        h.subscribe_intents(intent_received)
        h.subscribe_intent_not_recognized(intent_not_recognized)        
        h.loop_forever()
        # async mode  using 
        #h.loop_start() rospy.spin() h.loop_stop()
    
