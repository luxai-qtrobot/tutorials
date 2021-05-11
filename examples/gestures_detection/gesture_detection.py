#!/usr/bin/env python
import rospy
from qt_nuitrack_app.msg import Gestures

# Gestures message structure
# qt_nuitrack_app/GestureInfo[] gestures
#     int32 id
#     string name

def gesture_callback(msg):
    rospy.loginfo(msg)

def listener():
    gesture_sub = rospy.Subscriber('/qt_nuitrack_app/gestures', Gestures, gesture_callback)

if __name__ == '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    listener()
    try:
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")