#!/usr/bin/env python
import rospy
from qt_nuitrack_app.msg import Faces

# Faces message structure 
# qt_nuitrack_app/FaceInfo[] faces
#     int32 id
#     string gender
#     int32 age_years
#     string age_type
#     float64 emotion_neutral
#     float64 emotion_angry
#     float64 emotion_happy
#     float64 emotion_surprise
#     float64[] rectangle
#     float64[] left_eye
#     float64[] right_eye
#     float64[] angles

def face_callback(msg):
    rospy.loginfo(msg)

def listener():
    face_sub = rospy.Subscriber('/qt_nuitrack_app/faces', Faces, face_callback)

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