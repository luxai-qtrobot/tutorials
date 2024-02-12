#!/usr/bin/env python
import rospy
from qt_nuitrack_app.msg import Hands

# Hands message structure
# qt_nuitrack_app/HandInfo[] hands
#   int32 id
#   float32[] right_projection
#   float32[] right_real
#   bool right_click
#   int32 right_pressure
#   float32[] left_projection
#   float32[] left_real
#   bool left_click
#   int32 left_pressure

def hands_callback(msg):
    rospy.loginfo(msg)

def listener():
    hands_sub = rospy.Subscriber('/qt_nuitrack_app/hands', Hands, hands_callback)

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