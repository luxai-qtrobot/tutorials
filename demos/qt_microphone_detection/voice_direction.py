from tuning import Tuning
import usb.core
import usb.util
import time
import sys
import rospy
from std_msgs.msg import Float64MultiArray

# microphone orientation
MICROPHONE_ANGLE_OFFSET = 90

if __name__ == '__main__':

    rospy.init_node('voice_direction', anonymous=True)
    dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
    if not dev:
        rospy.logerr("Cannot establish connection!")
        sys.exit()

    head_pub = rospy.Publisher('/qt_robot/head_position/command', Float64MultiArray, queue_size=1)

    # wait for publisher connections
    wtime_begin = rospy.get_time()
    while (head_pub.get_num_connections() == 0) :
        rospy.loginfo("waiting for publisher connections...")
        if rospy.get_time() - wtime_begin > 10.0:
            rospy.logerr("Timeout while waiting for publisher connection!")
            sys.exit()
        rospy.sleep(1)

    # centering the head
    href = Float64MultiArray(data=[0,0])
    head_pub.publish(href)
    rospy.sleep(1)

    microphone = Tuning(dev)
    while not rospy.is_shutdown():
        if not microphone.is_voice():
            continue
        mic = abs(microphone.direction - 180)
        angle = mic - MICROPHONE_ANGLE_OFFSET
        rospy.loginfo("mic: %d , head: %d" % (mic, angle))
        href.data = [angle, 0]
        head_pub.publish(href)

    rospy.loginfo("shutdowned!")
