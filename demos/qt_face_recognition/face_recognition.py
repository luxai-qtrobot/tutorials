#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from qt_nuitrack_app.msg import Faces, FaceInfo

class image_converter:
  faces = None

  def __init__(self):
    self.image_pub = rospy.Publisher("/face_recognition/out", Image, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.image_callback)
    self.face_sub = rospy.Subscriber("/qt_nuitrack_app/faces", Faces, self.face_callback)

  def face_callback(self, data):
      print("face_callback")
      print(data.faces[0])
      self.faces = data.faces

  def image_callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    # if cols > 60 and rows > 60 :
    if self.faces:
      rect = self.faces[0].rectangle
      cv2.rectangle(cv_image, (int(rect[0]*cols),int(rect[1]*rows)),
                              (int(rect[0]*cols+rect[2]*cols), int(rect[1]*rows+rect[3]*rows)), (0,255,0), 2)
      x = int(rect[0]*cols)
      y = int(rect[1]*rows)
      w = int(rect[2]*cols)
      h = int(rect[3]*rows)
      #cv2.putText(cv_image, "Gender:", (x, y+h+10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), lineType=cv2.LINE_AA)
      cv2.putText(cv_image, "Gender: %s" % self.faces[0].gender, (x, y+h+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)
      cv2.putText(cv_image, "Age: %d" % self.faces[0].age_years, (x, y+h+40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)

      # Neutral
      cv2.putText(cv_image, "Neutral:", (x, y+h+60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)
      cv2.rectangle(cv_image, (x+80,y+h+50),
                              (x+80+int(self.faces[0].emotion_neutral*100), y+h+10+50), (0,255,0), cv2.FILLED)
      cv2.rectangle(cv_image, (x+80,y+h+50),
                              (x+80+100, y+h+10+50), (255,255,255), 1)
      # Angry
      cv2.putText(cv_image, "Angry:", (x, y+h+80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)
      cv2.rectangle(cv_image, (x+80,y+h+70),
                              (x+80+int(self.faces[0].emotion_angry*100), y+h+10+70), (0,255,0), cv2.FILLED)
      cv2.rectangle(cv_image, (x+80,y+h+70),
                              (x+80+100, y+h+10+70), (255,255,255), 1)

      # Happy
      cv2.putText(cv_image, "Happy:", (x, y+h+100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)
      cv2.rectangle(cv_image, (x+80,y+h+90),
                              (x+80+int(self.faces[0].emotion_happy*100), y+h+10+90), (0,255,0), cv2.FILLED)
      cv2.rectangle(cv_image, (x+80,y+h+90),
                              (x+80+100, y+h+10+90), (255,255,255), 1)

      cv2.putText(cv_image, "Surprise:", (x, y+h+120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)
      cv2.rectangle(cv_image, (x+80,y+h+110),
                              (x+80+int(self.faces[0].emotion_surprise*100), y+h+10+110), (0,255,0), cv2.FILLED)
      cv2.rectangle(cv_image, (x+80,y+h+110),
                              (x+80+100, y+h+10+110), (255,255,255), 1)

    # cv2.imshow("Image View", cv_image)
    # cv2.waitKey(1)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

if __name__ == '__main__':
  rospy.init_node('face_recognition', anonymous=True)
  ic = image_converter()  
  
  #cv2.namedWindow("Image")
  #cv2.setWindowProperty("Image", cv2.WND_PROP_AUTOSIZE, cv2.WINDOW_NORMAL)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  #cv2.destroyAllWindows()

