#!/usr/bin/env python3

import json 
import cv2
import numpy as np
from deepface import DeepFace

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class QTDeepFace:    
    def __init__(self, actions=['emotion']): # actions : ['age', 'gender', 'race', 'emotion'])     
        self.actions = actions   
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/face_recognition/out", Image, queue_size=1)
        # self.face_pub = rospy.Subscriber("/qt_deep_face/faces", Faces, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.image_callback)        
        self.faces = None

    def image_callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        rows, cols, channels = cv_image.shape
        faces = DeepFace.analyze(np.array(cv_image), actions = self.actions, enforce_detection=False)

        
        for face in faces:
            face_confidence = face.get('face_confidence', 0.0)
            if face_confidence == 0.0:
                continue
            
            emotion = face.get('emotion', {})
            dominant_emotion = face.get('dominant_emotion')
            region = face.get('region', {})
            x = region.get('x', 0)
            y = region.get('x', 0)
            h = region.get('h', 0)
            w = region.get('w', 0)
            eye_l = region.get('left_eye', [])
            eye_r = region.get('right_eye', [])

            print(dominant_emotion)
            print(region)
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0,255,0), 2)

            # x = int(rect[0]*cols)
            # y = int(rect[1]*rows)
            # w = int(rect[2]*cols)
            # h = int(rect[3]*rows)
            # #cv2.putText(cv_image, "Gender:", (x, y+h+10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), lineType=cv2.LINE_AA)
            # cv2.putText(cv_image, "Gender: %s" % face.gender, (x, y+h+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)
            # cv2.putText(cv_image, "Age: %d" % face.age_years, (x, y+h+40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)

            # # Neutral
            # cv2.putText(cv_image, "Neutral:", (x, y+h+60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)
            # cv2.rectangle(cv_image, (x+80,y+h+50),
            #                         (x+80+int(face.emotion_neutral*100), y+h+10+50), (0,255,0), cv2.FILLED)
            # cv2.rectangle(cv_image, (x+80,y+h+50),
            #                         (x+80+100, y+h+10+50), (255,255,255), 1)
            # # Angry
            # cv2.putText(cv_image, "Angry:", (x, y+h+80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)
            # cv2.rectangle(cv_image, (x+80,y+h+70),
            #                         (x+80+int(face.emotion_angry*100), y+h+10+70), (0,255,0), cv2.FILLED)
            # cv2.rectangle(cv_image, (x+80,y+h+70),
            #                         (x+80+100, y+h+10+70), (255,255,255), 1)

            # # Happy
            # cv2.putText(cv_image, "Happy:", (x, y+h+100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)
            # cv2.rectangle(cv_image, (x+80,y+h+90),
            #                         (x+80+int(face.emotion_happy*100), y+h+10+90), (0,255,0), cv2.FILLED)
            # cv2.rectangle(cv_image, (x+80,y+h+90),
            #                         (x+80+100, y+h+10+90), (255,255,255), 1)

            # cv2.putText(cv_image, "Surprise:", (x, y+h+120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, lineType=cv2.LINE_AA)
            # cv2.rectangle(cv_image, (x+80,y+h+110),
            #                         (x+80+int(face.emotion_surprise*100), y+h+10+110), (0,255,0), cv2.FILLED)
            # cv2.rectangle(cv_image, (x+80,y+h+110),
            #                         (x+80+100, y+h+10+110), (255,255,255), 1)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(str(e))


if __name__ == '__main__':    
    rospy.init_node('qt_deep_face')        
    dp = QTDeepFace(['emotion'])
    rospy.loginfo("qt_deep_face started!")
    rospy.spin()
    rospy.loginfo("qt_deep_face is shutting down...")    
