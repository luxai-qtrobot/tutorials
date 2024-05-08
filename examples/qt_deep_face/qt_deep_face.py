#!/usr/bin/env python3

import json 
import cv2
import numpy as np
from deepface import DeepFace

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class QTDeepFace:    
    def __init__(self, actions=['emotion']): # actions : ['age', 'gender', 'race', 'emotion'])     
        self.actions = actions   
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/qt_deep_face/image:o", Image, queue_size=10)
        self.face_pub = rospy.Publisher("/qt_deep_face/faces", String, queue_size=10)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)        
        self.faces = None

    def image_callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(str(e))
            
        # rows, cols, channels = cv_image.shape
        try:
            faces = DeepFace.analyze(np.array(cv_image), actions = self.actions, enforce_detection=True)
        except:
            faces = []

        if not faces: 
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                rospy.logerr(str(e))
            return 

        # we have detected some faces 
        self.face_pub.publish(json.dumps(faces))
        for face in faces:
            face_confidence = face.get('face_confidence', 0.0)
            if face_confidence == 0.0:
                continue
            
            region = face.get('region', {})
            emotions = face.get('emotion', {})
            dominant_emotion = face.get('dominant_emotion')            
            age = face.get('age')
            x = region.get('x', 0)
            y = region.get('y', 0)
            h = region.get('h', 0)
            w = region.get('w', 0)
            eye_l = region.get('left_eye', [])
            eye_r = region.get('right_eye', [])

            print(json.dumps(face, indent=2))
            
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0,255,0), 1)
            
            if eye_l:
                cv2.circle(cv_image, (eye_l[0], eye_l[1]), 2, (50,0,200), 2)
            if eye_r:
                cv2.circle(cv_image, (eye_r[0], eye_r[1]), 2, (50,0,200), 2)

            y_offset = y + h + 10
            x_offset = x + 40
            for key, value in emotions.items():
                cv2.putText(cv_image, f"{key}:", (x, y_offset+2), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,255,0), 1, lineType=cv2.LINE_AA)
                cv2.rectangle(cv_image, (x_offset, y_offset+1), (x_offset+int(value/2.0), y_offset+3), (0,255,0), cv2.FILLED)
                cv2.rectangle(cv_image, (x_offset, y_offset), (x_offset+50, y_offset+4), (255,255,255), 1)
                y_offset = y_offset + 10

            if age:
                cv2.putText(cv_image, f"age: {age}", (x, y_offset+2), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,255,0), 1, lineType=cv2.LINE_AA)
                y_offset = y_offset + 10

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
