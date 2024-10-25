# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import sys
import time
import os
import random
import copy
from queue import Queue
from threading import Lock, Thread
import cv2
import numpy as np
import rospy
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from deepface import DeepFace


from kinematics.kinematic_interface import QTrobotKinematicInterface


class TemporalFilter:
    def __init__(self, time_window=1.0, sample_count=3, error_margine=5):
        self.time_window = time_window
        self.sample_count = sample_count
        self.error_margine = error_margine 
        self.prev_time = time.time()
        self.prev_value = np.inf
        self.count = 0

    def apply(self, sample):
        should_reset = abs(sample - self.prev_value) > self.error_margine
        should_reset = should_reset or (time.time() - self.prev_time > self.time_window)
        if should_reset:
            self.prev_value = sample
            self.prev_time = time.time()
            self.count = 1
            return None

        self.count += 1
        self.prev_time = time.time()
        if self.count < self.sample_count:
            return None

        return sample


class HumanPresenceDetection:
    DEPTH_ASSUMPTION = 1.5
    MICROPHONE_HEIGHT = 0.65
    MIN_HUMAN_FACE_WIDTH = 30 # in pixel: to avoid perople from far distant 
    MAX_KNOWN_FACES = 5
    
    def __init__(self, kinematic_interface=None, detection_framerate=None, external_vad_trigger=False):
        self.presence_callbacks = []        
        self.face_rate = rospy.Rate(detection_framerate) if detection_framerate else None
        self.detected_persons = {}
        self.detected_persons_lock = Lock()

        # use queue for syncing
        self.image_queue = Queue(maxsize=1)
        self.vod_queue = Queue(maxsize=1)

        # for deepface
        self.deep_known_faces = {}
        self.face_extract_backend = 'retinaface'
        self.face_detector_backend = 'ssd' #'retinaface' # ssd

        self.kinematics = kinematic_interface if kinematic_interface else QTrobotKinematicInterface()
        self.vod_filter = TemporalFilter(time_window=2.0,
                                         sample_count=3, 
                                         error_margine=5)

        # create all subscribers
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self._image_callback)
        self.vod_sub =  rospy.Subscriber('/qt_respeaker_app/sound_direction', Int32, self._vod_callback)
        self.image_pub = rospy.Publisher("/human/image/out", Image, queue_size=10)
        if not external_vad_trigger:
            self.vad_sub =  rospy.Subscriber('/qt_respeaker_app/is_speaking', Bool, self._vad_callback)

        # Start the processing thread
        self.processing_thread = Thread(target=self.process_deepface, daemon=True)        
        self.processing_thread.start()



    def register_callback(self, callback):
        if callback not in self.presence_callbacks:
            self.presence_callbacks.append(callback)

    def unregister_callback(self, callback):
        if callback in self.presence_callbacks:
            self.presence_callbacks.remove(callback)            

    def _image_callback(self, data):        
        try:
            bridge = CvBridge()
            image = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(str(e))
            return 

        try:
            self.image_queue.get_nowait()
        except:
            pass
        finally:
            self.image_queue.put_nowait(image)


    def _vod_callback(self, msg):
        try:
            self.vod_queue.get_nowait()
        except:
            pass
        finally:
            self.vod_queue.put_nowait(msg.data)


    def _get_voice_xyz(self, angle, head_yaw_pos):
        voice_angle = angle - 270 + head_yaw_pos
        theta = np.radians(voice_angle)
        x = HumanPresenceDetection.DEPTH_ASSUMPTION * np.cos(theta)
        y = HumanPresenceDetection.DEPTH_ASSUMPTION * np.sin(theta)
        z = HumanPresenceDetection.MICROPHONE_HEIGHT
        return [x,y,z], voice_angle


    def on_vad_trigged(self):        
        try: 
            vod = self.vod_queue.get(timeout=2)             
        except:
            return 

        # ignore values if not valid data or head is in movement 
        head_yaw_velocity = self.kinematics.joints_state.velocity[self.kinematics.joints_state.name.index("HeadYaw")]
        head_pitch_velocity = self.kinematics.joints_state.velocity[self.kinematics.joints_state.name.index("HeadPitch")]
        head_yaw_pos = self.kinematics.joints_state.position[self.kinematics.joints_state.name.index("HeadYaw")]
        if head_yaw_velocity != 0.0 or head_pitch_velocity != 0.0:
            return 
        
        voice_xyz, voice_angle = self._get_voice_xyz(vod, head_yaw_pos)
        persisit_angle = self.vod_filter.apply(voice_angle)

        voice_event = {            
                'angle' : voice_angle,
                'xyz': voice_xyz,
                'persistent': persisit_angle != None                
            }
        # print(vod, xyz, f"persistenct: {persisit_angle != None}")        
        self.detected_persons_lock.acquire()        
        persons = copy.deepcopy(self.detected_persons)
        self.detected_persons_lock.release()

        min_theta = np.inf
        closest_person_id = None        
        for id, person in persons.items():            
            face = person.get('face')
            if not face:
                continue
            xyz_face = face['xyz']
            theta_face = np.degrees(np.arctan2(xyz_face[1], xyz_face[0]))            
            diff_angle = abs(theta_face - voice_angle)
            if diff_angle < min_theta:
                min_theta = diff_angle
                closest_person_id = id
    

        if closest_person_id:
            persons[closest_person_id]['voice'] = voice_event
            persons[closest_person_id]['confidence'] = 1.0 - (min_theta / 360.0)
        else:
            persons = {}
            persons[0] = {'id': 0, 'voice': voice_event }
                
        for callback in self.presence_callbacks:
            callback(persons)


    def _vad_callback(self, msg):        
        self.on_vad_trigged()


    def find_face(self, face):
        for id, known in self.deep_known_faces.items():
            try:
                result = DeepFace.verify(
                        img1_path = face,
                        img2_path = known,
                        detector_backend = self.face_detector_backend,
                        enforce_detection=True,
                        silent=True
                        ) 
                verified = result['verified']
                if verified:
                    return id
            except Exception as e:
                print(e)
                pass
        
        return None    

    def process_deepface(self):
        while not rospy.is_shutdown():            
            if not self.image_queue.empty():
                image = self.image_queue.get() 
                head_pose = self.kinematics.get_head_pos()
                
                #face detection and alignment
                face_objs = DeepFace.extract_faces(
                    img_path= np.array(image),
                    detector_backend = self.face_extract_backend, 
                    enforce_detection=False,
                    expand_percentage=10)                
                self.detected_persons_lock.acquire()                
                self.detected_persons = {} 

                for face_obj in face_objs:                         
                    confidence = face_obj['confidence']                    
                    area = face_obj['facial_area']
                    x = area['x']
                    y = area['y']
                    w = area['w']
                    h = area['h'] 
                    px = int(x + w/2)
                    py = int(y + h/2)
                                            
                    if confidence < 0.7 or w < HumanPresenceDetection.MIN_HUMAN_FACE_WIDTH: 
                        continue

                    xyz = self.kinematics.pixel_to_base([px, py], HumanPresenceDetection.DEPTH_ASSUMPTION, head_pose)

                    face_id = 0
                    try:
                        roi = image[y:y+h, x:x+w]
                        embedding = DeepFace.represent(img_path=roi, detector_backend=self.face_detector_backend, model_name="VGG-Face")[0]['embedding']
                        face_id = self.find_face(embedding)                        
                        if not face_id:                            
                            if self.deep_known_faces:
                                face_id = int(next(reversed(self.deep_known_faces))) + 1
                            else:
                                face_id = 1
                            # trim the deep_known_faces to MAX_KNOWN_FACES limit
                            if len(self.deep_known_faces) > HumanPresenceDetection.MAX_KNOWN_FACES:
                                first_key = next(iter(self.deep_known_faces))
                                self.deep_known_faces.pop(first_key)
                            self.deep_known_faces[face_id] = embedding
                            continue
                    except Exception as e:                    
                        continue

                    self.detected_persons[face_id] = {
                        'id': face_id,
                        'face':{                                                  
                            'x': int(x),
                            'y': int(y),
                            'w': int(w),
                            'h': int(h),
                            'cx': int(px),
                            'cy': int(py),
                            'xyz': xyz
                        }
                    }

                    # Draw bounding box and label
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0,255,0), 1)
                    wx = int(xyz[0] * 100)
                    wy = int(xyz[1] * 100)
                    wz = int(xyz[2] * 100)
                    cv2.putText(image, f"id:{face_id} ({wx}, {wy}, {wz})", (x+5, y+15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 2)                    
                    cv2.circle(image, (px, py), 4, (0, 0, 255), -1)
                        
                    
                self.detected_persons_lock.release()
                # publish image
                bridge = CvBridge()                
                self.image_pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))  
                
                if self.detected_persons:
                    for callback in self.presence_callbacks:
                        callback(self.detected_persons) 
            if self.face_rate:
                self.face_rate.sleep()
