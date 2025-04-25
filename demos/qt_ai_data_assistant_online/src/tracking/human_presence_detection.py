# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT



import time
import copy
from queue import Queue
from threading import Lock
import cv2
import numpy as np

import rospy
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from ultralytics import YOLO

from kinematics.kinematic_interface import QTrobotKinematicInterface
from utils.base_node import BaseNode
from utils.logger import Logger

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


class HumanPresenceDetection(BaseNode):
    DEPTH_ASSUMPTION = 1.5
    MICROPHONE_HEIGHT = 0.65
    MIN_HUMAN_FACE_WIDTH = 30 # in pixel: to avoid perople from far distant 
    MAX_KNOWN_FACES = 5
    
    def setup(self, 
              kinematic_interface=None,
              detection_framerate=None,
              external_vad_trigger=False):
        
        self.presence_callbacks = []        
        self.face_rate = rospy.Rate(detection_framerate) if detection_framerate else None        
        self.detected_persons = {}
        self.detected_persons_lock = Lock()

        # use queue for syncing
        self.image_queue = Queue(maxsize=1)
        self.vod_queue = Queue(maxsize=1)


        self.kinematics = kinematic_interface if kinematic_interface else QTrobotKinematicInterface()
        self.vod_filter = TemporalFilter(time_window=2.0,
                                         sample_count=3, 
                                         error_margine=5)

        # using Yolo
        self.yolo_model = YOLO(model="./model/yolov8n-pose.pt", task="pose", verbose=False)

        # create all subscribers
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self._image_callback)
        self.vod_sub =  rospy.Subscriber('/qt_respeaker_app/sound_direction', Int32, self._vod_callback)
        self.image_pub = rospy.Publisher("/human/image/out", Image, queue_size=10)
        if not external_vad_trigger:
            self.vad_sub =  rospy.Subscriber('/qt_respeaker_app/is_speaking', Bool, self._vad_callback)


    def register_callback(self, callback):
        if callback not in self.presence_callbacks:
            self.presence_callbacks.append(callback)

    def unregister_callback(self, callback):
        if callback in self.presence_callbacks:
            self.presence_callbacks.remove(callback)            

    def _image_callback(self, data):  
        if self.paused():
            return
        
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
        if self.paused():
            return

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

        if not self.paused():        
            for callback in self.presence_callbacks:
                callback(persons)


    def _vad_callback(self, msg):
        if self.paused():
            return
        self.on_vad_trigged()


    # def find_face(self, face):
    #     for id, known in self.deep_known_faces.items():
    #         try:
    #             result = DeepFace.verify(
    #                     img1_path = face,
    #                     img2_path = known,
    #                     detector_backend = self.face_detector_backend,
    #                     enforce_detection=True,
    #                     silent=True
    #                     ) 
    #             verified = result['verified']
    #             if verified:
    #                 return id
    #         except Exception as e:
    #             print(e)
    #             pass
        
    #     return None    

    def process(self):                
        if not self.image_queue.empty():
            image = self.image_queue.get() 
            head_pose = self.kinematics.get_head_pos()
            # t = time.time()
            results = self.yolo_model.track(source=image, verbose=False, persist=True)
            # Logger.info(f"Yolo time: {time.time() - t}")
            # Process the results
            self.detected_persons_lock.acquire()                
            self.detected_persons = {}    
            id = 0            
            for result in results:
                index = -1
                for box in result.boxes:
                    index += 1
                    cls = int(box.cls[0])  # Class ID
                    confidence = box.conf[0]  # Confidence score
                    # print(id, confidence)
                    if confidence < 0.6 or cls!= 0:                            
                        continue                        
                    # Extract bounding box coordinates 
                    id = id + 1
                    x, y, x2, y2 = map(int, box.xyxy[0])                        
                    w = x2 - x
                    h = y2 - y                        
                    keypoint = result.keypoints[index]
                    xy = keypoint.xy[0]
                    point = xy[0]     # corresponds to nose position (center of face)
                    px, py = map(int, point)                        
                    xyz = self.kinematics.pixel_to_base([px, py], HumanPresenceDetection.DEPTH_ASSUMPTION, head_pose)
                    face_id = int(box.id[0]) if box.is_track else id
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
                    cv2.putText(image, f"id:{face_id} ({wx}, {wy}, {wz})", (x+5, y+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
                    cv2.circle(image, (px, py), 4, (0, 0, 255), -1)
                
                for keypoint in result.keypoints:
                    # confidence = keypoint.conf[0]  # Confidence score     
                    for xy in keypoint.xy:                            
                        for point in xy:                                
                            x, y = map(int, point)
                            cv2.circle(image, (int(x), int(y)), 2, (0,255,0), -1)
            

            self.detected_persons_lock.release()
            # publish image
            bridge = CvBridge()                
            self.image_pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))  
            
            if self.detected_persons:
                for callback in self.presence_callbacks:
                    callback(self.detected_persons) 

        self.face_rate.sleep()


    def cleanup(self):    
        rospy.loginfo(f"{self.name} is terminating...")
