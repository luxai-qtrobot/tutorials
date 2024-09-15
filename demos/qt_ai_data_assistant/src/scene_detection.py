# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import io
from queue import Queue
from threading import Lock, Thread
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ollama import chat

class SceneDetection:    
    def __init__(self,      
                 model="moondream",
                 prompt="Describe in details what you see. if you see people, also describe how they dressed and what they carry.",
                 detection_framerate=1, 
                 contineous_detection=False):
        
        self.model = model
        self.prompt = prompt
        self.detection_callbacks = []  
        self.detection_framerate = detection_framerate
        self.rate = rospy.Rate(detection_framerate)        
        self.scene_description = None
        self.scene_description_lock = Lock()

        # use queue for syncing
        self.image_queue = Queue(maxsize=1)

        if contineous_detection:
            # Start the processing thread
            self.processing_thread = Thread(target=self.process, daemon=True)        
            self.processing_thread.start()

        # create all subscribers
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self._image_callback)
        self.image_pub = rospy.Publisher("/scene/image/out", Image, queue_size=10)

        print(f"SceneDetection is ready!")

    

    def register_callback(self, callback):
        if callback not in self.detection_callbacks:
            self.detection_callbacks.append(callback)

    def unregister_callback(self, callback):
        if callback in self.detection_callbacks:
            self.detection_callbacks.remove(callback)            


    def _put_text_with_wrapping(self, image, text, position, font, font_scale, color, thickness, max_width):
        # Split the text into words
        words = text.split()
        lines = []
        current_line = words[0]

        for word in words[1:]:
            # Check the width of the current line plus the next word
            if cv2.getTextSize(current_line + ' ' + word, font, font_scale, thickness)[0][0] <= max_width:
                current_line += ' ' + word
            else:
                # If the current line is too wide, start a new line
                lines.append(current_line)
                current_line = word
        # Add the last line
        lines.append(current_line)
        
        overlay = image.copy()
        cv2.rectangle(overlay, (position[0]-5, position[1]-10), (max_width+5, len(lines)*19), (0, 0, 0), -1)
        alpha = 0.5  # Transparency factor
        cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0, image)
                
        # Draw each line on the image
        y = position[1]
        for line in lines:
            cv2.putText(image, line, (position[0], y), font, font_scale, color, thickness)
            y += int(cv2.getTextSize(line, font, font_scale, thickness)[0][1] * 2)  # Move to the next line with some line spacing


    def _image_callback(self, data):        
        try:
            bridge = CvBridge()
            image = bridge.imgmsg_to_cv2(data, "bgr8")
            # publish image           
            image2 = image.copy() 
            if self.scene_description:
                self._put_text_with_wrapping(image2, self.scene_description, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, image2.shape[1] - 10)                
            self.image_pub.publish(bridge.cv2_to_imgmsg(image2, "bgr8"))
            
        except CvBridgeError as e:
            rospy.logerr(str(e))
            return 
        
        try:
            self.image_queue.get_nowait()
        except:
            pass
        finally:
            self.image_queue.put_nowait(image)


    def update_prompt(self, prompt):
        self.prompt = prompt


    def get_scene_description(self):
        self.scene_description_lock.acquire()
        desc = self.scene_description
        self.scene_description_lock.release()
        return desc


    def process(self):
        while not rospy.is_shutdown():            
            if not self.image_queue.empty():
                image = self.image_queue.get()                             
                caption = self._query_moondream(image, self.prompt)
                if caption:
                    self.scene_description_lock.acquire()
                    self.scene_description = caption
                    self.scene_description_lock.release()
                    for callback in self.detection_callbacks:
                        callback(caption)                 
            # self.rate.sleep()
            rospy.sleep(1.0/self.detection_framerate)


    def query(self, prompt: str) -> str:
        try:
            image = self.image_queue.get(timeout=3)
            return self._query_moondream(image, prompt)
        except Exception as e:
            rospy.logerr(f"OllamaSceneDetection: {str(e)}")
        return None
    
    def _query_moondream(self, image, prompt):
        _, buffer = cv2.imencode(".jpg", image)
        messages = [{            
            "role": "user",
            "content": prompt,
            "images": [io.BytesIO(buffer)]
            }]
        response = chat(model=self.model, messages=messages)
        return response['message']['content']


