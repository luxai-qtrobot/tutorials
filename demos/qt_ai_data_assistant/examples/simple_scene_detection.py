# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import os
import sys
import rospy
from termcolor import colored 

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))
from scene_detection import SceneDetection

def scene_callback(caption):    
    print(colored(f"{caption}", 'blue'))
    

if __name__ == '__main__':
    rospy.init_node('simple_scene_detection')
    detector = SceneDetection(contineous_detection=True, detection_framerate=0.3)
    detector.register_callback(scene_callback)
    rospy.spin()

    # while not rospy.is_shutdown():
    #     user = input("User: ").strip()
    #     if not user:
    #         continue

    #     if user.lower() == 'exit':
    #         break        
                
    #     response = detector.query(user)
    #     print(colored(f"{response}", 'blue'))


