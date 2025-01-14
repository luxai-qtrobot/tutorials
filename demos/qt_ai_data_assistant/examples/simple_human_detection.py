# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import sys
import os
import json
import rospy

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from human_presence_detection import HumanPresenceDetection


def pretty_print(item):
    print(json.dumps(item, indent=2), "\n")


def on_presence(data):
    faces = data.get('faces')
    voice = data.get('voice')          
    pretty_print(data)


if __name__ == '__main__':    
    rospy.init_node('simple_human_detection')    
    detector = HumanPresenceDetection(
        paused=True,
        setup_kwargs={
            'detection_framerate': 10
            })
    detector.register_callback(on_presence)    
    rospy.spin()
    detector.terminate()
