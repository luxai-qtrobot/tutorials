# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import os
import sys
import rospy

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))
from riva_speech_recognition_vad import RivaSpeechRecognitionSilero


def asr_evnet_callback(event):
    print(event)

    
def reccognition_callback(text, lang):
    print(f"{lang}: {text}")
    

if __name__ == '__main__':
    rospy.init_node('simple_asr_riva')
    asr = RivaSpeechRecognitionSilero(
        setup_kwargs={
            'event_callback': asr_evnet_callback,
            'use_vad': True,
            'continuous_recog_callback': reccognition_callback
            },
        )
    
    
    # while not rospy.is_shutdown():         
    #     text, lang = asr.recognize_once()
    #     if text:
    #         print(f"{lang}:", text)
    
    input("Press enter to stop...")
    print("stopping...")
    asr.terminate()
        
        