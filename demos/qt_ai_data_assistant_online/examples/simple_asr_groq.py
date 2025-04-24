# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import os
import sys
import rospy


sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))
from utils.logger import Logger
from asr.groq_speech_recognition import GroqSpeechRecognition


def asr_evnet_callback(event):
    Logger.info(event)

    
def reccognition_callback(text, lang):
    Logger.info(f"{lang}: {text}")
    


if __name__ == '__main__':
    rospy.init_node('simple_asr_riva')
    asr = GroqSpeechRecognition(
        setup_kwargs={
            'api_key': os.environ.get('GROQ_API_KEY'),
            'context_prompt': "QTrobot",
            'event_callback': asr_evnet_callback,            
            'continuous_recog_callback': reccognition_callback
            },
        )
        
    # while not rospy.is_shutdown():         
    #     text, lang = asr.recognize_once()
    #     if text:
    #         Logger.info(f"{lang}:", text)
    
    input("Press enter to stop...")
    Logger.info("stopping...")
    asr.terminate()
        
        