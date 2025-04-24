# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import os
import sys
import rospy


sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))
from utils.logger import Logger
from asr.azure_speech_recognition import AzureSpeechRecognition


def asr_evnet_callback(event):
    Logger.info(event)

    
def reccognition_callback(text, lang):
    Logger.info(f"{lang}: {text}")
    


if __name__ == '__main__':
    rospy.init_node('simple_asr_riva')
    asr = AzureSpeechRecognition(
        setup_kwargs={
            'subscription': os.environ.get('AZURE_SUBSCRIPTION_KEY'),
            'region': os.environ.get('AZURE_REGION'),
            'languages': ['en-US'],
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
        
        