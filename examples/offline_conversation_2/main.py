#!/usr/bin/env python3
import re
import rospy

from llm_prompts import SYS_ROLE
from interfaces.robot_interface import RobotInterface
from interfaces.ollama_interface import ChatWithMemory
from interfaces.asr_interface import RivaSpeechRecognition
from qt_robot_interface.srv import *


class QTChatBot:
    
    # use llama3 for ollama interface
    def __init__(self, model='llama3'):
        self.paused = False
        self.asr_language = "en-US"
        self.tts_voice = "en-US"
        self.chat = ChatWithMemory(model=model, system_role=SYS_ROLE, max_converstion_memmory=5)
        self.robot_interface = RobotInterface()
        self.asr = RivaSpeechRecognition(self.asr_language)

        # set the voice
        rospy.loginfo(f"Setting tts voice to {self.tts_voice}")        
        if not self.robot_interface.set_languge(self.tts_voice):
            rospy.logwarn(f"Failed to set tts voice to {self.tts_voice}")



    
    def _asr_callback(self, text, language):  
        rospy.loginfo(f"User: {text}")
        try:             
            response_stream = self.chat.get_stream_response(text=text)
            self._proccess_response(text, response_stream)
        except Exception as e:
            rospy.logerr(str(e))
            self.robot_interface.execute([{"command": "talk", "message": 'I have encounter some technical issues. please try again.'}])        


    def _proccess_response(self, user, response_stream):
        for msg in response_stream:
            msg = msg.strip()                    
            msg = re.sub(r'[.*\\/#+]', '', msg)
            msg = msg.replace("Ahaha!", "").replace("Ahaha", "")
            if msg.strip():                
                self.robot_interface.execute([{"command": "talk", "message": msg}])            

        rospy.loginfo('finished executing commands')


    def start(self):
        while not rospy.is_shutdown():            
            try:
                rospy.loginfo('Waiting fo speech command...')
                text, lang = self.asr.recognize_once()
                print(text, lang)
                if text:
                    self._asr_callback(text, lang)
            except Exception as e:
                rospy.logerr(str(e))


if __name__ == '__main__':    
    rospy.init_node('offline_conversation_2')
    rospy.loginfo("starting...")
    chatbot = QTChatBot()       
    rospy.loginfo("offline_conversation_2 started!")
    chatbot.start()    
    rospy.loginfo("qt_learning_mate shutting down") 