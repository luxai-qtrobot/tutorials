#!/usr/bin/env python3

# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import time
import json
import os
import re
import argparse
from threading import Lock
import rospy
from enum import Enum
from termcolor import colored

from command_interface import CommandInterface
from llamaindex_interface import ChatWithRAG
from llm_prompts import ConversationPrompt, WakeupPrompt
from human_presence_detection import HumanPresenceDetection
from human_tracking import HumanTracking
from idle_attention import IdleAttention

from riva_speech_recognition_vad import RivaSpeechRecognitionSilero
from scene_detection import SceneDetection

from paramify.paramify_web import ParamifyWeb

def pretty_print(item):
    print(json.dumps(item, indent=2))



class QTAIDataAssistant(ParamifyWeb):
    IDLE_INTERACTION_TIMEOUT = 1 * 60 # seconds

    class InteractionState(Enum):
        IDLE = 1
        LISTENING = 2
        PROCESSING = 3
        RESPONDING = 4
        PAUSED = 5    

    def __init__(self, config):
        super().__init__(config, port=5001)

        self.chat = ChatWithRAG(model=self.parameters.llm,
                                system_role=ConversationPrompt['system_role'],
                                document_path=self.parameters.docs,
                                max_num_documents=self.parameters.max_docs,
                                document_formats=self.parameters.formats,
                                scene_procesing=self.parameters.enable_scene,
                                disable_rag=self.parameters.disable_rag,
                                mem_store_file=self.parameters.mem_store)
        self.language = self.parameters.lang
        self.state = None
        self.state_lock = Lock()
        self.finish = False
        self.paused = False 
        self.active_speaker = None
        self.last_interaction_time = rospy.get_time()
        self.command_interface = CommandInterface(self._function_call_response_callback)

        self.vad_enabled = self.command_interface.set_respeaker_param("AGCONOFF", 0)
        self.vad_enabled = self.vad_enabled and self.command_interface.set_respeaker_param("AGCGAIN", 40)

        self.asr = RivaSpeechRecognitionSilero(
            language=self.language,
            use_vad=self.vad_enabled,
            event_callback=self.asr_event_callback
            )
        self.human_detector = HumanPresenceDetection(detection_framerate=5, external_vad_trigger=True)
        self.human_detector.register_callback(self._human_presence_callback)
        self.human_tracker = HumanTracking(human_detector=self.human_detector)
        self.idle_attention = IdleAttention(attention_time=5, human_tracker=self.human_tracker)
        
        self.scene_detector = SceneDetection(contineous_detection=True, detection_framerate=0.1) if self.parameters.enable_scene else None
        if self.scene_detector:
            self.scene_detector.register_callback(self._scene_derection_callback)



        # get current robot attention pos
        self.robot_attention_pos =  None # self.command_interface.ikin.get_head_pos()      

        # set the tts language/voice (for acapela)  
        ret = self.command_interface.set_languge(self.language, 0, 100)
        rospy.loginfo(f"Setting TTS languge to '{self.language}': {ret}")

        print("")
        print(colored(self, 'green'))
        print("")


    def _function_call_response_callback(self, function, result):
        self._set_state(QTAIDataAssistant.InteractionState.PROCESSING)
        f_name = function.get('command')
        f_id = function.get('fid')
        user = function.get('user')
        if f_name == 'get_datetime':
            rospy.loginfo(f"responding to {f_name} for {user}")
            response, tool_calls = self.chat.respond_to_function_call(f_id, f_name, result, postponed=False)
            self.proccess_response(user, response, tool_calls)

        # elif f_name == 'get_image':
        #     rospy.loginfo(f"responding to {f_name} for {user}")
        #     self.rest_robot_attention(True)
        #     img = self.command_interface.lqueue.get(timeout=1)
        #     response, tool_calls = self.chat.get_response(user, img)
        #     self.proccess_response(user, response, tool_calls)

        elif f_name == 'forget_conversation':
            rospy.loginfo(f"responding to {f_name} for {user}")
            self.chat.clear_memmory()
            
        elif f_name == 'pause_interaction':
            rospy.loginfo(f"responding to {f_name} for {user}")
            self._set_state(QTAIDataAssistant.InteractionState.PAUSED)
            self.paused = True            
            self.command_interface.show_emotion("QT/confused")
            self.rest_robot_attention()
            self._reset_interaction()
            
        elif f_name == 'resume_interaction':
            rospy.loginfo(f"responding to {f_name} for {user}")
            self.paused = False

        elif f_name == 'set_language':
            # TODO: check if tts is set correctly by checking the 'ret'
            #       use try catch around RivaASR ans test it first to see it set properly
            rospy.loginfo(f"responding to {f_name} for {user}")            
            language = function.get('code')
            ret = self.command_interface.set_languge(language, 0, 100)
            rospy.loginfo(f"Setting TTS languge to '{language}': {ret}")
            self.asr.stop()
            self.asr = RivaSpeechRecognitionSilero(language=language, use_vad=self.vad_enabled, event_callback=self.asr_event_callback)
            rospy.loginfo(f"Riva ASR set to '{language}'")
            confirmation = {"en-US": "Sure!", "en-GB": "Sure!", "ar-AR": "بالتأكيد!", "de-DE": "Sicher!", "es-ES": "¡Claro!", "fr-FR": "Bien sûr!", "hi-IN": "ज़रूर!", "it-IT": "Certo!", "ja-JP": "もちろん!", "ru-RU": "Конечно!", "ko-KR": "물론이야!", "pt-BR": "Claro!", "zh-CN": "当然!"}
            self.command_interface.execute([{"command": "talk", "message": confirmation.get(language, "")}])
            
        elif f_name in ['look_at_xyz', 'look_at_pixel'] and function.get('duration', 0) == 0:            
            self.robot_attention_pos = self.command_interface.ikin.get_head_pos()
            rospy.loginfo(f"updated robot attention pos to {self.robot_attention_pos}" )


    def _set_state(self, state):        
        with self.state_lock:
            print(colored(f"[State]: {state}", 'cyan'))
            self.state = state
        

    def _get_state(self):         
        with self.state_lock:   
            st = self.state        
        return st


    def _scene_derection_callback(self, caption):
        self.chat.update_camera_feed(caption)
        rospy.loginfo("camera feed updated.")

    def _human_presence_callback(self, persons):
        # if voice and faces:        
        for id, person in persons.items():
            if person.get('voice'):                
                self.active_speaker = person
        
            
    def acknowledge_human(self): 
        t = rospy.get_time()
        while not self.active_speaker and (rospy.get_time() - t < 3):
            rospy.sleep(0.05)
        if not self.active_speaker:
            rospy.logwarn('no active speaker')
            return 
        self.idle_attention.stop()
        self.human_tracker.track(self.active_speaker)


    def _reset_interaction(self):
        rospy.loginfo("Reseting interaction to idle")
        self.robot_attention_pos = None
        self.human_tracker.untrack()
        self.command_interface.ikin.home(['right_arm', 'left_arm'], False)
        self.idle_attention.start()
        

    def rest_robot_attention(self, sync=False):  
        print(f"rest_robot_attention {self.robot_attention_pos}")
        if self.robot_attention_pos:
            self.human_tracker.untrack()
            self.command_interface.ikin._move_part('head', self.robot_attention_pos, sync)


    def asr_event_callback(self, event):
        
        if event == RivaSpeechRecognitionSilero.Event.RECOGNIZING:            
            self.human_detector.on_vad_trigged()
            if self._get_state() != QTAIDataAssistant.InteractionState.LISTENING:
                self._set_state(QTAIDataAssistant.InteractionState.LISTENING)
                self.acknowledge_human()

    def _asr_callback(self, text, language):     
        user_id = self.human_tracker.get_current_person_id()
        rospy.loginfo(f"User {user_id}: {text}")
        self._set_state(QTAIDataAssistant.InteractionState.PROCESSING)
        self.acknowledged = False   
        try:      
            if self.paused:
                rospy.loginfo("Interaction paused. say 'start conversation' to restart the concersartion.")            
                resp = self.chat.get_raw_chat(WakeupPrompt['system_role'], text)                 
                if resp.replace('\n', '').replace('.', '').strip().lower()  == 'no':
                    self.command_interface.show_emotion("QT/confused")
                else:
                    self.paused = False
                    rospy.loginfo("Interraction restarted.")
                    self._set_state(QTAIDataAssistant.InteractionState.RESPONDING)
                    self.command_interface.execute([{"command": "talk", "message": resp}])
                return                 
            
            # conversation is not paused and we continue            
            response_stream = self.chat.get_stream_response(text=text, user_id=user_id)
            self.proccess_response(text, response_stream)
        except Exception as e:
            rospy.logerr(str(e))
            self.command_interface.execute([{"command": "talk", "message": 'I have encounter some technical issues. please try again.'}])        


    def proccess_response(self, user, response_stream):
        self._set_state(QTAIDataAssistant.InteractionState.RESPONDING)
        tool_call = None
        
        for msg in response_stream:
            msg = msg.strip()                    
            try: 
                tool_call = json.loads(msg)                
            except:
                pass 

            if tool_call:
                continue
            msg = re.sub(r'[*\\/#+"]', '', msg)
            msg = msg.replace("QTrobot", "Cutee robot")
            if msg.strip():
                self._set_state(QTAIDataAssistant.InteractionState.RESPONDING)
                self.command_interface.execute([{"command": "talk", "message": msg}])
        
        if tool_call:
            self._set_state(QTAIDataAssistant.InteractionState.RESPONDING)
            self.command_interface.execute(commands=[tool_call])

        rospy.loginfo('finished executing command')
        self.rest_robot_attention()


    
    def stop(self):
        # stop asr 
        self.asr.stop()
        self.chat.close()


    def start(self):
        self._reset_interaction()

        while not rospy.is_shutdown():            
            try:
                self._set_state(QTAIDataAssistant.InteractionState.IDLE)
                rospy.loginfo("waiting fo speech command...")
                text, lang = self.asr.recognize_once()
                if text:                        
                    self._asr_callback(text, lang)
                    self.last_interaction_time = rospy.get_time()                
                if rospy.get_time() - self.last_interaction_time > QTAIDataAssistant.IDLE_INTERACTION_TIMEOUT:
                    self.last_interaction_time = rospy.get_time() # to avoid reseting interaction everytime
                    self._reset_interaction()
            except Exception as e:
                rospy.logerr(str(e))
        self.stop()                


    def on_enable_scene_set(self, value):
        rospy.loginfo(f"enable_scene was updated to {value}")

    def on_disable_rag_set(self, value):
        rospy.loginfo(f"disable_rag was updated to {value}")

    def on_lang_set(self, value):
        rospy.loginfo(f"lang was updated to {value}")


# main 
if __name__ == '__main__':

    current_dir = os.path.dirname(os.path.abspath(__file__))   
    
    rospy.init_node('qt_ai_data_assistant')
    rospy.loginfo("starting...")

    demo = QTAIDataAssistant(f"{current_dir}/config.yaml")

    rospy.loginfo("qt_ai_data_assistants started")
    demo.start()    
    rospy.loginfo("qt_ai_data_assistant shutting down") 

