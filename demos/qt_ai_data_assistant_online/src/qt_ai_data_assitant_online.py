#!/usr/bin/env python3

# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import json
import os
import re
from threading import Lock
from asr.speech_recognition_event import SpeechRecogntionEvent
from llm.llm_llamaindex_openai import LLMLamaIndexOpenAI
import rospy
from enum import Enum

from command_interface import CommandInterface

from llm.llm_llamaindex_groq import LLMLamaIndexGroq
from utils.logger import Logger
from vector_store.vector_store_faiss import VectorStoreFAISS

from llm.llm_prompts import ConversationPrompt, WakeupPrompt
from tracking.human_presence_detection import HumanPresenceDetection
from tracking.human_tracking import HumanTracking
from idle_attention import IdleAttention


# from scene_detection import SceneDetection

from paramify.paramify_web import ParamifyWeb
from utils.base_node import BaseNode


class QTAIDataAssistantOnline(ParamifyWeb, BaseNode):
    IDLE_INTERACTION_TIMEOUT = 1 * 60 # seconds

    class InteractionState(Enum):
        IDLE = 1
        LISTENING = 2
        PROCESSING = 3
        RESPONDING = 4
        PAUSED = 5    

    def __init__(self, 
                 config=None,
                 port: int = 6060):        

        # Initialize ParamifyWeb
        ParamifyWeb.__init__(self, config=config, port=port)
        # Initialize BaseNode        
        BaseNode.__init__(self, paused=self.parameters.paused)


    def setup(self):
        # set the default system prompt if it is not set via parameters
        if not self.parameters.role or not self.parameters.role.strip():
            self.set_role(ConversationPrompt['system_role'])

        
        self.chat = None
        self.asr = None
        self.document_loader = None
        self.vector_store_engine = None        
        self.language = self.parameters.lang
        
        # initialize chat engine
        self.chat = self._reset_chat_engine()
    
        self.state = None
        self.state_lock = Lock()
        self.finish = False
        self.hold_on = False 
        self.active_speaker = None
        self.last_interaction_time = rospy.get_time()
        self.command_interface = CommandInterface(self._function_call_response_callback)

        self.vad_enabled = self.command_interface.set_respeaker_param("AGCONOFF", 0)
        self.vad_enabled = self.vad_enabled and self.command_interface.set_respeaker_param("AGCGAIN", 40)

        
        self.human_detector = HumanPresenceDetection(
            setup_kwargs={
                'detection_framerate': 5,
                'external_vad_trigger': True
            },
            paused=self.parameters.paused)
            
        self.human_detector.register_callback(self._human_presence_callback)
        self.human_tracker = HumanTracking(human_detector=self.human_detector)
        
        self.idle_attention = IdleAttention(setup_kwargs={'attention_time':5, 'human_tracker':self.human_tracker}, paused=self.parameters.paused)
        
        # self.scene_detector = SceneDetection(setup_kwargs={'detection_framerate': 0.1}, paused=(self.parameters.paused or not self.parameters.enable_scene))
        # self.scene_detector.register_callback(self._scene_derection_callback)


        # get current robot attention pos
        self.robot_attention_pos =  None # self.command_interface.ikin.get_head_pos()      

        # set the tts language/voice (for acapela)  
        ret = self.command_interface.set_languge(self.language, 0, 100)
        Logger.info(f"Setting TTS languge to '{self.language}': {ret}")

        ret = self.command_interface.set_volume(self.parameters.volume)
        Logger.info(f"Setting robot's volume level to '{self.parameters.volume}': {ret}")

    
        Logger.debug(f"\n{self}\n")

        
        self.asr = self._reset_asr_engine(self.language)
        self._reset_interaction()


    def _reset_asr_engine(self, language):
        if self.asr:
            self.asr.terminate()
            del self.asr


        if self.parameters.asr_engine == 'azure':
            from asr.azure_speech_recognition import AzureSpeechRecognition
            self.asr = AzureSpeechRecognition(
                setup_kwargs={
                    'subscription': os.environ.get("AZURE_SUBSCRIPTION_KEY"),
                    'region': os.environ.get("AZURE_REGION"),
                    'languages': [language],
                    'event_callback': self.asr_event_callback
                },
                paused=self.parameters.paused
            )

        elif self.parameters.asr_engine == 'groq':  
            from asr.groq_speech_recognition import GroqSpeechRecognition
            self.asr = GroqSpeechRecognition(
                setup_kwargs={
                    'api_key': os.environ.get('GROQ_API_KEY'),                
                    'language': language.split('_')[0].split('-')[0],  # ISO-639-1 language code 'en-US' to 'en'
                    'event_callback': self.asr_event_callback
                    },
                )

        elif self.parameters.asr_engine == 'google':
            from asr.google_speech_recognition import GoogleSpeechRecognition
            self.asr = GoogleSpeechRecognition(
                setup_kwargs={
                    'language': language,
                    'event_callback': self.asr_event_callback
                    },
                paused=self.parameters.paused
            )
        else:
            raise ValueError(f"Unsupported ASR engine: {self.parameters.asr_engine}")
        
        Logger.info(f"ASR language set to '{language}'")
        return self.asr


    def _reset_chat_engine(self):
        if self.chat:
            self.chat.close()
            del self.chat
        
        # from where to load the documents
        if not self.parameters.disable_rag and not self.document_loader:
            if self.parameters.source == 'local':
                from data_loader.local_files_reader import LocalFilesReader
                self.document_loader = LocalFilesReader(
                    document_path=self.parameters.docs,
                    document_formats=self.parameters.formats,
                    max_num_documents=self.parameters.max_docs
                ) 
            elif self.parameters.source == 'web':
                from data_loader.simple_web_reader import SimpleWebReader
                self.document_loader = SimpleWebReader(
                    page_urls=self.parameters.urls
                )
            else:
                raise ValueError(f"Unsupported document source: {self.parameters.source}")


        # where to store the vector embeddings
        if not self.parameters.disable_rag and not self.vector_store_engine:
            collection_name = "qt_ai_agent_collection_openai" if self.parameters.llm_engine == 'openai' else "qt_ai_agent_collection_groq"            
            current_dir = os.path.dirname(os.path.abspath(__file__))
            print(f"current_dir: {current_dir}")
            self.vector_store_engine = VectorStoreFAISS(
                persist_dir=os.path.join(current_dir, '..', f"data/{collection_name}"),
                embedding_dimension=1536 if self.parameters.llm_engine == 'openai' else 384
            )

            # self.vector_store_engine = VectorStoreAstraDB(
            #     token=os.environ.get('ASTRA_DB_TOKEN'),
            #     endpoint=os.environ.get('ASTRA_DB_ENDPOINT'),
            #     collection_name="qt_ai_agent_collection_openai" if self.parameters.llm_engine == 'openai' else "qt_ai_agent_collection_groq",
            #     embedding_dimension=1536 if self.parameters.llm_engine == 'openai' else 384)

        if self.parameters.llm_engine == 'openai':
            self.chat = LLMLamaIndexOpenAI(
                api_key=os.environ.get('OPENAI_API_KEY'),
                model=self.parameters.llm_model, 
                system_role=self.parameters.role,
                data_loader=self.document_loader,
                vector_store_engine=self.vector_store_engine,  
                disable_rag=self.parameters.disable_rag,
                mem_store_file=self.parameters.mem_store,            
                reload_documents=self.parameters.reload)

        elif self.parameters.llm_engine == 'groq':
            self.chat = LLMLamaIndexGroq(
                api_key=os.environ.get('GROQ_API_KEY'),
                model=self.parameters.llm_model, 
                max_tokens=1024,
                system_role=self.parameters.role,
                data_loader=self.document_loader,
                vector_store_engine=self.vector_store_engine,
                disable_rag=self.parameters.disable_rag,
                mem_store_file=self.parameters.mem_store,            
                reload_documents=self.parameters.reload)
        else: 
            raise ValueError(f"Unsupported LLM engine: {self.parameters.llm_engine}")
        
        return self.chat

    def _function_call_response_callback(self, function, result):
        self._set_state(QTAIDataAssistantOnline.InteractionState.PROCESSING)
        f_name = function.get('command')
        f_id = function.get('fid')
        user = function.get('user')
        if f_name == 'get_datetime':
            Logger.info(f"responding to {f_name} for {user}")
            response, tool_calls = self.chat.respond_to_function_call(f_id, f_name, result, postponed=False)
            self.proccess_response(user, response, tool_calls)

        # elif f_name == 'get_image':
        #     Logger.info(f"responding to {f_name} for {user}")
        #     self.rest_robot_attention(True)
        #     img = self.command_interface.lqueue.get(timeout=1)
        #     response, tool_calls = self.chat.get_response(user, img)
        #     self.proccess_response(user, response, tool_calls)

        elif f_name == 'forget_conversation':
            Logger.info(f"responding to {f_name} for {user}")
            self.chat.clear_memmory()
            
        elif f_name == 'pause_interaction':
            Logger.info(f"responding to {f_name} for {user}")
            self._set_state(QTAIDataAssistantOnline.InteractionState.PAUSED)
            self.hold_on = True            
            self.command_interface.show_emotion("QT/confused")
            self.rest_robot_attention()
            self._reset_interaction()
            
        elif f_name == 'resume_interaction':
            Logger.info(f"responding to {f_name} for {user}")
            self.hold_on = False

        elif f_name == 'set_language':
            # TODO: check if tts is set correctly by checking the 'ret'
            #       use try catch around RivaASR ans test it first to see it set properly
            Logger.info(f"responding to {f_name} for {user}")            
            self._set_language(function.get('code'))
            
        elif f_name in ['look_at_xyz', 'look_at_pixel'] and function.get('duration', 0) == 0:            
            self.robot_attention_pos = self.command_interface.ikin.get_head_pos()
            Logger.info(f"updated robot attention pos to {self.robot_attention_pos}" )

    def _set_language(self, language:str):
            ret = self.command_interface.set_languge(language, 0, 100)        
            Logger.info(f"Setting TTS languge to '{language}': {ret}")
            self._reset_asr_engine(language)            
            confirmation = {"en-US": "Sure!", "en-GB": "Sure!", "ar-AR": "بالتأكيد!", "de-DE": "Sicher!", "es-ES": "¡Claro!", "fr-FR": "Bien sûr!", "hi-IN": "ज़रूर!", "it-IT": "Certo!", "ja-JP": "もちろん!", "ru-RU": "Конечно!", "ko-KR": "물론이야!", "pt-BR": "Claro!", "zh-CN": "当然!"}
            self.command_interface.execute([{"command": "talk", "message": confirmation.get(language, "")}])

    def _set_state(self, state):        
        with self.state_lock:
            Logger.info(f"[State]: {state}")
            self.state = state
        

    def _get_state(self):         
        with self.state_lock:   
            st = self.state        
        return st


    def _scene_derection_callback(self, caption):
        self.chat.update_camera_feed(caption)
        Logger.info("camera feed updated.")

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
        Logger.info("Reseting interaction to idle")
        self.robot_attention_pos = None
        self.human_tracker.untrack()
        self.command_interface.ikin.home(['right_arm', 'left_arm'], False)
        self.idle_attention.start()
        

    def rest_robot_attention(self, sync=False):  
        Logger.debug(f"rest_robot_attention {self.robot_attention_pos}")
        if self.robot_attention_pos:
            self.human_tracker.untrack()
            self.command_interface.ikin._move_part('head', self.robot_attention_pos, sync)


    def asr_event_callback(self, event):
        if event == SpeechRecogntionEvent.RECOGNIZING:
            self.human_detector.on_vad_trigged()
            if self._get_state() != QTAIDataAssistantOnline.InteractionState.LISTENING:
                self._set_state(QTAIDataAssistantOnline.InteractionState.LISTENING)
                self.acknowledge_human()


    def _asr_callback(self, text, language):     
        user_id = self.human_tracker.get_current_person_id()
        Logger.info(f"User {user_id}: {text}")
        self._set_state(QTAIDataAssistantOnline.InteractionState.PROCESSING)
        self.acknowledged = False   
        try:      
            if self.hold_on:
                Logger.info("Interaction paused. say 'start conversation' to restart the concersartion.")            
                resp = self.chat.get_raw_chat(WakeupPrompt['system_role'], text)                 
                if resp.replace('\n', '').replace('.', '').strip().lower()  == 'no':
                    self.command_interface.show_emotion("QT/confused")
                else:
                    self.hold_on = False
                    Logger.info("Interraction restarted.")
                    self._set_state(QTAIDataAssistantOnline.InteractionState.RESPONDING)
                    self.command_interface.execute([{"command": "talk", "message": resp}])
                return                 
            
            # conversation is not paused and we continue            
            response_stream = self.chat.get_stream_response(text=text, user_id=user_id)
            self.proccess_response(text, response_stream)
        except Exception as e:
            rospy.logerr(str(e))
            self.command_interface.execute([{"command": "talk", "message": 'I have encounter some technical issues. please try again.'}])        


    def proccess_response(self, user, response_stream):
        self._set_state(QTAIDataAssistantOnline.InteractionState.RESPONDING)
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
                self._set_state(QTAIDataAssistantOnline.InteractionState.RESPONDING)
                self.command_interface.execute([{"command": "talk", "message": msg}])
        
        if tool_call:
            self._set_state(QTAIDataAssistantOnline.InteractionState.RESPONDING)
            self.command_interface.execute(commands=[tool_call])

        Logger.info('finished executing command')
        self.rest_robot_attention()


    
    def cleanup(self):
        Logger.info("qt_ai_data_assistant_online shutting down...") 
        self.asr.terminate()
        self.human_detector.terminate()
        # self.scene_detector.terminate() if self.scene_detector else None
        self.idle_attention.terminate()
        self.chat.close()
        

    def process(self):
        self._reset_interaction()
        try:
            self._set_state(QTAIDataAssistantOnline.InteractionState.IDLE)
            Logger.info("waiting fo speech command...")
            text, lang = self.asr.recognize_once()
            if text:                        
                self._asr_callback(text, lang)
                self.last_interaction_time = rospy.get_time()                
            if rospy.get_time() - self.last_interaction_time > QTAIDataAssistantOnline.IDLE_INTERACTION_TIMEOUT:
                self.last_interaction_time = rospy.get_time() # to avoid reseting interaction everytime
                self._reset_interaction()
        except Exception as e:
            rospy.logerr(str(e))
    

    def on_disable_rag_set(self, value):
        Logger.info(f"disable_rag param set to {value}")
        self.chat = self._reset_chat_engine()

    def on_enable_scene_set(self, value):
        Logger.info(f"enable_scene param set to {value}")
        # if not value:
        #     self.scene_detector.pause()
        #     return
        # self.chat = self._reset_chat_engine()
        # self.scene_detector.resume()

    def on_lang_set(self, value):        
        Logger.info(f"lang param set to {value}")
        self._set_language(value)

    def on_hold_on_set(self, value):
        Logger.info(f"hold_on param set to {value}")
        self.hold_on = value

    def on_paused_set(self, value):        
        Logger.info(f"paused param set to {value}")
        if value:
            self.asr.pause()
            self.human_detector.pause()
            self.idle_attention.pause()
            self.pause()
        else: 
            self.asr.resume()
            self.human_detector.resume()
            self.idle_attention.resume()
            self.resume()        

    def on_volume_set(self, value):           
        Logger.info(f"volume param set to {value}")         
        status = self.command_interface.set_volume(value)
        if not status:
            rospy.logwarn(f"Could not set robot's volume level to '{value}'")


# main 
if __name__ == '__main__':

    
    rospy.init_node('qt_ai_data_assistant_online')
    Logger.info("starting...")

    current_dir = os.path.dirname(os.path.abspath(__file__))   
    demo = QTAIDataAssistantOnline(config=os.path.join(os.path.dirname(current_dir), "config", "default.yaml"))

    Logger.info(f"qt_ai_data_assistant_online started (Paused: {demo.parameters.paused})")
    rospy.spin() 
    # terminating     
    demo.terminate()

