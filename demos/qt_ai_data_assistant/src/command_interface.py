# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import queue
import rospy
from concurrent.futures import ThreadPoolExecutor, wait
from qt_robot_interface.srv import *
from datetime import datetime

# import cv2
# import base64
# from std_msgs.msg import Bool, Int32, ColorRGBA
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

from kinematics.kinematic_interface import QTrobotKinematicInterface

try:
    from qt_respeaker_app.srv import tuning_set, tuning_get
except:
    pass


class CommandInterface(object):
    """command interface"""

    def __init__(self, function_call_response_callback=None):
        # add the available command interfaces 
        self.interfaces = {
            "look_at_xyz": self._cmd_look_at_xyz,
            "look_at_pixel" : self._cmd_look_at_pixel,
            "point_at_pixel": self._cmd_point_at_pixel,            
            "point_at_xyz": self._cmd_point_at_xyz,
            "talk": self._cmd_talk,            
            "pause_interaction": self._cmd_pause_interaction,
            "resume_interaction": self._cmd_resume_interaction,
            "forget_conversation": self._cmd_forget_conversation,       
            "get_datetime": self._cmd_get_datetime,
            "set_language": self._cmd_set_language,
            # "get_image": self._cmd_get_image,
        }        

        self.function_call_response_callback = function_call_response_callback
        self.ikin = QTrobotKinematicInterface()
        # self.bridge = CvBridge()

        # create a publishers        
        self.talkText = rospy.ServiceProxy('/qt_robot/behavior/talkText', behavior_talk_text)
        self.speechConfig = rospy.ServiceProxy('/qt_robot/speech/config', speech_config)
        self.emotionShow = rospy.ServiceProxy('/qt_robot/emotion/show', emotion_show)
        # self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self._image_callback)
        self.lqueue = queue.LifoQueue(maxsize=1)
        self.ikin.home(['head', 'right_arm', 'left_arm'], True)


    def execute(self, commands, should_wait=True):
        print(f'excecuting', commands)        
        futures = []
        with ThreadPoolExecutor() as executor:               
            for command in commands:
                cmd = command.get('command', None)                
                try:
                    callback = self.interfaces[cmd]
                    # callback(command)                
                    futures.append(executor.submit(callback, command))
                except KeyError:
                    rospy.logwarn(f"unknown command {cmd}")
                except Exception as e:
                    rospy.logwarn(f"command execution failed: {str(e)}")
            
            if should_wait:
                # Wait for all futures to complete
                wait(futures)


    # def _image_callback(self, data):
    #     try:
    #         cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #         _, buffer = cv2.imencode(".jpg", cv_image)        
    #         if self.lqueue.full():
    #             self.lqueue.get_nowait()
    #         self.lqueue.put(base64.b64encode(buffer).decode("utf-8"))            
    #     except CvBridgeError as e:
    #         rospy.logerr(str(e))


    # get datetime 
    def _cmd_get_datetime(self, params):
        self.function_call_response_callback(params, str(datetime.now()))
        
    def _cmd_pause_interaction(self, params): 
        if not self.function_call_response_callback:
            rospy.logwarn('_cmd_pause_interaction: function_call_response_callback is None')
            return
        print('_cmd_pause_interaction')
        self.function_call_response_callback(params, None)

    def _cmd_resume_interaction(self, params): 
        if not self.function_call_response_callback:
            rospy.logwarn('_cmd_resume_interaction: function_call_response_callback is None')
            return
        print('_cmd_resume_interaction')
        self.function_call_response_callback(params, None)


    def _cmd_forget_conversation(self, params): 
        if not self.function_call_response_callback:
            rospy.logwarn('_cmd_forget_conversation: function_call_response_callback is None')
            return
        print('_cmd_forget_conversation')
        self.function_call_response_callback(params, None)

    def _cmd_set_language(self, params): 
        if not self.function_call_response_callback:
            rospy.logwarn('_cmd_set_language: function_call_response_callback is None')
            return
        print('_cmd_set_language')
        self.function_call_response_callback(params, None)

    # get image
    # def _cmd_get_image(self, params): 
    #     if not self.function_call_response_callback:
    #         rospy.logwarn('_cmd_get_image: function_call_response_callback is None')
    #         return
    #     print('_cmd_get_image')
    #     img = self.lqueue.get(timeout=1)        
    #     self.function_call_response_callback(params, img)        


   # look command 
    def _cmd_look_at_pixel(self, params):
        uv = params.get('uv')
        duration = params.get('duration', 0)
        print(f'looking at {uv} for duration {duration}')        
        self.ikin.look_at_pixel(uv=uv, depth=2.0, duration=duration, sync=True)
        self.function_call_response_callback(params, None)

    def _cmd_look_at_xyz(self, params):
        xyz = params.get('xyz')
        duration = params.get('duration', 0)
        print(f'looking at {xyz} for duration {duration}')
        self.ikin.look_at_xyz(xyz=xyz, duration=duration, sync=True)
        print("done looking")
        self.function_call_response_callback(params, None)


    def _cmd_point_at_xyz(self, params):
        xyz = params.get('xyz')
        duration = params.get('duration', 0)
        print(f'pointing at {xyz} for duration {duration}')
        self.ikin.point_at_xyz(xyz=xyz, duration=duration)

    def _cmd_point_at_pixel(self, params):
        uv = params.get('uv')
        duration = params.get('duration', 0)
        print(f'pointing at {uv} for duration {duration}')        
        self.ikin.point_at_pixel(uv=uv, depth=2.0, duration=duration)


   # talk command 
    def _cmd_talk(self, params):        
        message = params.get('message', '')        
        self.talkText(message)


   # set language and voice
    def set_languge(self, lang_code, pitch=100, speed=100):        
        status = self.speechConfig(lang_code, pitch, speed)
        return status
        
    def show_emotion(self, emotion: str):
        status = self.emotionShow(emotion)

    def set_respeaker_param(self, param, value): 
        if not tuning_set:
            rospy.logwarn(f"set_respeaker_param: tunning_set interface is not available!")
            return False
        param_setter = rospy.ServiceProxy('/qt_respeaker_app/tuning/set', tuning_set)
        try:
            return param_setter(param, value)            
        except:
            rospy.logwarn(f"set_respeaker_param: could not set param {param} to {value}.")
        return False
