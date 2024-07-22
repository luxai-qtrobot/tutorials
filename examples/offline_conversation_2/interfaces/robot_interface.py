import rospy
from concurrent.futures import ThreadPoolExecutor, wait
from qt_robot_interface.srv import *


class RobotInterface(object):
    """robot interface"""

    def __init__(self):
        # add the available command interfaces 
        self.interfaces = {
            "talk": self.talk,
        }        
        # create a publishers
        self.talkText = rospy.ServiceProxy('/qt_robot/behavior/talkText', behavior_talk_text)
        self.speechConfig = rospy.ServiceProxy('/qt_robot/speech/config', speech_config)


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


   # talk command 
    def talk(self, params):        
        message = params.get('message', '')
        print(f'Talking: ', message)
        self.talkText(message)

   # set language and voice
    def set_languge(self, lang_code, pitch=0, speed=0):
        status = self.speechConfig(lang_code, pitch, speed)
        return status
