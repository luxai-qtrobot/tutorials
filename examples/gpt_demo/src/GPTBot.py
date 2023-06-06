#!/usr/bin/env python3
import random
import time
import concurrent.futures
import asyncio
import rospy
import json
from utils import aimodel
from std_msgs.msg import String
from qt_robot_interface.srv import *
from qt_gspeech_app.srv import *
from nltk.tokenize import sent_tokenize, word_tokenize

class Synchronizer():
    """
    A simple concurrent tasks synchornizer
    """

    def __init__(self, max_workers=5):
        self.loop = self.get_or_create_eventloop()
        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=max_workers)

    def get_or_create_eventloop(self):
        try:
            return asyncio.get_event_loop()
        except RuntimeError as ex:
            if "There is no current event loop in thread" in str(ex):
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                return asyncio.get_event_loop()
            
    def __worker(self, *args):
        delay_exe = args[0][0] 
        func = args[0][1]
        time.sleep(delay_exe)
        return func()

    async def __non_blocking(self, tasks):
        fs = []
        for task in tasks:
            fs.append(self.loop.run_in_executor(self.executor, self.__worker, task))        
        done, pending = await asyncio.wait(fs=fs, return_when=asyncio.ALL_COMPLETED)        
        results = [task.result() for task in done]        
        return results 


    def sync(self, tasks):
        """
        call this function with multiple tasks to run concurrently.
        tasks is a list of (delay, lamda function) tuple. for exmaple: 
        tasks = [ (0, lambda: print("hello")), (3, lambda: print("world")), ...] 
        returns a list of each lamda function return value   
        """
        results = self.loop.run_until_complete(self.__non_blocking(tasks))        
        return results



class QTGPTGspeechBot():
    """QTrobot talks with you via GPT3 and Google Speech"""

    def __init__(self, config_path="/home/qtrobot/code/tutorials/examples/gpt_demo/config/config.json"):
        self.config_path = config_path  
        with open(self.config_path) as f:
            self.config = json.load(f)
        if self.config["chatengine"]["OPENAI_GPT_MODEL"]:
            self.aimodel = aimodel.ChatGPT(self.config)
        else:
            self.aimodel = aimodel.Davinci3(self.config) 
        self.finish = False
        self.defaultlanguage = 'en-US'        
        self.error_feedback = "Sorry. It seems I have some technical problem. Please try again."

        # define a ros service and publisher
        self.emotion_pub = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=2)
        self.gesture_pub = rospy.Publisher('/qt_robot/gesture/play', String, queue_size=2)
        self.talkText = rospy.ServiceProxy('/qt_robot/behavior/talkText', behavior_talk_text)
        self.recognizeQuestion = rospy.ServiceProxy('/qt_robot/gspeech/recognize', speech_recognize) 

        # block/wait for ros service
        rospy.wait_for_service('/qt_robot/behavior/talkText')     
        rospy.wait_for_service('/qt_robot/gspeech/recognize')

    def speak(self, text):
        sentences = sent_tokenize(text)
        closing_words = ["bye","goodbye"]        
        for sentence in sentences:
            words = word_tokenize(sentence.lower())
            if any(endw in closing_words for endw in words):
                print("Bye detected!")
                self.gesture_pub.publish(random.choice(["QT/bye"]))
                self.finish = True                
            elif 'surprise' in words or 'surprised' in words:
                self.gesture_pub.publish(random.choice(["QT/surprise"]))
            elif '?' in words:
                self.gesture_pub.publish(random.choice(["QT/show_tablet"]))
            elif 'yes' in words:
                self.gesture_pub.publish(random.choice(["yes"]))
            elif 'no' in words:
                self.gesture_pub.publish(random.choice(["no"]))
            elif len(words) > 10 and random.choice([0, 1]) == 0:
                self.gesture_pub.publish(random.choice(["talk", "crossed_arm"]))
            print("QTrobot:", sentence)
            self.talkText(sentence)      
        # home pos
        self.gesture_pub.publish(random.choice(["QT/neutral"]))

    def think(self):
        if random.choice([0, 1]) == 0:
            self.gesture_pub.publish(random.choice(["QT/angry", "think_right"]))
            self.emotion_pub.publish(random.choice(["QT/confused", "QT/calming_down"]))
        return True

    def bored(self):
        if random.choice([0, 1]) == 0:
            self.gesture_pub.publish(random.choice(["QT/angry"]))
            self.talkText(random.choice(["#COUGH02","#BREATH01#","#THROAT02#"]))

    def show_sentiment(self, sentiment):
        if sentiment == 'positive':
            self.emotion_pub.publish(random.choice(["QT/happy", "QT/showing_smile", "QT/calming_down"]))
        elif sentiment == 'negative':
            self.emotion_pub.publish(random.choice(["QT/confused", "QT/sad"]))                    

    
    def start(self):
        self.talkText("How can I help you?")
        while not rospy.is_shutdown() and not self.finish:            
            print('listenting...') 
            try:    
                recognize_result = self.recognizeQuestion(language=self.defaultlanguage, options='', timeout=0)                        
                if not recognize_result or not recognize_result.transcript:
                    self.bored()
                    continue
            except:
                continue

            print('Human:', recognize_result.transcript)
            prompt = recognize_result.transcript
            words = word_tokenize(prompt.lower())
            if 'stop' in words:
                self.gesture_pub.publish(random.choice(["QT/bye"]))
                self.talkText("Okay bye!")
                self.finish = True
            response = None
            bs = Synchronizer()
            results = bs.sync([
                (0, lambda: self.aimodel.generate(prompt)),
                (0.5, lambda: self.think())
            ])
            if isinstance(results[0], bool):                
                response = results[1]
            else:
                response = results[0]   

            if not response:
                response = self.error_feedback

            self.speak(response)

        print("QTGptBot_node Stopping!")
        self.finish = False


if __name__ == "__main__":
   
    rospy.init_node('QTGptBot_node')
    rospy.loginfo("QTGptBot_node started!")                               
    speechBot = QTGPTGspeechBot()
    speechBot.start()
    rospy.spin()
    rospy.loginfo("QTGptBot_node is ready!")    

    
    
    
    
    
