#!/usr/bin/env python3
import random
import time
import concurrent.futures
import asyncio
import rospy
import text2emotion as te
from utils import aimodel
from std_msgs.msg import String
import nltk
from nltk.tokenize import sent_tokenize, word_tokenize
from nltk.sentiment import SentimentIntensityAnalyzer

from qt_robot_interface.srv import *
from qt_gspeech_app.srv import *
# from qt_riva_asr_app.srv import *

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



class QTChatBot():
    """QTrobot talks with you via GPT3 and Google Speech"""

    def __init__(self):
        nltk.download('vader_lexicon')
        self.model_engine = rospy.get_param("/gpt_demo/chatengine/engine", "chatgpt")

        if self.model_engine == 'chatgpt':
            self.aimodel = aimodel.ChatGPT()
        elif self.model_engine == 'davinci3':
            self.aimodel = aimodel.Davinci3() 
        elif self.model_engine == 'fastchat':
            self.aimodel = aimodel.FastChat()
        else:
            raise ValueError(f'{self.model_engine} not supported!')

        self.sia = SentimentIntensityAnalyzer()
        self.finish = False
        self.defaultlanguage = 'en-US'        
        self.error_feedback = "Sorry. It seems I have some technical problem. Please try again."

        # define a ros service and publisher
        self.emotion_pub = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=2)
        self.gesture_pub = rospy.Publisher('/qt_robot/gesture/play', String, queue_size=2)
        self.talkText = rospy.ServiceProxy('/qt_robot/behavior/talkText', behavior_talk_text)
        self.recognizeQuestion = rospy.ServiceProxy('/qt_robot/gspeech/recognize', speech_recognize) 
        # self.recognizeQuestion = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize) 

        # block/wait for ros service
        rospy.wait_for_service('/qt_robot/behavior/talkText')     
        rospy.wait_for_service('/qt_robot/gspeech/recognize')
        # rospy.wait_for_service('/qt_robot/speech/recognize')
        
    def talk(self, text):
        print('QT talking:', text)
        self.talkText(text)        


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
            elif len(words) > 5 and random.choice([0, 1]) == 0:
                self.gesture_pub.publish(random.choice(["talk", "crossed_arm"]))
            self.talk(sentence)      
        # home pos
        self.gesture_pub.publish(random.choice(["QT/neutral"]))

    def think(self):
        if random.choice([0, 1]) == 0:
            self.gesture_pub.publish(random.choice(["QT/angry", "think_right"]))
            self.emotion_pub.publish("QT/confused")
        return True

    def bored(self):
        if random.choice([0, 1]) == 0:
            self.gesture_pub.publish(random.choice(["QT/angry"]))
            self.talk(random.choice(["#THROAT01#","#BREATH01#","#THROAT02#"]))

    def show_sentiment(self, sentiment):
        if sentiment['emotion'] == 'happy':
            self.emotion_pub.publish("QT/happy")
            self.gesture_pub.publish(random.choice(["QT/happy", "QT/monkey"]))
            self.talk(random.choice(["Yeah!","WOW!","Fantastic!", "This is great!"]))
        elif sentiment['emotion'] == 'angry':
            self.emotion_pub.publish("QT/angry")
            self.gesture_pub.publish(random.choice(["QT/angry", "QT/challenge"]))
            self.talk(random.choice(["That's terrible", "That's not good"]))    
        elif sentiment['emotion'] == 'surprise':
            if sentiment['compound'] > 0: 
                self.emotion_pub.publish("QT/surprise")
                self.gesture_pub.publish(random.choice(["QT/surprise", "QT/show_QT"]))
                self.talk(random.choice(["Oh WOW!", "That's amazing!"]))
            else:
                self.emotion_pub.publish("QT/surprise")
                self.gesture_pub.publish(random.choice(["QT/angry", "QT/surprise"]))
                self.talk(random.choice(["Oh no!", "Oh my goodness, no!"]))
        elif sentiment['emotion'] == 'sad':
            self.emotion_pub.publish("QT/sad")    
            self.gesture_pub.publish(random.choice(["QT/sad", "crossed_arm"]))
            self.talk(random.choice(["Oh no!", "No way!", "No, really?"]))
        elif sentiment['emotion'] == 'fear':
            self.emotion_pub.publish("QT/afraid")
            self.gesture_pub.publish(random.choice(["QT/sad", "crossed_arm"]))
            self.talk(random.choice(["Oh gosh!", "Oh, I understand", "That must be worrisome!","Sorry to hear that!"]))
        elif sentiment['emotion'] == 'neutral':
            self.emotion_pub.publish("QT/neutral") 
            self.gesture_pub.publish("QT/neutral")                   

    def get_sentiment(self, sentence):
        response_sia = self.sia.polarity_scores(sentence)
        response_te = te.get_emotion(sentence)
        emotions = [response_te.get("Happy"), response_te.get("Sad"), response_te.get("Angry"), response_te.get("Surprise"), response_te.get("Fear"), response_sia.get("neu")]
        em = max(emotions)
        em_index = emotions.index(em)
        if em_index == 0 and em >= 0.9 and response_sia.get('compound') > 0:
            return {'emotion': 'happy', 'compound':response_sia.get('compound')}
        elif em_index == 1 and em >= 0.9 and response_sia.get('compound') < 0:
            return {'emotion': 'sad', 'compound':response_sia.get('compound')} 
        elif em_index == 2 and em >= 0.9 and response_sia.get('compound') < 0:
            return {'emotion': 'angry', 'compound':response_sia.get('compound')}
        elif em_index == 3 and em >= 0.9 and response_sia.get('compound') > 0:
            return {'emotion': 'surprise', 'compound':response_sia.get('compound')} 
        elif em_index == 3 and em >= 0.9 and response_sia.get('compound') < 0:
            return {'emotion': 'surprise', 'compound':response_sia.get('compound')}
        elif em_index == 4 and em >= 0.9:
            return {'emotion': 'fear', 'compound':response_sia.get('compound')}
        else:
            return {'emotion': 'neutral', 'compound':response_sia.get('compound')}
        
    def refine_sentence(self, text):
        if not text: 
            raise TypeError     
        tokenized = sent_tokenize(text)
        last_sentence = tokenized[-1].strip()        
        is_finished = last_sentence.endswith('.') or last_sentence.endswith('!') or last_sentence.endswith('?')
        if not is_finished:
            tokenized.pop()
            if not tokenized:
                return text
            return ' '.join(tokenized)
        return text
    
    def intro(self):
        response =  self.aimodel.generate("Who are you?")
        print("Intro: ",response)
        self.talk(response) 


    def start(self):
        self.intro()
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
            self.show_sentiment(self.get_sentiment(prompt))
            words = word_tokenize(prompt.lower())
            if 'stop' in words:
                self.gesture_pub.publish(random.choice(["QT/bye"]))
                self.talk("Okay bye!")
                self.finish = True
            response = None
            bs = Synchronizer()
            results = bs.sync([
                (0, lambda: self.aimodel.generate(prompt)),
                (0.5, lambda: self.think()),
            ])
            if isinstance(results[0], bool):                
                response = results[1]
            else:
                response = results[0]   

            if not response:
                response = self.error_feedback

            self.speak(self.refine_sentence(response))
            
        print("qt_gpt_demo_node Stopping!")
        self.finish = False


if __name__ == "__main__":
   
    rospy.init_node('qt_gpt_demo_node')
    rospy.loginfo("qt_gpt_demo_node started!")                               
    speechBot = QTChatBot()
    speechBot.start()
    rospy.spin()
    rospy.loginfo("qt_gpt_demo_node is ready!")    

    
    
    
    
    
