import rospy
from qt_riva_asr_app.srv import *


class RivaSpeechRecognition:

    def __init__(self, language='en-US', detection_timeout=10):
        self.finished = False
        self.language = language
        self.detection_timeout = detection_timeout
        self.asr = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize) 
        rospy.loginfo("RivaSpeechRecognition waiting for '/qt_robot/speech/recognize'...")
        rospy.wait_for_service('/qt_robot/speech/recognize')


    def recognize_once(self):                       
        try:                
            result = self.asr(language=self.language, options='', timeout=self.detection_timeout)            
            if result and result.transcript:
                return result.transcript, self.language
        except Exception as e:
            rospy.logwarn(f"RivaSpeechRecognition: {str(e)}")
        return None, None


    def stop(self):        
        self.finished = True

    def recognize(self, recognition_callback):
        self.finished = False
        while not rospy.is_shutdown() and not self.finished:
            try:                
                text, lang = self.recognize_once()
                if text:
                    recognition_callback(text, lang)
            except Exception as e:
                rospy.logerr(str(e))

