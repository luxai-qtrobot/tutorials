#!/usr/bin/env python3
import pyaudio
import rospy
import time
from six.moves import queue
from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
from google.api_core import exceptions as gexcp
from qt_gspeech_interface.srv import *

class QTrobotGoogleSpeech(object):
    """QTrobot speech recognition using google cloud service"""

    def __init__(self, prefix):
        self.prefix = prefix
        self.paudio = pyaudio.PyAudio()
        self.stream_buff = queue.Queue()

        # find respeaker mic
        self.device  = self.get_respeaker_device()
        if not self.device:
            rospy.logfatal("could not find Respeack microphone device")
            raise Exception('device')

        # open mic audio device
        self.stream = self.paudio.open(start=False,
                    format=pyaudio.paInt16,
                    input_device_index=self.device['index'],
                    channels=1,
                    rate=16000,
                    frames_per_buffer=int(16000 / 10),
                    stream_callback=self.callback_stream,
                    input=True)

        # init google speech client
        self.client = speech.SpeechClient()

        # start recognize service
        self.gspeech_recognize = rospy.Service(prefix+'/recognize', QTrobotGspeech, self.callback_recognize)

    def __del__(self):
        try:
            self.stream.stop_stream()
            self.stream.close()
            self.paudio.terminate()
        except:
            pass

    """
        Get ReSpeaker microphone device
    """
    def get_respeaker_device(self):
        for i in range(self.paudio.get_host_api_count()):
            try:
                host = self.paudio.get_host_api_info_by_index(i)
                if host["name"]=="ALSA":
                    host_index = host['index']
                    for d in range(host['deviceCount']):
                        device = self.paudio.get_device_info_by_host_api_device_index(host_index, d)
                        if "ReSpeaker" in device['name']:
                            return device
            except:
                return None
        return None

    """
        microphone stream callback
    """
    def callback_stream(self, in_data, frame_count, time_info, status):
        print("received:", len(in_data), frame_count, time_info, status)
        self.stream_buff.put(in_data)
        return None, pyaudio.paContinue


    """
        ros speech recognize callback
    """
    def callback_recognize(self, req):
        print(len(req.options), req.options)
        answer_context = []
        speech_context = None
        for option in req.options:
            if option.strip():
                answer_context.append(option.lower().strip())
        speech_context = [types.SpeechContext(phrases = answer_context)] if len(answer_context) else None
        config = types.RecognitionConfig(
            encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=16000,
            language_code=req.language.strip() if req.language.strip() else "en-US",
            speech_contexts = speech_context)

        streaming_config = types.StreamingRecognitionConfig(config=config, interim_results=True)
        self.stream.start_stream()
        time.sleep(2)
        chunk = self.stream_buff.get()
        data = [chunk]
        print(len(data))
        # Now consume whatever other data's still buffered.
        while True:
            try:
                chunk = self.stream_buff.get(block=False)
                data.append(chunk)
            except queue.Empty:
                break
        print(len(data))
        #request = types.StreamingRecognizeRequest(audio_content=b'...')
        self.stream.stop_stream()
        self.stream_buff.put(None)

        return QTrobotGspeechResponse("hello")


if __name__ == "__main__":
    rospy.init_node('qt_gspeech_service')

    try:
        gspeech = QTrobotGoogleSpeech('/qt_robot/speech')
        rospy.loginfo("qt_gspeech_service is ready!")
        rospy.spin()
        rospy.loginfo("qt_gspeech_service shutdown")
    except Exception as e:
        rospy.logfatal("could not start", e)
