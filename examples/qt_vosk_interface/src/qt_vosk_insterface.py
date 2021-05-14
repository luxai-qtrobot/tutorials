#!/usr/bin/env python3
import os
import queue
import sounddevice as sd
import vosk
import sys
import time
import rospy
import json

from qt_vosk_interface.srv import *


q = queue.Queue()

def callback(indata, frames, time, status):
    """This is called (from a separate thread) for each audio block."""
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))


class QTrobotVoskSpeech(object):
    """QTrobot speech recognition using google cloud service"""

    def __init__(self, prefix, model_path):
        self.prefix = prefix
        # find respeaker mic
        self.device_index  = self.get_respeaker_device_index()
        if not self.device_index  :
            rospy.logfatal("could not find Respeack microphone device")
            raise Exception('device')

        # open mic audio device
        device_info = sd.query_devices(self.device_index, 'input')
        # soundfile expects an int, sounddevice provides a float:
        self.device_samplerate = int(device_info['default_samplerate'])

        self.model = vosk.Model(model_path)


        # start recognize service
        self.speech_recognize = rospy.Service(prefix+'/recognize', qt_vosk_speech, self.callback_recognize)


    """
        Get ReSpeaker microphone device
    """
    def get_respeaker_device_index(self):
        devices = sd.query_devices()
        index = 0
        for dev in devices:
            if "ReSpeaker" in dev['name']:
                return index
            index = index + 1
        return None


    """
        ros speech recognize callback
    """
    def callback_recognize(self, req):
        print("options:", len(req.options), req.options)
        print("language:", req.language)
        print("timeout:", str(req.timeout))
        timeout = (req.timeout if (req.timeout != 0) else 20)

        with sd.RawInputStream(samplerate=self.device_samplerate, blocksize = 8000, device=self.device_index, dtype='int16', channels=1, callback=callback):
            rec = vosk.KaldiRecognizer(self.model, self.device_samplerate)
            t_start = time.time()
            should_stop = False
            transcript = ''
            while not should_stop:
                data = q.get()
                if rec.AcceptWaveform(data):
                    result = rec.Result()
                    # print(result)
                    jres = json.loads(result)
                    transcript = jres['text']
                    should_stop = True
                else:
                    result = rec.PartialResult()
                    # print(result)
                    jres = json.loads(result)
                    for option in req.options:
                        if option.strip() and option in jres['partial']:
                            transcript = option
                            should_stop = True
                should_stop = should_stop or ((time.time() - t_start) > timeout)

        return qt_vosk_speechResponse(transcript)


if __name__ == "__main__":
    rospy.init_node('qt_vosk_service')

    try:
        gspeech = QTrobotVoskSpeech('/qt_robot/speech', '/home/qtrobot/catkin_ws/src/qt_vosk_interface/data/model')
        rospy.loginfo("qt_vosk_service is ready!")
        rospy.spin()
        rospy.loginfo("qt_vosk_service shutdown")
    except Exception as e:
         rospy.logfatal("could not start")
