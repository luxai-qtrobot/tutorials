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


class MicrophoneStream(object):

    def __init__(self, buffer):
        self.stream_buff = buffer
        self.closed = True

    def __enter__(self):
        self.closed = False
        return self

    def __exit__(self, type, value, traceback):
        self.closed = True
        self.stream_buff.put(None)

    def generator(self):
        while not self.closed:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self.stream_buff.get()
            if chunk is None:
                return
            data = [chunk]
            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self.stream_buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b"".join(data)


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

        # start recognize service
        self.gspeech_recognize = rospy.Service(prefix+'/recognize', QTrobotGspeech, self.callback_recognize)

    def __del__(self):
        try:
            self.stream.close()
            self.paudio.terminate()
        except Exception as e:
            rospy.logfatal("could not close the service", e)

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
        #print("received:", len(in_data), frame_count, time_info, status)
        self.stream_buff.put(in_data)
        return None, pyaudio.paContinue


    """
        ros speech recognize callback
    """
    def callback_recognize(self, req):
        # init google speech client
        self.client = speech.SpeechClient()
        
        self.stream.start_stream()
        print("options:", len(req.options), req.options)
        print("language:", req.language)
        print("timeout:", str(req.timeout))
        speech_context = None
        answer_context = []
        for option in req.options:
            if option.strip():
                answer_context.append(option.lower().strip() if '$' not in option else option.strip())        
                
        if answer_context:
            speech_context = types.SpeechContext(phrases = answer_context)
            config = types.RecognitionConfig(
                encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
                sample_rate_hertz=16000,
                language_code= str(req.language.strip()) if req.language.strip() else "en-US",
                speech_contexts = [speech_context]
            )
        else:
            config = types.RecognitionConfig(
                encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
                sample_rate_hertz=16000,
                language_code= str(req.language.strip()) if req.language.strip() else "en-US"
            )

        streaming_config = types.StreamingRecognitionConfig(config=config, interim_results=True, single_utterance=True)
        with MicrophoneStream(self.stream_buff) as mic:
            audio_generator = mic.generator()
            requests = (
                types.StreamingRecognizeRequest(audio_content=content)
                for content in audio_generator
            )
            try:
                #print(requests)
                responses = self.client.streaming_recognize(streaming_config, requests, timeout=(req.timeout if (req.timeout != 0) else 30))
                #print('responses', responses)
                output = self.validate_response(responses, answer_context)
            except gexcp.DeadlineExceeded as e:
                output = "#TIMEOUT#"
                print("#TIMEOUT#")
        self.stream.stop_stream()
        print("Detected [%s]" % (output))
        return QTrobotGspeechResponse(output)


    """
        looping over google responses
    """
    def validate_response(self, responses, context):
        # to return:
        # output = transcript
        transcript = ""

        #num_chars_printed = 0
        for response in responses:
            if not response.results:
                continue
            result = response.results[0]
            if not result.alternatives:
                continue
            transcript = result.alternatives[0].transcript
            if not result.is_final:
                if context:
                    for option in context::
                        if option == transcript.lower().strip()
                            return transcript
            else:
                 return transcript

        return transcript



if __name__ == "__main__":
    rospy.init_node('qt_gspeech_service')

    try:
        gspeech = QTrobotGoogleSpeech('/qt_robot/speech')
        rospy.loginfo("qt_gspeech_service is ready!")
        rospy.spin()
        rospy.loginfo("qt_gspeech_service shutdown")
    except Exception as e:
        rospy.logfatal("could not start", e)
