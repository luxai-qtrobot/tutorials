# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import queue
from threading import Thread

from asr.speech_recognition_event import SpeechRecogntionEvent
from asr.microphone_stream import MicrophoneStream
from utils.base_node import BaseNode
from utils.logger import Logger


try:
    from google.cloud import speech
except ImportError:
    raise ImportError("google-cloud-speech package is not installed. Please install it using `pip install google-cloud-speech`.")


class GoogleMicrophoneStream(object):

    def __init__(self):                
        # start microphone stream
        self.microphone_stream = MicrophoneStream(use_vad=True, silence_timeout=None)
        self.microphone_stream.__enter__()  # manually enter the context

    def __enter__(self): 
        return self

    def __exit__(self, type, value, traceback):        
        self.microphone_stream.__exit__(None, None, None)  # manually exit the context        
        
    def generator(self):
        for chunk in self.microphone_stream:
            yield chunk
        
    def get_rate(self):
        return self.microphone_stream.get_rate()
    
    def reset(self):
        self.microphone_stream.reset()

    def wait_for_voice(self, timeout=None):
        return self.microphone_stream.wait_for_voice(timeout=timeout)


class GoogleSpeechRecognition(BaseNode):
    """
    GoogleSpeechRecognition is a ROS-compatible speech recognition module using Groq's Whisper ASR model.

    """

    def setup(self, 
              language='en-US',      # a BCP-47 language tag
              event_callback=None,              
              continuous_recog_callback=None):
        

        self.language_code = language
        self.event_callback = event_callback    
        self.continuous_recog_callback = continuous_recog_callback
                            
        # Initialize the google client
        self.google_microphone_stream = GoogleMicrophoneStream()
        self.client = speech.SpeechClient()
        config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=self.google_microphone_stream.get_rate(),
            model='default',            
            language_code= self.language_code)
        self.streaming_config = speech.StreamingRecognitionConfig(config=config, interim_results=True)



        self.asr_event_queue = queue.Queue(maxsize=1)
        self.asr_event_thread = Thread(target=self._proccess_asr_events, daemon=True)        
        self.asr_event_thread.start()

        

    def _proccess_asr_events(self):
        while not self.terminating():
            try:
                evt = self.asr_event_queue.get(timeout=1)
                if evt and self.event_callback:
                    self.event_callback(evt)
            except Exception as e:
                pass        

    def _asr_event_callback(self, evt):
        try:
            self.asr_event_queue.get_nowait()
        except:
            pass
        finally:
            self.asr_event_queue.put_nowait(evt)



    def recognize_once(self):        
        Logger.debug('waiting for voice activity...')
        self.google_microphone_stream.reset()
        if not self.google_microphone_stream.wait_for_voice(timeout=10.0):
            return None, None
        
        self._asr_event_callback(SpeechRecogntionEvent.STARTED)        
        audio_generator = self.google_microphone_stream.generator()
        requests = (
            speech.StreamingRecognizeRequest(audio_content=content)
            for content in audio_generator
        )
        try:
            # print(requests)
            responses = self.client.streaming_recognize(self.streaming_config, requests)
            # print('responses', responses)            
            for response in responses:
                self._asr_event_callback(SpeechRecogntionEvent.RECOGNIZING)
                if not response.results:
                    continue
                # The `results` list is consecutive. For streaming, we only care about
                # the first result being considered, since once it's `is_final`, it
                # moves on to considering the next utterance.
                result = response.results[0]
                if not result.alternatives:
                    continue
                # Display the transcription of the top alternative.
                transcript = result.alternatives[0].transcript
                if result.is_final:
                    self._asr_event_callback(SpeechRecogntionEvent.RECOGNIZED)
                    self._asr_event_callback(SpeechRecogntionEvent.STOPPED)
                    return transcript, self.language_code
            
        except Exception as e:
            print(type(e))
            Logger.error(f"Error in speech recognition: {e}")      
        
        self._asr_event_callback(SpeechRecogntionEvent.STOPPED)
        return None, None



    def process(self):        
        if self.continuous_recog_callback:
            try:                
                text, lang = self.recognize_once()
                if text:
                    self.continuous_recog_callback(text, lang)
            except Exception as e:                
                Logger.error(f"Error in continuous recognition: {e}")
        else:
            self.terminate_event.wait() # disable continuous recognition if continuous_recog_callback is none!
            

    def interrupt(self):
        Logger.debug(f"{self.name} is interupting..")
        self.google_microphone_stream.__exit__(None, None, None)  # manually exit the context

    # def cleanup(self):         
    #     Logger.debug(f"{self.name} is terminating..")        
        



