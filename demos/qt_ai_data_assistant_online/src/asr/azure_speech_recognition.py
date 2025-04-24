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
    import azure.cognitiveservices.speech as speechsdk 
except ImportError:
    raise ImportError("azure.cognitiveservices.speech package is not installed. Please install it using `pip install azure-cognitiveservices-speech`.")




class AudioInputStreamCallback(speechsdk.audio.PullAudioInputStreamCallback):
    def __init__(self):
        super().__init__()
        # start microphone stream
        self.microphone_stream = MicrophoneStream(use_vad=True, silence_timeout=None)
        self.microphone_stream.__enter__()  # manually enter the context
        self._stream_iterator = iter(self.microphone_stream)

    def read(self, buffer: memoryview) -> int:        
        try:
            chunk = next(self._stream_iterator)
            if chunk is None:
                return 0
            buffer[:len(chunk)] = chunk
            return len(chunk)
        except StopIteration:
            return 0  # Stream ended
        except Exception as e:
            Logger.warning(f'AudioInputStreamCallback: {str(e)}')
            return 0

    def close(self):
        # Logger.debug(f'AudioInputStreamCallback is closed.')
        self.microphone_stream.__exit__(None, None, None)  # manually exit the context        

    def reset(self):
        # Logger.debug(f'AudioInputStreamCallback is reset.')
        self.microphone_stream.reset()

    def wait_for_voice(self, timeout=None):
        return self.microphone_stream.wait_for_voice(timeout=timeout)


class AzureSpeechRecognition(BaseNode):
    """
    AzureSpeechRecognition is a ROS-compatible speech recognition module using 
    Microsoft's Azure Speech SDK.

    This class captures audio from a ROS audio topic using MicrophoneStream and 
    streams it to Azure for real-time transcription. It supports both single-shot 
    recognition and continuous mode with callback integration.

    === SETUP ===
    Call `setup()` before using the recognizer:

        recognizer.setup(
            subscription='your-azure-subscription-key',
            region='westeurope',
            languages=['en-US'],  # List of supported recognition languages (see azure docs)
            silence_timeout=0.2,
            event_callback=handle_event,
            continuous_recog_callback=handle_text
        )

    === USAGE MODES ===
    1. Single recognition (if implemented):
        text, lang = recognizer.recognize_once()

    2. Continuous recognition:
        # Calls continuous_recog_callback for each utterance

    === EVENTS ===
    You can hook into recognition state transitions using `event_callback`. It receives:
        - SpeechRecogntionEvent.STARTED:       VAD triggered
        - SpeechRecogntionEvent.RECOGNIZING:   Voice is being processed
        - SpeechRecogntionEventRECOGNIZED:    Transcription result available
        - SpeechRecogntionEvent.STOPPED:       End of utterance
        - SpeechRecogntionEvent.CANCELED:      Canceled or errored


    === ATTRIBUTES ===
    - subscription: str           (Azure subscription key)
    - region: str                 (Azure service region)
    - speech_recognition_languages: list[str]
    - silence_timeout: float      (timeout in seconds to end utterance)
    - event_callback: callable
    - continuous_recog_callback: callable
    AzureSpeechRecognition is a ROS-compatible speech recognition module using 
    Microsoft's Azure Speech SDK.

    This class captures audio from a ROS audio topic using MicrophoneStream and 
    streams it to Azure for real-time transcription. It supports both single-shot 
    recognition and continuous mode with callback integration.

    
    === CLEANUP ===
    Always call `terminat()` to stop microphone stream and recognition:
        recognizer.terminate()
    """

    def setup(self, 
              subscription,
              region,
              languages=["en-US"],
              silence_timeout=0.2,          
              event_callback=None,              
              continuous_recog_callback=None):
        
        self.subscription = subscription
        self.region = region
        self.speech_recognition_languages = languages        
        self.event_callback = event_callback    
        self.continuous_recog_callback = continuous_recog_callback        
                            
        # setup the Azure ASR engine        
        if len(self.speech_recognition_languages) > 1:
            self.auto_detect_language = speechsdk.languageconfig.AutoDetectSourceLanguageConfig(languages=self.speech_recognition_languages)            
        else: 
            self.auto_detect_language = None        
        
        self.speech_config = speechsdk.SpeechConfig(
            subscription=subscription, 
            region=region,
            speech_recognition_language=None if self.auto_detect_language else self.speech_recognition_languages[0])

        self.speech_config.set_property(speechsdk.PropertyId.Speech_SegmentationSilenceTimeoutMs, str(silence_timeout*1000))  # set this to higher value for more pause in the speech
        self.speech_config.set_property(speechsdk.PropertyId.Conversation_Initial_Silence_Timeout, "5000")
        self.speech_config.set_property(speechsdk.PropertyId.SpeechServiceConnection_InitialSilenceTimeoutMs, "5000")
        self.audio_input_stream = AudioInputStreamCallback()
        self.audio_config = speechsdk.audio.AudioConfig(stream=speechsdk.audio.PullAudioInputStream(self.audio_input_stream))  
        
        self.speech_recognizer = speechsdk.SpeechRecognizer(speech_config=self.speech_config, 
                                                            audio_config=self.audio_config,
                                                            auto_detect_source_language_config=self.auto_detect_language)

        # Connect callbacks to the events fired by the speech recognizer            
        self.speech_recognizer.recognizing.connect(lambda evt: self._asr_event_callback(SpeechRecogntionEvent.RECOGNIZING))
        self.speech_recognizer.recognized.connect(lambda evt: self._asr_event_callback(SpeechRecogntionEvent.RECOGNIZED))
        self.speech_recognizer.session_started.connect(lambda evt: self._asr_event_callback(SpeechRecogntionEvent.STARTED))
        self.speech_recognizer.session_stopped.connect(lambda evt: self._asr_event_callback(SpeechRecogntionEvent.STOPPED))
        self.speech_recognizer.canceled.connect(lambda evt: self._asr_event_callback(SpeechRecogntionEvent.CANCELED))

        # setup event queue and thread
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
        self.audio_input_stream.reset()                
        if not self.audio_input_stream.wait_for_voice(timeout=10.0):
            return None, None
        result = self.speech_recognizer.recognize_once_async().get()
        if result.reason == speechsdk.ResultReason.RecognizedSpeech:            
            if self.auto_detect_language:
                auto_detect_source_language_result = speechsdk.AutoDetectSourceLanguageResult(result)
                detected_language = auto_detect_source_language_result.language            
            return result.text, detected_language if self.auto_detect_language else self.speech_recognition_languages[0]
        
        if result.reason == speechsdk.ResultReason.NoMatch:            
            return None, None

        if result.reason == speechsdk.ResultReason.Canceled:
            cancellation_details = result.cancellation_details
            Logger.warning(f"Speech Recognition canceled: {cancellation_details.reason}")
            if cancellation_details.reason == speechsdk.CancellationReason.Error:
                raise Exception(f"AzureSpeechRecognition: {cancellation_details.error_details}")                

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
        self.audio_input_stream.close()

    # def cleanup(self):         
    #     Logger.debug(f"{self.name} is terminating..")

        



