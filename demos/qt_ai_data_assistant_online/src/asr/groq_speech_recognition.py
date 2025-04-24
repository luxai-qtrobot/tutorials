# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import io
import queue
from threading import Thread
import wave


from asr.speech_recognition_event import SpeechRecogntionEvent
from asr.microphone_stream import MicrophoneStream
from utils.base_node import BaseNode
from utils.logger import Logger
from groq import Groq


class GroqSpeechRecognition(BaseNode):
    """
    GroqSpeechRecognition is a ROS-compatible speech recognition module using Groq's Whisper ASR model.

    This class listens to microphone input via a ROS audio topic, performs voice activity detection (VAD),
    segments voice into utterances, and sends them to the Groq Whisper API for transcription.

    It can be used in both:
      - single-shot mode (`recognize_once`)
      - continuous callback mode (`process` with a `continuous_recog_callback`)

    === SETUP ===
    Call `setup()` before using the recognizer:
    
        recognizer.setup(
            api_key='your-groq-api-key',
            language='en',  # ISO-639-1 language code (e.g., 'en', 'fr', etc.)
            context_prompt='robotics domain',
            silence_timeout=1.0,
            event_callback=handle_event,
            continuous_recog_callback=handle_text
        )

    === USAGE MODES ===
    1. Single recognition:
        text, lang = recognizer.recognize_once()

    2. Continuous recognition (e.g., in `spin()`):
        # triggers callback with recognized text

    === EVENTS ===
    You can hook into state transitions using the `event_callback`, which receives:
        - SpeechRecogntionEvent.STARTED:       recognition started and waiting for VAD trigger
        - SpeechRecogntionEvent.RECOGNIZING:   Receiving audio
        - SpeechRecogntionEvent.RECOGNIZED:    Transcription complete
        - SpeechRecogntionEvent.STOPPED:       recognition stoped
        - SpeechRecogntionEvent.CANCELED:      Not currently used

    === ATTRIBUTES ===
    - language_code: str         (ISO-639-1 language code, default 'en')
    - context_prompt: str        (domain-specific text to help ASR to recognize unknown words)
    - silence_timeout: float     (timeout in seconds to end utterance)
    - continuous_recog_callback: callable
    - event_callback: callable

    === CLEANUP ===
    Always call `terminat()` to stop microphone stream and recognition:
        recognizer.terminate()    
    """


    def setup(self, 
              api_key,
              language='en',      # ISO-639-1 language code
              context_prompt=None,        
              silence_timeout=0.5,
              event_callback=None,              
              continuous_recog_callback=None):
        
        if context_prompt and len(context_prompt) > 224:
            raise ValueError("context_prompt must be less than 224 characters")

        self.language_code = language      
        self.context_prompt = context_prompt
        self.event_callback = event_callback    
        self.continuous_recog_callback = continuous_recog_callback        
                            
        # Initialize the Groq client
        self.client = Groq(api_key=api_key)

        self.asr_event_queue = queue.Queue(maxsize=1)
        self.asr_event_thread = Thread(target=self._proccess_asr_events, daemon=True)        
        self.asr_event_thread.start()

        self.microphone_stream = MicrophoneStream(use_vad=True, silence_timeout=silence_timeout)
        self.microphone_stream.__enter__()  # manually enter the context

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
        self.microphone_stream.reset(seconds_to_keep=0)  # reset the microphone stream
        Logger.debug('waiting for voice activity...')
        self._asr_event_callback(SpeechRecogntionEvent.STARTED)
        try:
            buffered_chunks = []
            for chunk in self.microphone_stream:
                self._asr_event_callback(SpeechRecogntionEvent.RECOGNIZING)
                buffered_chunks.append(chunk)                
            
            if not buffered_chunks:                
                Logger.debug("No audio data received.")
                self._asr_event_callback(SpeechRecogntionEvent.STOPPED)
                return None, None
            
            # call Groq API            

            audio_bytes = io.BytesIO()
            with wave.open(audio_bytes, 'wb') as wf:
                wf.setnchannels(self.microphone_stream.get_channels())
                wf.setsampwidth(self.microphone_stream.get_sample_width())
                wf.setframerate(self.microphone_stream.get_rate())
                wf.writeframes(b''.join(buffered_chunks))
            audio_bytes.seek(0)

            transcription = self.client.audio.transcriptions.create(
                file=('audio.wav', audio_bytes.read()),
                model="whisper-large-v3-turbo",
                response_format="verbose_json",  
                language=self.language_code,
                temperature=0.0,
                prompt=self.context_prompt
            )            
            if transcription.text and transcription.text.strip():
                self._asr_event_callback(SpeechRecogntionEvent.RECOGNIZED)
                self._asr_event_callback(SpeechRecogntionEvent.STOPPED)
                return transcription.text.strip(), self.language_code
        except Exception as e:            
            Logger.error(f"[ERROR] Exception during recording: {e}")

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
        self.microphone_stream.__exit__(None, None, None)  # manually exit the context

    # def cleanup(self):         
    #     Logger.debug(f"{self.name} is terminating..")
        
        



