# Copyright (c) 2024 LuxAI S.A.
# 
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import time
from enum import Enum
import queue
from threading import Event, Thread
import rospy
from audio_common_msgs.msg import AudioData
import riva.client


class MicrophoneStream:
    """Opens a recording stream as responses yielding the audio chunks."""

    def __init__(self, buffer: queue.Queue) -> None:
        self.stream_buff = buffer
        self.closed = True

    def __enter__(self):
        self.closed = False
        return self


    def __exit__(self, type, value, traceback):
        self.closed = True
        self.stream_buff.put(None)


    def __next__(self) -> bytes:
        if self.closed:
            raise StopIteration
        chunk = self.stream_buff.get(timeout=2)
        if chunk is None:
            raise StopIteration
        
        data = [chunk]
        while True:
            try:
                chunk = self.stream_buff.get(block=False)
                if chunk is None:
                    assert not self.closed
                data.append(chunk)
            except queue.Empty:
                break

        return b''.join(data)

    def __iter__(self):
        return self


class RivaSpeechRecognition:
    class Event(Enum):
        STARTED = 1
        RECOGNIZING = 2
        RECOGNIZED = 3
        STOPPED = 4
        CANCELED = 5

    def __init__(self, language='en-US', detection_timeout=5, event_callback=None):        
        self.event_callback = event_callback    
        self.finished = False
        self._is_paused = Event()
        self._is_paused.clear()  # Start in paused state
        self.detection_timeout = detection_timeout

        self.aqueue = queue.Queue(maxsize=2000) # more than one minute         
        self.audio_rate = 16000
        self.language_code = language        
        self.server = 'localhost:50051'
        self.use_ssl = False
        self.ssl_cert = None
        self.profanity_filter = False
        self.automatic_punctuation = True
        self.no_verbatim_transcripts = False
        self.boosted_lm_words = []
        self.boosted_lm_score = 4.0
        self.speaker_diarization = True

        print(f"audio rate:{self.audio_rate}, language code:{self.language_code}", self.use_ssl, self.ssl_cert, self.boosted_lm_words)

        self.auth = riva.client.Auth(self.ssl_cert, self.use_ssl, self.server)
        self.asr_service = riva.client.ASRService(self.auth)
        self.config = riva.client.StreamingRecognitionConfig(
            config=riva.client.RecognitionConfig(
                encoding=riva.client.AudioEncoding.LINEAR_PCM,
                language_code=self.language_code,
                max_alternatives=1,
                profanity_filter=self.profanity_filter,
                enable_automatic_punctuation=self.automatic_punctuation,
                verbatim_transcripts=not self.no_verbatim_transcripts,
                enable_word_time_offsets=True,
                sample_rate_hertz=self.audio_rate,
                audio_channel_count=1,
            ),
            interim_results=True,
        )
        riva.client.add_word_boosting_to_config(self.config, self.boosted_lm_words, self.boosted_lm_score)
        riva.client.add_speaker_diarization_to_config(self.config, diarization_enable=self.speaker_diarization)        
        # start recognize service
        rospy.Subscriber('/qt_respeaker_app/channel0', AudioData, self._callback_audio_stream)

        self.asr_event_queue = queue.Queue(maxsize=1)
        self.asr_event_thread = Thread(target=self._proccess_asr_events, daemon=True)        
        self.asr_event_thread.start()



    def _callback_audio_stream(self, msg): 
        indata = bytes(msg.data)          
        try:
            self.aqueue.put_nowait(indata)            
        except:
            pass

    def _proccess_asr_events(self):
        while not rospy.is_shutdown():
            try:
                evt = self.asr_event_queue.get(timeout=2)
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

    def pasuse(self):
        self._is_paused.set()

    def resume(self):
        self._is_paused.clear()


    def recognize_once(self):               
        # self._is_paused.wait()
        # if self.finished:
        #     return None        
        # example : if audio rate is 16000 and respeaker buffersize is 512, then the last one second will be around 31 item in queue
        # while self.audio_queue.qsize() > int(16000 / 512 / 2):
        #     self.audio_queue.get()          
        #
        self.aqueue.queue.clear() 
        with MicrophoneStream(self.aqueue) as audio_chunk_iterator:
            if rospy.is_shutdown():
                return None, None 
            
            self._asr_event_callback(RivaSpeechRecognition.Event.STARTED)
            start_time = time.time()      
            try:
                responses = self.asr_service.streaming_response_generator(audio_chunks=audio_chunk_iterator, streaming_config=self.config)            
                transcript = None
                for response in responses:                
                    if response.results:                                        
                        for result in response.results:
                            if not result.alternatives:
                                continue
                            self._asr_event_callback(RivaSpeechRecognition.Event.RECOGNIZING)
                            transcript = result.alternatives[0].transcript
                            if result.is_final:
                                self._asr_event_callback(RivaSpeechRecognition.Event.RECOGNIZED)                            
                                return transcript.strip(), self.language_code

                    # check timeout 
                    if self.detection_timeout > 0 and not transcript:
                        elapsed_time = time.time() - start_time
                        if elapsed_time > self.detection_timeout:
                            break
            except Exception as e: 
                if not rospy.is_shutdown():
                    rospy.logwarn(str(e))

        self._asr_event_callback(RivaSpeechRecognition.Event.STOPPED)        
        return None, None


    def stop(self):        
        self.finished = True
        self._is_paused.clear()


    def recognize(self, recognition_callback):
        self.finished = False
        while not rospy.is_shutdown() and not self.finished:
            try:                
                text, lang = self.recognize_once()
                if text:
                    recognition_callback(text, lang)
            except Exception as e:
                rospy.logerr(str(e))


