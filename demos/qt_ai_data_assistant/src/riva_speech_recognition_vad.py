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
import grpc

import math
import numpy as np
import torch


class SileroVAD():
    def __init__(self, confidence_threshold=0.6, rate=16000):
        self.model, utils = torch.hub.load(repo_or_dir='snakers4/silero-vad',
                                           model='silero_vad',
                                           force_reload=False,
                                           trust_repo=True)

        if rate not in (16000, 8000):
            raise ValueError(f"SileroVAD: audio sample rate must be either 16000 or 8000")

        # (get_speech_ts, _, _, _, _) = utils
        self.rate = rate
        self.confidence_threshold = confidence_threshold
        (self.get_speech_timestamps,
        self.save_audio,
        self.read_audio,
        self.VADIterator,
        self.collect_chunks) = utils

    def _validate(self, model, inputs: torch.Tensor):
        with torch.no_grad():
            outs = model(inputs)
        return outs

    def _int2float(self, sound):
        abs_max = np.abs(sound).max()
        sound = sound.astype('float32')
        if abs_max > 0:
            sound *= 1/32768
        sound = sound.squeeze()  # depends on the use case
        return sound

    def get_sample_rate(self):
        return self.rate

    def get_frame_size(self):
        return 512 if self.rate == 16000  else 256
    
    def is_voice(self, audio_chunk):
        audio_int16 = np.frombuffer(audio_chunk, np.int16)
        audio_float32 = self._int2float(audio_int16)
        # get the confidences and add them to the list to plot them later
        confidence = self.model(torch.from_numpy(audio_float32), self.rate).item()
        return confidence > self.confidence_threshold


class MicrophoneStream:
    """Opens a recording stream as responses yielding the audio chunks."""

    def __init__(self, rate=16000, num_samples=512, vad: SileroVAD = None) -> None:
        self.vad = vad
        self.rate = rate
        self.num_samples = num_samples

        self.stream_buff = queue.Queue(maxsize=math.ceil(60 / (num_samples/rate))) # more than one minute
        self.closed = True

        self.voice_event = Event()  # Event to signal voice detection  

    def __enter__(self):
        self.closed = False
        return self


    def __exit__(self, type, value, traceback):        
        self.closed = True
        self.stream_buff.put(None)
        self.voice_event.set()


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


    def reset(self, seconds_to_keep=0.5):
        if seconds_to_keep <= 0:
            self.stream_buff.queue.clear()  # Clear the queue
            self.voice_event.clear()
            return 
        frames_to_keep = math.ceil(seconds_to_keep / (self.num_samples/self.rate))        
        # delete all except last frames_to_keep
        last_two_items = list(self.stream_buff.queue)[-1 * frames_to_keep:]  # Get the last item
        self.stream_buff.queue.clear()  # Clear the queue
        for item in last_two_items:
            self.stream_buff.put(item)  # Reinsert the last two items
        self.voice_event.clear()
        

    # TODO: do not add all chuncks when there is not voice activity
    #       instead buffere last few chunk and add it when self.voice_event.is_set()
    def put_chunk(self, chunk):
        try:
            # add the chunk to queue
            self.stream_buff.put_nowait(chunk)

            if not self.vad:                
                self.voice_event.set()
                return 
            
            # voice activity is starting 
            if self.vad.is_voice(chunk):
                if not self.voice_event.is_set():                    
                    # keep last 0.5 second and delete the rest
                    self.reset(seconds_to_keep=0.5)
                self.voice_event.set()
        except:
            pass

    def wait_for_voice(self, timeout=None):        
        # This will block until voice_event is set
        if not self.voice_event.wait(timeout=timeout):
            return False
        # self.voice_event.clear()  # Reset the event for future calls
        return not self.closed


class RivaSpeechRecognitionSilero:
    class Event(Enum):
        STARTED = 1
        RECOGNIZING = 2
        RECOGNIZED = 3
        STOPPED = 4
        CANCELED = 5

    def __init__(self, language='en-US', 
                 detection_timeout=5, 
                 event_callback=None,
                 use_vad=False):
        self.use_vad = use_vad
        self.event_callback = event_callback    
        self.finished = False
        self._is_paused = Event()
        self._is_paused.clear()  # Start in paused state
        self.detection_timeout = detection_timeout
        
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
        
        self.microphone_stream = MicrophoneStream(vad=SileroVAD(rate=self.audio_rate) if self.use_vad else None)
        self.audio_chunk_iterator = self.microphone_stream.__enter__()

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


    def _reinitilize_riva_client(self):
        self.auth = None
        self.asr_service = None
        self.auth = riva.client.Auth(self.ssl_cert, self.use_ssl, self.server)
        self.asr_service = riva.client.ASRService(self.auth)


    def _callback_audio_stream(self, msg):         
        self.microphone_stream.put_chunk(bytes(msg.data))


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

        self.microphone_stream.reset()
        if self.use_vad:
            rospy.logdebug('waiing for voice activity...')
            if not self.microphone_stream.wait_for_voice(timeout=5.0):
                return None, None            
            # rospy.logdebug('wait_for_voice detected!')
                
        if rospy.is_shutdown():
            return None, None 
        
        self._asr_event_callback(RivaSpeechRecognitionSilero.Event.STARTED)
        start_time = time.time()      
        try:
            responses = self.asr_service.streaming_response_generator(audio_chunks=self.audio_chunk_iterator, streaming_config=self.config)            
            transcript = None
            for response in responses:                
                if response.results:                                        
                    for result in response.results:
                        if not result.alternatives:
                            continue
                        self._asr_event_callback(RivaSpeechRecognitionSilero.Event.RECOGNIZING)
                        transcript = result.alternatives[0].transcript
                        if result.is_final:
                            self._asr_event_callback(RivaSpeechRecognitionSilero.Event.RECOGNIZED)                            
                            return transcript.strip(), self.language_code

                # check timeout 
                if self.detection_timeout > 0 and not transcript:
                    elapsed_time = time.time() - start_time
                    if elapsed_time > self.detection_timeout:
                        break              
        except Exception as e: 
            if not rospy.is_shutdown():
                code = None
                try: 
                    code = e.code()
                except:
                    pass
                if code == grpc.StatusCode.UNAVAILABLE:
                    rospy.logerr('Riva server is not available. Checking after 10 second...')
                    time.sleep(10)
                    self._reinitilize_riva_client()
                else:
                    rospy.logwarn(str(e))

        self._asr_event_callback(RivaSpeechRecognitionSilero.Event.STOPPED)        
        return None, None


    def stop(self):        
        self.finished = True
        self._is_paused.clear()        
        self.audio_chunk_iterator.__exit__(None, None, None)


    def recognize(self, recognition_callback):
        self.finished = False
        while not rospy.is_shutdown() and not self.finished:
            try:                
                text, lang = self.recognize_once()
                if text:
                    recognition_callback(text, lang)
            except Exception as e:
                rospy.logerr(str(e))


