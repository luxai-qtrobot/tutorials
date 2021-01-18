#!/usr/bin/env python3
import pyaudio
import rospy
from six.moves import queue
from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
from google.api_core import exceptions as gexcp
from qt_gspeech_interface.srv import *
NAME = 'qt_gspeech_service'

# Audio recording parameters
RATE = 16000
CHUNK = 1024  # 100ms

class MicrophoneStream(object):
    """Opens a recording stream as a generator yielding the audio chunks."""

    def __init__(self, rate, chunk):
        self._rate = rate
        self._chunk = chunk

        # Create a thread-safe buffer of audio data
        self._buff = queue.Queue()
        self.closed = True

    def __enter__(self):
        self._audio_interface = pyaudio.PyAudio()
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16,
            # The API currently only supports 1-channel (mono) audio
            # https://goo.gl/z757pE
            channels=1,
            rate=self._rate,
            input=True,
            frames_per_buffer=self._chunk,
            # ReSpeaker Mic device index is 2
            input_device_index = 2,
            # Run the audio stream asynchronously to fill the buffer object.
            # This is necessary so that the input device's buffer doesn't
            # overflow while the calling thread makes network requests, etc.
            stream_callback=self._fill_buffer,
        )

        self.closed = False

        return self

    def __exit__(self, type, value, traceback):
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self._buff.put(None)
        self._audio_interface.terminate()

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        """Continuously collect data from the audio stream, into the buffer."""
        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def generator(self):
        while not self.closed:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b"".join(data)

def qt_gspeech_callback(req):
    '''
    Callback function used by the service server to process
    requests from clients. It returns a QTrobotGspeechResponse
    '''
    if req.timeout:
        mic_timeout = req.timeout
    else:
        mic_timeout = 30
    output = ""
    answer_context = []
    speech_context = []
    language_code = "en-US"  # a BCP-47 language tag
    client = speech.SpeechClient()
    print(req.options)
    if req.options:
        print("there is options....")
        for option in req.options:
            answer_context.append(option.lower().strip())
        speech_context = types.SpeechContext(phrases = answer_context)
        config = types.RecognitionConfig(
            encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=RATE,
            language_code=language_code,
            speech_contexts = [speech_context]
        )
    else:
        print("there is no options....")
        config = types.RecognitionConfig(
            encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=RATE,
            language_code=language_code
        )

    streaming_config = types.StreamingRecognitionConfig(
        config=config, interim_results=True
    )

    with MicrophoneStream(RATE, CHUNK) as stream:
        audio_generator = stream.generator()
        requests = (
            types.StreamingRecognizeRequest(audio_content=content)
            for content in audio_generator
        )
        try:
            responses = client.streaming_recognize(streaming_config, requests, timeout=req.timeout)
            output = validate_response(responses, answer_context)
            pass

        except gexcp.DeadlineExceeded as e:
            output = "#TIMEOUT#"
            pass

    print("Detected [%s]" % (output))
    return QTrobotGspeechResponse(output)



def validate_response(responses, context):
    # to return:
    # output = transcript
    transcript = ""

    #num_chars_printed = 0
    for response in responses:
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

        # Display interim results, but with a carriage return at the end of the
        # line, so subsequent lines will overwrite them.
        #
        # If the previous result was longer than this one, we need to print
        # some extra spaces to overwrite the previous result
        #overwrite_chars = " " * (num_chars_printed - len(transcript))
        if not result.is_final:
             if context:
                 for option in context:
                     if option == transcript.lower().strip():
                         return transcript
        else:
             return transcript

    return transcript

def qtrobot_gspeech_service():
    rospy.init_node(NAME)
    s = rospy.Service('qt_gspeech_service', QTrobotGspeech, qt_gspeech_callback)
    print("Ready to Listen!")
    # spin() keeps Python from exiting until node is shutdown
    rospy.spin()

if __name__ == "__main__":
    qtrobot_gspeech_service()
