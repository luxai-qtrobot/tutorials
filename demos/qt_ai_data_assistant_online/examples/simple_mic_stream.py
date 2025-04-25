import os
import sys
import wave
import rospy

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))
from asr.microphone_stream import MicrophoneStream
from utils.logger import Logger

if __name__ == '__main__':
    rospy.init_node('simple_mic_stream', anonymous=True)

    output_file = 'recorded_voice.wav'
    wf = wave.open(output_file, 'wb')

    with MicrophoneStream(use_vad=True, silence_timeout=1.0) as mic:        
        wf.setnchannels(mic.get_channels())
        wf.setsampwidth(mic.get_sample_width())  # 16-bit audio = 2 bytes
        wf.setframerate(mic.get_rate())
        
        while not rospy.is_shutdown():
            try:
                Logger.info("waiting for voice...")
                for chunk in mic:
                    Logger.debug(f"Voice segment received ({len(chunk)} bytes), saving...")
                    wf.writeframes(chunk)
            except Exception as e:
                Logger.warning(f"Exception during recording: {e}")

    wf.close()
    Logger.info(f"Audio saved to: {output_file}")
