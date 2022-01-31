#!/usr/bin/python3

from config import *
from fp_audio.srv import StartListening, StopListening
import numpy as np
import rospy
import speech_recognition as sr
from std_msgs.msg import Int16MultiArray
from threading import Lock


class AudioDetected(object):
    """This class use py_audio to get audio wawe with a detection."""

    def __init__(self, raw_audio_topic, verbose=True) -> None:
        """Initialize an object containing the node who listen the audio from the robot and 
        implement an voice detection.

        Args:
            raw_audio_topic (str): The name of the topic on which the audio must be published.
        """
        self._publisher = rospy.Publisher(raw_audio_topic, Int16MultiArray, queue_size=None)
        # Initialize a Recognizer
        self._r = sr.Recognizer()
        # Audio source
        self._m = sr.Microphone(device_index=MICROPHONE_INDEX, sample_rate=RATE, chunk_size=CHUNK_SIZE)

        # Mutex because this service can be called from multiple handler, so we need that 
        # thread safe.
        self._mutex = Lock()
        # self._stop_listening_func = None
        # self._running = False
        self._verbose = verbose
    
    def start(self):
        """Start the node and calibrate the microphone to the local noise.
        """
        rospy.init_node('audio_detected_node', anonymous=True)
        rospy.Service('startListening', StartListening, self.startListening)
        # rospy.Service('stopListening', StopListening, self._handle_stop_listening)

        # self.startListening()

        rospy.spin()

    def startListening(self, req):
        """Calibrate the microphone with the ambient noise and start listening.
        """
        with self._m as source:
            if self._verbose:
                print("[T2S] Start calibrating...")

            self._r.adjust_for_ambient_noise(source, duration=CALIBRATION_TIME)

            if self._verbose:
                print("[T2S] Calibrating finished.")

            if self._verbose:
                print("[T2S] Start listened")
            audio = self._r.listen(source)

            data_to_send = Int16MultiArray()
            data_to_send.data = np.frombuffer(audio.get_raw_data(), dtype=np.int16)

            if self._verbose:
                print("[T2S] Listened")
            
            return data_to_send

    # def stopListening(self):
    #     """Stop the listening from the microphone.
    #     This method id thread-safe.
    #     """
    #     self._mutex.acquire()

    #     if self._running:
    #         if self._stop_listening_func is not None:
    #             self._stop_listening_func()
    #             self._stop_listening_func = None
    #             if self._verbose:
    #                 print("[T2S] Stop listening")

    #         self._running = False
        
    #     self._mutex.release()

    # def _listened_callback(self, recognizer, audio):
    #     """Callback function to the listen_in_background function.
    #     """
    #     data_to_send = Int16MultiArray()
    #     data_to_send.data = np.frombuffer(audio.get_raw_data(), dtype=np.int16)
    #     self._publisher.publish(data_to_send)

    # def _handle_start_listening(self, req):
    #     """Callback function for startListening service.
    #     """
    #     self.startListening()
    #     return "ACK"

    # def _handle_stop_listening(self, req):
    #     """Callback function for stopListening service.
    #     """
    #     self.stopListening()
    #     return "ACK"


if __name__ == "__main__":
    try:
        node = AudioDetected(RAW_AUDIO_TOPIC)
        node.start()
    except rospy.ROSInterruptException:
        node.stopListening()
