#!/usr/bin/python3

from config import *
from fp_audio.srv import Speech2Text, Speech2TextResponse
import numpy as np
import rospy
from speech_recognition import AudioData
import speech_recognition as sr
from std_msgs.msg import String


class Speech2TextService(object):

    def __init__(self) -> None:
        """Initialize a Speech to text node.
        """
        # Init recognizer
        self._r = sr.Recognizer()


    def start(self):
        """Start the node and the Speech2Text service.
        """
        # Init the node
        rospy.init_node('speech2text_service', anonymous=True)
        rospy.Service('s2t', Speech2Text, self._handle, buff_size=1)
        rospy.spin()


    def _handle(self, req):
        """Callback function for Speech2Text. Receive an audio and return a string
        containing the trascription of the audio, 'ERR1' if the audio does not contains
        words or is just non understendable, 'ERR2' if there are connection issues to
        the speect2text provider.

        Args:
            req: The input of the request.
        """
        data = np.array(req.input.data, dtype=np.int16)
        audio_data = AudioData(data.tobytes(), RATE, 2)
        try:
            spoken_text = self._r.recognize_google(audio_data, language=LANGUAGE)    
        except sr.UnknownValueError:
            spoken_text = 'ERR1'
            #print('[Speech2text] did not understand')
        except sr.RequestError as e:
            spoken_text = 'ERR2'
        return Speech2TextResponse(String(spoken_text))


if __name__ == '__main__':
    try:
        node = Speech2TextService()
        node.start()
    except rospy.ROSInterruptException:
        pass
