#!/usr/bin/python3

from buffer import Buffer
from config import *
from datetime import datetime
from fp_audio.srv import Speech2Text, GetEmbedding, SetEmbedding, GetLabel, StartListening, NextLabel
from gtts import gTTS
import numpy as np
from ros_chatbot.srv import Dialogue
from pepper_nodes.srv import Text2Speech, WakeUp, StartFollowing, ListeningMoving, ResponseMoving
from playsound import playsound
import rospy
from scipy.io.wavfile import write
from std_msgs.msg import Int16MultiArray, String, Int16, Bool
from threading import Lock


class CoreNode(object):
    """Node implementing the core flow and the two states of the robot."""

    def __init__(self, verbose=True):
        """Initialize the core node.
        """
        self._buffer = Buffer(max_size=MAX_EMBEDDING_PER_LABEL)
        self._human_presence = False
        self._mutex = Lock()
        self._persistent_services = dict()  # dict str -> Touple[func, module]
        self._prev_time = None
        self._prev_label = -1
        self._prev_name = ''

        self._verbose = verbose

    def _persistence_service_init(self, service_name, service_srv):
        """Init a persistent connection to a service and store the needed parameters.

        Args:
            service_name (str): the service name.
            service_srv (Module): the srv module.
        """
        self._persistent_services[service_name] = (rospy.ServiceProxy(service_name, service_srv, persistent=True), service_srv)

    def _persistence_service_call(self, service_name, *args):
        """Call a service with the provided arguments and return what the service provide.
        This method handle connection problem, but do NOT handle wrong call.

        Args:
            service_name (str): the service name.

        Returns:
            any: the service output.
        """
        rospy.wait_for_service(service_name)
        try:
            return self._persistent_services[service_name][0](*args)
        except rospy.ServiceException:
            self._persistence_service_init(service_name, self._persistent_services[service_name][1])
            return self._persistent_services[service_name][0](*args)

    def _persistence_service_close(self, service_name):
        """Close a service connection.

        Args:
            service_name (str): the service to shutdown.
        """
        try:
            self._persistent_services[service_name][0].close()
        except rospy.ServiceException:
            pass

    def _service_call(self, service_name, service_srv, *args):
        """Call a service whitout use a persistent connection.

        Args:
            service_name (str): the service name
            service_srv (Module): the service srv

        Returns:
            any: The service output
        """
        rospy.wait_for_service(service_name)
        func = rospy.ServiceProxy(service_name, service_srv)
        return func(*args)

    def start(self):
        """Start the node. Subscribe to all topic and services required. Wake up pepper
        and start some modules on that.
        """
        rospy.init_node('core', anonymous=True)
        rospy.Subscriber(RAW_AUDIO_TOPIC, Int16MultiArray, self._handle_audio)
        rospy.Subscriber(HUMAN_PRESENCE_TOPIC, Bool, self._handle_tracking)
        rospy.on_shutdown(self._handle_shutdown)

        # Init the persistent service proxy
        self._persistence_service_init('dialogue_server', Dialogue)
        self._persistence_service_init('getEmbedding', GetEmbedding)
        self._persistence_service_init('getLabel', GetLabel)
        self._persistence_service_init('listeningMoving', ListeningMoving)
        self._persistence_service_init('nextLabel', NextLabel)
        self._persistence_service_init('s2t', Speech2Text)
        self._persistence_service_init('responseMoving', ResponseMoving)
        self._persistence_service_init('setEmbedding', SetEmbedding)
        self._persistence_service_init('startListening', StartListening)
        self._persistence_service_init('tts', Text2Speech)

        # Pepper WakeUp & start following people
        if ON_PEPPER:
            if self._verbose:
                print('[CORE] Pepper wakeup')
            self._service_call('wakeup', WakeUp)

            if self._verbose:
                print('[CORE] Pepper startfollowing')
            self._service_call('startFollowing', StartFollowing)
        
        if self._verbose:
            print('[CORE] Starting done.')

        rospy.spin()

    def _chatbot_interaction(self, text, name, label):
        """Interaction with the chatbot

        Args:
            text (str): the sentence from the user
            name (str): the name of the user from re-identification (empty string if unknown)
            label (int): the id of the user from re-identification (-1 if unknown)

        Returns:
            tuple[str, int, str]: A touple containing the reply of the chatbot, the label of the
            user actually setted in the chatbot, and the name of the user actually setted in the chatbot.
        """

        if CHATBOT_RUNNING:
            bot_answer = self._persistence_service_call('dialogue_server', text, name, label)
            
            response = bot_answer.answer
            new_label = bot_answer.bot_id
            new_name = bot_answer.bot_username

        else:
            # Simulating the chatbot answer
            new_name = input('name: ')
            new_label = int(input('label: ') or '-1')
            response = input('response: ')
        return response, new_label, new_name

    def _handle_audio(self, audio):
        """Callback function to handle the receipt of an audio (so to understand and answer to
        a person who are talking).
        This method id thread-safe.

        Args:
            audio (Int16MultiArray): the audio received by the topic.

        Raise:
            rospy.ServiceException: if a service fail or it's not reachble.
        """
        # ______________________________________________________________________________
        # 1.    This method must be thread-safe, so we check with a mutex if it's already
        #       running. In that case, we stop the unnecessary new run.
        self._mutex.acquire()

        if not self._human_presence:
            self._mutex.release()
            return
        else:
            self._mutex.release()

        # ______________________________________________________________________________
        # 1.1   Speech2Text to understand if there is noise or not in that audio.
        #       Also, get the text from the audio. 
        #       If there's no words in the audio, restart the listening to keep active
        #       the michrophone.

        speech2textResp = self._persistence_service_call('s2t', audio)
        text = speech2textResp.output.data

        if (text == '' or text == 'ERR1' or text == 'ERR2'):
            if self._verbose:
                print(f'[CORE] 1. Does not unterstood, text={text}.')
            
            self._mutex.acquire()
            if self._human_presence:
                self._persistence_service_call('startListening')
            self._mutex.release()
            return

        if self._verbose:
            print(f'[CORE] 1. Listened: {text}')

        # This part is used just for save some audio to use in the tests from home.
        if SAVE_RAW_AUDIO:
            audio_data = np.array(audio.data).astype(np.float32, order='C') / 32768.0  # to float32
            write(os.path.join(REF_PATH, 'saved_audio', f'{datetime.now().strftime("%m-%d-%Y-%H-%M-%S")}.wav'), RATE, audio_data)

        # ______________________________________________________________________________
        # 2.    If the audio contains voice, we use getEmbedding to compute the 
        #       embedding of that audio and we add that in a buffer. 

        getEmbeddingResp = self._persistence_service_call('getEmbedding', audio)
        embedding = getEmbeddingResp.output

        # ______________________________________________________________________________
        # 3.    If in the previos conversational round the user was known and the time 
        #       between the two rounds is less than CONVERSATION_ROUND_MIN_LEN seconds, 
        #       use the previos known identity.
        time = datetime.now()

        if (self._prev_time is not None) and \
            ((time - self._prev_time).seconds <= CONVERSATION_ROUND_MAX_LEN) and \
            (self._prev_label != -1):

            if self._verbose:
                print(f'[CORE] 3. Re-Identification: Using previous identity.')
            
            label = self._prev_label
            name = self._prev_name
        
        # ______________________________________________________________________________
        #  3.1  Else try to get the label of the embedding previous computed.  
        else:
            getLabelResp = self._persistence_service_call('getLabel', embedding)
            label = getLabelResp.out_label.data
            name = getLabelResp.out_name.data

            if self._verbose:
                print(f'[CORE] 3.1 Re-Identification: name={name}, label={label}')
        
        self._prev_time = time
        self._prev_label = label
        self._prev_name = name
        
        # ______________________________________________________________________________
        # 3.2   If the label is known, we store that embedding with that label. 
        #       Otherwise we add that in a buffer because we assume the chatbot 
        #       give us the name in feaw conversation round. The buffer will be 
        #       cleared if the people will be identified to avoid to assign that 
        #       embedding to other people.

        if label == -1:  
            # unknown voice
            self._buffer.put(embedding)

        else:  # known voice
            self._persistence_service_call('setEmbedding', embedding, Int16(label), String(name))
            self._buffer.clear()


        # ______________________________________________________________________________
        # 4.    Interaction with the chatbot. The chatbot take in input the sentence, 
        #       and name, label from the reidentification service and provide in output
        #       a response and a new_label, new_name actually present in the slots.
        #
        #       If the identity is not known we provide the next label because there is
        #       a case in witch the chatbot require the label to execute the operation 
        #       just before of return the name to set the identity.

        if label != -1:
            response_text, new_label, new_name = self._chatbot_interaction(text, name, label)
        else:
            nextLabelResp = self._persistence_service_call('nextLabel')
            next_label = nextLabelResp.out_label.data
            response_text, new_label, new_name = self._chatbot_interaction(text, name, next_label)

        if self._verbose:
            print(f'[CORE] 4. Chatbot: response={response_text}, name={new_name}, label={new_label}.')


        # ______________________________________________________________________________
        # 5.    Set embedding if previously was unknown and now the name is provided by
        #       the chatbot. 
        
        if label == -1 and new_name != '':
        
            while not self._buffer.isEmpty():
                setEmbeddingResp = self._persistence_service_call('setEmbedding', self._buffer.pop(), Int16(new_label), String(new_name))
                new_label = setEmbeddingResp.out_label.data

            self._prev_label = new_label
            self._prev_name = new_name
            
            if self._verbose:
                print(f'[CORE] 5. Setted label={new_label} and name={new_name}')

        # ______________________________________________________________________________
        # 5.1.  Chatbot response can come after a person go away, so we chech if the person is
        #       here (for example saying goodbye and walk away without wait for a response)
        self._mutex.acquire()
        hp = self._human_presence
        self._mutex.release()

        if not hp:
            return


        # ______________________________________________________________________________      
        # 6.    Speech2Text to reproduce the response as audio.
        #       Pepper start making speacking movement. After that we made pepper speak.
        #       Finally pepper start moving for listening.
        if ON_PEPPER:
            self._persistence_service_call('responseMoving') 
            self._persistence_service_call('tts', response_text)
            self._persistence_service_call('listeningMoving')
        else:
            try:
                to_speak = gTTS(text=response_text, lang=LANGUAGE, slow=False)
                to_speak.save("temp.wav")
                playsound("temp.wav")
                os.remove("temp.wav")
            except AssertionError:
                pass
            
        if self._verbose:
            print(f'[CORE] 6. TTS: {response_text}')        

        # ______________________________________________________________________________
        # 6.1   After Pepper has told, we restart the listening.
        self._persistence_service_call('startListening')

    def _handle_tracking(self, presence):
        """Callback function for topic track/human_presence. Implement an FSM to 
        stop all operation and reset the state if the human exit the scene and 
        restart if an human enter in the scene.
        This method id thread-safe.
        """
        # ______________________________________________________________________________  
        # 0.    This variable contains True if an human is present, false if not. We can
        #       assume if this function is running that state this value is just changed.
        self._mutex.acquire()
        self._human_presence = presence.data

        if self._human_presence:
            # ______________________________________________________________________________  
            # STATE S1:
            # In this state the microphone is enabled and consequentially all the 
            # application work.

            if self._verbose:
                print('[CORE] Tracking: Human presence detected.')

            self._persistence_service_call('startListening')

            # Also, Pepper start moving for listening.
            if ON_PEPPER:            
                self._persistence_service_call('listeningMoving')

        else:
            # ______________________________________________________________________________  
            # STATE S0:
            # In this state there is no human so we can disable the microphone and reset
            # the state of the application so that can restart when a new human come.

            if self._verbose:
                print('[CORE] Tracking: There is not human anymore.')

            self._prev_time = None
            self._prev_label = -1
            self._prev_name = ''
            self._buffer.clear()
            
            # Also reset the state of the chatbot, so that will be ready to start a new
            # conversation.
            if CHATBOT_RUNNING:
                self._chatbot_interaction('/restart', '', -1)

            # Stop moving Pepper
            if ON_PEPPER:
                self._persistence_service_call('stopMoving')

        self._mutex.release()

    def _handle_shutdown(self):
        """On killing the application, stop all the service that can be running on 
        Pepper, so the following and the movement. After that set Pepper in rest 
        position.
        """
        if self._verbose:
            print('[CORE] Shutdown signal received. Stopping the application')

        # Close all persistent service
        for ps in self._persistent_services:
            self._persistence_service_close(ps)


if __name__ == "__main__":
    try:
        node = CoreNode()
        node.start()

    except rospy.ServiceException as e:
            print(f'[CORE] rospy.ServiceException. Service call failed: {e}')
    except Exception as e:
        print(f'[CORE] ERROR in execution: {e}')
