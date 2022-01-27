#!/usr/bin/python3

from buffer import Buffer
from config import *
from datetime import datetime
from fp_audio.srv import Speech2Text, GetEmbedding, SetEmbedding, GetLabel, StartListening, StopListening, NextLabel
from ros_chatbot.srv import Dialogue, ActionService
from pepper_nodes.srv import Text2Speech, WakeUp, Rest, StartFollowing, StopFollowing, StopMoving, ListeningMoving, ResponseMoving
import rospy
from std_msgs.msg import Int16MultiArray, String, Int16, Bool


class CoreNode(object):
    """Node implementing the core flow and the two states of the robot."""

    def __init__(self, verbose=True):
        """Initialize the core node.
        """
        self._buffer = Buffer(max_size=MAX_EMBEDDING_PER_LABEL)
        self._conversation_started = False
        self._prev_time = None
        self._prev_label = -1
        self._prev_name = ''

        self._verbose = verbose

    def start(self):
        """Start the node.
        """
        rospy.init_node('core', anonymous=True)
        rospy.Subscriber(RAW_AUDIO_TOPIC, Int16MultiArray, self._handle_audio)
        rospy.Subscriber(HUMAN_PRESENCE_TOPIC, Bool, self._handle_tracking)
        rospy.on_shutdown(self._handle_shutdown)

        # --- Pepper WakeUp & start following people
        if ON_PEPPER:
            print('wakeup')
            rospy.wait_for_service('wakeup')
            wakeUpFunc = rospy.ServiceProxy('wakeup', WakeUp)
            _ = wakeUpFunc()

            print('startfollowing')
            rospy.wait_for_service('startFollowing')
            startFollowingFunc = rospy.ServiceProxy('startFollowing', StartFollowing)
            _ = startFollowingFunc()

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
            rospy.wait_for_service('dialogue_server')
            dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)
            bot_answer = dialogue_service(text, name, label)
            
            response = bot_answer.answer
            new_label = bot_answer.bot_id
            new_name = bot_answer.bot_username

            self._conversation_started = True
        else:
            # Simulating the chatbot answer
            new_name = input('name: ')
            new_label = int(input('label: ') or '-1')
            response = input('response: ')
        return response, new_label, new_name

    def _handle_audio(self, audio):
        """Callback function to handle the receipt of an audio (so to understand and answer to
        a person who are talking).

        Args:
            audio (Int16MultiArray): the audio received by the topic.

        Raise:
            rospy.ServiceException: if a service fail or it's not reachble.
        """
        try:
            # ______________________________________________________________________________
            # 1.    Speech2Text to understand if there is noise or not in that audio.
            #       Also, get the text from the audio. 

            rospy.wait_for_service('s2t')
            speech2textFunc = rospy.ServiceProxy('s2t', Speech2Text)
            speech2textResp = speech2textFunc(audio)
            text = speech2textResp.output.data

            if self._verbose:
                print(f'[CORE] 1. Listened: {text}')

            if (text == '' or text == 'ERR1' or text == 'ERR2'):
                return

            # ______________________________________________________________________________
            # 1.1   Stop background listening after a sentence was getted to avoid the issues 
            #       of Pepper listen what it said itself.
            rospy.wait_for_service('stopListening')
            stopListeningFunc = rospy.ServiceProxy('stopListening', StopListening)
            _ = stopListeningFunc()

            if self._verbose:
                print('[CORE] 1.1. Stop Listening')


            # ______________________________________________________________________________
            # 2.    If the audio contains voice, we use getEmbedding to compute the 
            #       embedding of that audio and we add that in a buffer. 

            rospy.wait_for_service('getEmbedding')
            getEmbeddingFunc = rospy.ServiceProxy('getEmbedding', GetEmbedding)
            getEmbeddingResp = getEmbeddingFunc(audio)
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
                getLabelFunc = rospy.ServiceProxy('getLabel', GetLabel)
                getLabelResp = getLabelFunc(embedding)
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
                rospy.wait_for_service('setEmbedding')
                setEmbeddingFunc = rospy.ServiceProxy('setEmbedding', SetEmbedding)
                _ = setEmbeddingFunc(embedding, Int16(label), String(name))

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
                rospy.wait_for_service('nextLabel')
                nextLabelFunc = rospy.ServiceProxy('nextLabel', NextLabel)
                nextLabelResp = nextLabelFunc()
                next_label = nextLabelResp.out_label.data

                response_text, new_label, new_name = self._chatbot_interaction(text, name, next_label)

            if self._verbose:
                print(f'[CORE] 4. Chatbot: response={response_text}, name={new_name}, label={new_label}.')


            # ______________________________________________________________________________
            # 5.    Set embedding if previously was unknown and now the name is provided by
            #       the chatbot. 
            
            if label == -1 and new_name != '':
                rospy.wait_for_service('setEmbedding')
                setEmbeddingFunc = rospy.ServiceProxy('setEmbedding', SetEmbedding)
                
                while not self._buffer.isEmpty():
                    setEmbeddingResp = setEmbeddingFunc(self._buffer.pop(), Int16(new_label), String(new_name))
                    new_label = setEmbeddingResp.out_label.data

                self._prev_label = new_label
                self._prev_name = new_name
                
                if self._verbose:
                    print(f'[CORE] 5. Setted label={new_label} and name={new_name}')


            # ______________________________________________________________________________      
            # 6.    Speech2Text to reproduce the response as audio.

            # ______________________________________________________________________________
            # 6.1   Pepper start making speacking movement

            if ON_PEPPER:
                rospy.wait_for_service('responseMoving')
                responseMovingFunc = rospy.ServiceProxy('responseMoving', ResponseMoving)
                _ = responseMovingFunc()  

            # ______________________________________________________________________________
            # 6.2   We have to stop the listening before the robot talk because the microphone 
            #       hear pepper and handle that sentence.

            if ON_PEPPER:
                rospy.wait_for_service('tts')
                text2speechFunc = rospy.ServiceProxy('tts', Text2Speech)
                _ = text2speechFunc(response_text)

            if self._verbose:
                print(f'[CORE] TTS: {response_text}')

            # ______________________________________________________________________________
            # 6.3   Pepper start listening, so we start listening movements

            if ON_PEPPER:
                rospy.wait_for_service('listeningMoving')
                listeningMovingFunc = rospy.ServiceProxy('listeningMoving', ListeningMoving)
                _ = listeningMovingFunc() 

            # ______________________________________________________________________________
            # 6.4   After Pepper has told, we restart the listening.

            rospy.wait_for_service('startListening')
            startListeningFunc = rospy.ServiceProxy('startListening', StartListening)
            _ = startListeningFunc()

            if self._verbose:
                print('[CORE] 6.4. Start Listening')

        except rospy.ServiceException as e:
            print(f'[CORE] rospy.ServiceException. Service call failed: {e}')

        except KeyboardInterrupt:
            print('[CORE] Terminated bu user.')

        except Exception as e:
            print(f'[CORE] ERROR in execution: {e}')

    def _handle_tracking(self, presence):
        """Callback function for topic track/human_presence. Implement an FSM to 
        stop all operation and reset the state if the human exit the scene and 
        restart if an human enter in the scene.
        """
        # This variable contains True if an human is present, false if not. We can
        # assume if this function is running that state this value is just changed.
        humanPresent = presence.data

        if humanPresent:
            # In this state the microphone is enabled and consequentially all the 
            # application work.

            if self._verbose:
                print('[CORE] Tracking: Human presence detected.')

            rospy.wait_for_service('startListening')
            startListeningFunc = rospy.ServiceProxy('startListening', StartListening)
            _ = startListeningFunc()

            if self._verbose:
                print('[CORE] Start Listening')

            # Also, Pepper start moving for listening.
            if ON_PEPPER:            
                rospy.wait_for_service('listeningMoving')
                listeningMovingFunc = rospy.ServiceProxy('listeningMoving', ListeningMoving)
                _ = listeningMovingFunc() 
        else:
            # In this state there is no human so we can disable the microphone and reset
            # the state of the application so that can restart when a new human come.

            if self._verbose:
                print('[CORE] Tracking: There is not human anymore.')

            rospy.wait_for_service('stopListening')
            stopListeningFunc = rospy.ServiceProxy('stopListening', StopListening)
            _ = stopListeningFunc()

            if self._verbose:
                print('[CORE] Stop Listening')

            self._prev_time = None
            self._prev_label = -1
            self._prev_name = ''
            self._buffer.clear()
            
            # Also reset the state of the chatbot, so that will be ready to start a new
            # conversation.
            if CHATBOT_RUNNING and self._conversation_started:
                rospy.wait_for_service('trigger_action')
                triggerActionFunc = rospy.ServiceProxy('trigger_action', ActionService)
                _ = triggerActionFunc('action_restart')

                self._conversation_started = False

            # Stop moving Pepper
            if ON_PEPPER:
                rospy.wait_for_service('stopMoving')
                stopFollowingFunc = rospy.ServiceProxy('stopMoving', StopMoving)
                _ = stopFollowingFunc()

    def _handle_shutdown(self):
        """On killing the application, stop all the service that can be running on 
        Pepper, so the following and the movement. After that set Pepper in rest 
        position.
        """
        if self._verbose:
            print('[CORE] Shutdown signal received. Stopping the application')

        if ON_PEPPER:
            rospy.wait_for_service('stopFollowing')
            stopFollowingFunc = rospy.ServiceProxy('stopFollowing', StopFollowing)
            _ = stopFollowingFunc()

            rospy.wait_for_service('stopMoving')
            stopFollowingFunc = rospy.ServiceProxy('stopMoving', StopMoving)
            _ = stopFollowingFunc()

            rospy.wait_for_service('rest')
            restFunc = rospy.ServiceProxy('rest', Rest)
            _ = restFunc()


if __name__ == "__main__":
    node = CoreNode()
    node.start()
