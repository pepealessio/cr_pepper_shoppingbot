#!/usr/bin/python
from naoqi import ALProxy
from optparse import OptionParser
from pepper_nodes.srv import *
import rospy
from std_msgs.msg import Bool


class MovingNode:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.listening_movevent = ALProxy("ALListeningMovement", ip, port)
        self.speacking_movevent = ALProxy("ALSpeakingMovement", ip, port)

    def start(self):
        """Start the node.
        """
        rospy.init_node("follow_node")      
        rospy.Service("stopMoving", StopMoving, self._handle_stop_moving)
        rospy.Service("listeningMoving", ListeningMoving, self._handle_listening_moving)
        rospy.Service("responseMoving", ResponseMoving, self._handle_response_moving)
        rospy.on_shutdown(self._handle_shutdown)
        rospy.spin()

    def _set_listening_moving(self, onOff):
        """Set the state of listening movement

        Args:
            onOff (bool): True if activated, otherwise False
        """
        try:
            self.listening_movevent.setEnabled(onOff)
        except:
            self.listening_movevent = ALProxy("ALListeningMovement", self.ip, self.port)
            self.listening_movevent.setEnabled(onOff)

    def _set_speacking_moving(self, onOff):
        """Set the state of speacking movement

        Args:
            onOff (bool): True if activated, otherwise False
        """
        try:
            self.speacking_movevent.setEnabled(onOff)
        except:
            self.speacking_movevent = ALProxy("ALSpeakingMovement", self.ip, self.port)
            self.speacking_movevent.setEnabled(onOff)

    def _handle_stop_moving(self, req):
        """Handler for stop moving requests

        Args:
            req (any): The request (in this case is empty)

        Returns:
            std_msgs/string: ack
        """
        self._set_listening_moving(False)
        self._set_speacking_moving(False)
        return "ACK" 

    def _handle_listening_moving(self, req):
        """Handler for start listening moving requests

        Args:
            req (any): The request (in this case is empty)

        Returns:
            std_msgs/string: ack
        """
        self._set_listening_moving(True)
        self._set_speacking_moving(False)
        return "ACK" 
    
    def _handle_response_moving(self, req):
        """Handler for start response moving requests

        Args:
            req (any): The request (in this case is empty)

        Returns:
            std_msgs/string: ack
        """
        self._set_listening_moving(False)
        self._set_speacking_moving(True)
        return "ACK"

    def _handle_shutdown(self):
        """Handler for rospy shutdown.
        """
        self._set_listening_moving(False)
        self._set_speacking_moving(False)
    

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()

    try:
        node = MovingNode(options.ip, int(options.port))
        node.start()
    except rospy.ROSInterruptException:
        pass
