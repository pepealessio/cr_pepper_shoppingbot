#!/usr/bin/python
from naoqi import ALProxy
from optparse import OptionParser
from pepper_nodes.srv import *
import rospy
from std_msgs.msg import Bool

HUMAN_PRESENCE_TOPIC = '/track/human_presence'

class FollowingNode:

    def __init__(self, ip, port):
        self.ip = ip
        self.port = port

        # self._publisher = rospy.Publisher(HUMAN_PRESENCE_TOPIC, Bool, queue_size=10)
        self.ba_service  = ALProxy("ALBasicAwareness", ip, port)
        # self.ba_service.setEngagementMode("FullyEngaged")
        # self.al_memory = ALProxy("ALMemory", ip, port)
        # self.qi_session = self.al_memory.session()
        # self._sub_list = []

    def start(self):
        rospy.init_node("follow_node")      
        rospy.Service("startFollowing", StartFollowing, self._handle_start_following)
        rospy.Service("stopFollowing", StopFollowing, self._handle_stop_following)

        # self._register_callback("ALBasicAwareness/HumanTracked", self._handle_human_tracked)
        # self._register_callback("ALBasicAwareness/HumanLost", self._handle_human_lost)

        rospy.spin()

    def _handle_start_following(self, req):
        try:
            self.ba_service.setEnabled(True)
        except:
            self.ba_service  = ALProxy("ALBasicAwareness", self.ip, self.port)
            self.ba_service.setEnabled(True)
        return "ACK"

    def _handle_stop_following(self, req):
        try:
            self.ba_service.setEnabled(False)
        except:
            self.ba_service  = ALProxy("ALBasicAwareness", self.ip, self.port)
            self.ba_service.setEnabled(False)
        return "ACK" 
    
    # def _register_callback(self, event_name, callback):
    #     try:
    #         mem = self.qi_session.service("ALMemory")
    #     except:
    #         self.al_memory = ALProxy("ALMemory", self.ip, self.port)
    #         self.qi_session = self.al_memory.session()
    #         mem = self.qi_session.service("ALMemory")

    #     sub = mem.subscriber(event_name)
    #     sub.signal.connect(callback)
    #     self._sub_list.append(sub)

    # def _handle_human_tracked(self, value):
    #     print('tracked' + str(value))
    #     data_to_send = Bool()
    #     data_to_send.data = True
    #     self._publisher.publish(data_to_send)

    # def _handle_human_lost(self, value):
    #     print('untracked' + str(value))
    #     data_to_send = Bool()
    #     data_to_send.data = False
    #     self._publisher.publish(data_to_send)
    

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()

    try:
        node = FollowingNode(options.ip, int(options.port))
        node.start()
    except rospy.ROSInterruptException:
        pass
