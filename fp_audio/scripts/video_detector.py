#!/usr/bin/env python3

from config import *
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from threading import Lock


class VideoDetector(object):
    """This node see if an human was present and publish that presence on that topic."""

    def __init__(self):
        """Load the model and initalize the publisher."""
        protoPath = os.path.sep.join([REF_PATH, 'face_detection_model', "deploy.prototxt"])
        modelPath = os.path.sep.join([REF_PATH, 'face_detection_model', "res10_300x300_ssd_iter_140000.caffemodel"])
        self._detector = cv2.dnn.readNetFromCaffe(protoPath, modelPath)

        self._publisher = rospy.Publisher(HUMAN_PRESENCE_TOPIC, Bool, queue_size=0)

        self._curr_state = False
        self._change_state_count = 0

        self._mutex_net = Lock()

    def _handle_frame(self, msg):
        """Callback function. Receive an image and use a detector to verify if almost one faces is in the image.
        This method is thread-safe."""
        # Obtain a frame from the rospy message
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
       
        # Transform the image in a blob
        imageBlob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0), swapRB=False, crop=False)
        
        # Make the prediction
        self._mutex_net.acquire()
        self._detector.setInput(imageBlob)
        detections = self._detector.forward()
        self._mutex_net.release()
        
        # we ensure to have at least one face
        if len(detections) > 0:
            # we're making the assumption that each image has only ONE face, so find the bounding 
            # box with the largest probability
            i = np.argmax(detections[0, 0, :, 2])
            confidence = detections[0, 0, i, 2]

            # ensure that the detection with the largest probability also means our minimum probability 
            # test (thus helping filter out weak detections)
            if confidence > FACE_MIN_DETECTION_CONFIDENCE:
                self._publish_state(True)
            else: 
                self._publish_state(False)
        else:
            self._publish_state(False)

    def _publish_state(self, new_state):
        """Publish the new state just if changed"""
        if new_state != self._curr_state:
            if self._change_state_count >= HUMAN_PRESENCE_GHOST_FRAME:
                self._publisher.publish(Bool(new_state))
                self._curr_state = new_state
            else:
                self._change_state_count += 1
        else:
            self._change_state_count = 0


    def start(self):
        """Init the node and subscribe to the video input"""
        rospy.init_node('video_detection', anonymous=True)
        rospy.Subscriber(VIDEO_TOPIC, Image, self._handle_frame)
        rospy.spin()


if __name__ == '__main__':
    my_node = VideoDetector()
    my_node.start()
