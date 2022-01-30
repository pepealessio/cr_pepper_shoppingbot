#!/usr/bin/python3

from config import *
from datetime import datetime
from fp_audio.srv import GetEmbedding, GetEmbeddingResponse, SetEmbedding, SetEmbeddingResponse, GetLabel, GetLabelResponse, NextLabel, NextLabelResponse
from identification.deep_speaker.audio import get_mfcc
from identification.deep_speaker.model import get_deep_speaker
from identification.utils import batch_cosine_similarity, dist2id
import numpy as np
import os
import pickle
import rospy
from scipy.io.wavfile import write
from std_msgs.msg import Float32MultiArray, String, Int16
from threading import Lock


class EmbeddingManager(object):

    def __init__(self):
        """Init an embedding manager node who has 3 service: get embeddng of an audio,
        set embedding associated with a label of an audio and get a label based on a 
        similarity distance between the various embeddings.
        """
        self._model = get_deep_speaker(os.path.join(REF_PATH, 'audio_embedding_model', 'deep_speaker.h5'))

        self._mutex_data = Lock()
        self._mutex_net = Lock()

    def _load_data(self):
        """Try to load saved data and if there not exist, create a new empty data structure.

        Data is structured like:
        {
            'X'     : [emb1, emb2, ...],
            'y'     : [1,    2,    ...],
            'y2name :   {
                            1 : 'alessio',
                            2 : 'teresa,
                        }
        }

        
        Returns:
            Dict: the dictionary with the data.
        """
        try:
            with open(os.path.join(REF_PATH, DATA_FILENAME), 'rb') as fh:
                data = pickle.load(fh)
        except Exception as e:
            data = dict()
            data['X'] = list()
            data['y'] = list()
            data['y2name'] = dict()
        return data
    
    def _store_data(self, data):
        """Save the data in a file.


        Args:
            data (dict): The dictionary containing all the data.
        """
        try:
            with open(os.path.join(REF_PATH, DATA_FILENAME), 'wb') as fh:
                pickle.dump(data, fh)
        except:
            print("[RE-IDENTIFICATION] Can't save the state.")

    def start(self):
        """Initialize the node and start the services provided by this node.
        """
        rospy.init_node('reidentification_node', anonymous=True)
        rospy.Service('getEmbedding', GetEmbedding, self._handle_get_embedding)
        rospy.Service('setEmbedding', SetEmbedding, self._handle_set_embedding)
        rospy.Service('getLabel', GetLabel, self._handle_get_label)
        rospy.Service('nextLabel', NextLabel, self._handle_next_label)
        rospy.spin()

    def _handle_get_embedding(self, req):
        """Callback function of GetEmbedding Service. That recive an audio, compute 
        the embedding using a NN and return that.
        This method is thread-safe.


        Args:
            req: The input of the request.
        """
        int_audio = np.array(req.input.data, dtype=np.int16)
        audio_data = int_audio.astype(np.float32, order='C') / 32768.0  # to float32

        if SAVE_RAW_AUDIO:
            write(os.path.join(REF_PATH, 'saved', f'{datetime.now().strftime("%m-%d-%Y-%H-%M-%S")}.wav'), RATE, audio_data)
        
        mfcc = get_mfcc(audio_data, RATE)

        self._mutex_net.acquire()
        embedding = self._model.predict(np.expand_dims(mfcc, 0))
        self._mutex_net.release()

        embedding = embedding[0].tolist()

        to_return = Float32MultiArray()
        to_return.data = embedding

        return GetEmbeddingResponse(to_return)
    
    def _handle_set_embedding(self, req):
        """Callback function of SetEmbedding Service. That recive an embedding and a label 
        and save this association.
        This method is thread-safe.
        

        Args:
            req: The input of the request.
        """
        # Get embeddings and label in 
        embedding = np.array(req.in_embedding.data)
        name = req.in_name.data
        label = req.in_label.data

        # Load data from file
        self._mutex_data.acquire()
        data = self._load_data()

        # If label exist use that label, otherwise set a new label and
        # associate the name at that.
        unique, counts = np.unique(data['y'], return_counts=True)
        label_count = dict(zip(unique, counts))

        if label not in data['y2name']:
            label = len(unique) + 1
            data['y2name'][label] = name

        if (label not in label_count or label_count[label] <= MAX_EMBEDDING_PER_LABEL):
            data['X'].append(embedding)
            data['y'].append(label)
            self._store_data(data)
        
        self._mutex_data.release()

        # print(f"[Re-Identification] Setting an embedding with label {label} .")

        return SetEmbeddingResponse(Int16(label))

    def _handle_get_label(self, req):
        """Callback function of GetLabel Service. That recive an embedding and get a label 
        if that voice is known (or an empty label).
        This method is thread-safe.


        Args:
            req: The input of the request.
        """
        embedding = np.array([req.in_embedding.data], dtype=np.float32)

        self._mutex_data.acquire()
        data = self._load_data()
        self._mutex_data.release()

        if len(data['X']) > 0:
            # Distance between the sample and the support set
            rep_embedding = np.repeat(embedding, len(data['X']), 0)
            cosine_dist = batch_cosine_similarity(np.array(data['X']), rep_embedding)
            # Matching
            label = dist2id(cosine_dist, data['y'], REID_TH, mode='avg')
        
        if len(data['X']) == 0 or label is None:
            label = -1
            name = ''
        else:
            name = data['y2name'][label]

        return GetLabelResponse(Int16(label), String(name))

    def _handle_next_label(self, req):
        """Callback function for NextLabel Service. Return the label that will be assigned
        to the next identity.
        This method is thread-safe.
        """
        self._mutex_data.acquire()
        data = self._load_data()
        self._mutex_data.release()

        unique = np.unique(data['y'])
        next_label = len(unique) + 1
        
        return NextLabelResponse(Int16(next_label))


if __name__ == '__main__':
    try:
        node = EmbeddingManager()
        node.start()
    except rospy.ROSInterruptException:
        pass
