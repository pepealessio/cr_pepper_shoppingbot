#!/usr/bin/python3

from std_msgs.msg import Int16MultiArray, Int16, String
from fp_audio.srv import GetEmbedding, SetEmbedding, GetLabel
import numpy as np
import os
import rospy
import librosa

# try use RATE defined in cofing put file __init__.py in scripts
RATE = 16000
TEST_PATH = os.path.dirname(os.path.abspath(__file__))
EMBEDDINGS_FILE = TEST_PATH+"/../../scripts/embeddings_data.pk"
IDENTITIES = {
    1:"Alessio",
    2:"Giusy",
    3:"Teresa",
    4:"Alfonso",
    5:"Daniele",
    6:"Olandese",
    -1:"Unknown"
}

class ReidentificationTest(object):
    """
    Node implementing the test of reidentification module
    """
    
    def __init__(self): 
        self.__getEmbeddingFunc = None 
        self.__setEmbeddingFunc = None
        self.__getLabelFunc = None

    def __read_audio(self,path, sample_rate):
        """
        Load audio from file
        Args:
            path (String): the audio file path.
            sample_rate(int): audio sample rate.
        Return:
            audio: (np.ndarray) audio like numpy.ndarray int16 type
        """
        audio = librosa.load(path, sr=sample_rate)[0]
        return np.int16(audio * 32768)
       

    def __getEmbeddingFromAudio(self,audio):
        """
        Get audio embeddings.
        Args:
            audio: (np.ndarray) audio like numpy.ndarray int16 type
        Return:
            audio embeddings
        """
        data_to_send = Int16MultiArray()
        data_to_send.data = audio.tolist()
        getEmbeddingResp = self.__getEmbeddingFunc(data_to_send)
        return getEmbeddingResp.output

    def __getEmbeddings(self,folder):
        """
        Compute the embeddings from the audio stored in ID_folders. The ID_folders are stored in input folder 
        Args:
            folder: folder containing audio samples 
        Return:
            data: a list where each elements is a list like [audio_emb,audio_ID,audio_name]
        """
        # Get name of ID_Folders
        id_dir = os.listdir(folder)
        id_dir.sort()
        data = list()
        rospy.wait_for_service('getEmbedding')
        if self.__getEmbeddingFunc is None:
            self.__getEmbeddingFunc = rospy.ServiceProxy('getEmbedding', GetEmbedding)
        
        for id in id_dir:
            data_dir = os.path.join(folder,id)
            # Get name of files stored in current ID_Folder
            file_list = os.listdir(data_dir)
            file_list.sort()
            id = int(id)
            name = IDENTITIES[id]
            for f in file_list:
                file_audio_path = os.path.join(data_dir,f)
                audio = self.__read_audio(file_audio_path,RATE)
                embedding = self.__getEmbeddingFromAudio(audio)
                data.append([embedding,id,name])
        
        return data

    def __setup(self,training_folder):
        """
        Extract embeddings from the training folder and stored that.
        Args:
            training_folder (String): training folder path
        """
        training_data = self.__getEmbeddings(training_folder)
        
        rospy.wait_for_service('setEmbedding')
        if self.__setEmbeddingFunc is None:
            self.__setEmbeddingFunc = rospy.ServiceProxy('setEmbedding', SetEmbedding)
                
        for data in training_data:
            _ = self.__setEmbeddingFunc(data[0], Int16(data[1]), String(data[2]))

    def test(self,training_folder,test_folder):
        """
        Test re-identification module using as training data samples in input training folder
        and as test data samples in input test folder. 
        """
        # initialize trainig data
        self.__setup(training_folder)
        # get embeddings and id from the test data stored in test_folder
        test_data = self.__getEmbeddings(test_folder)
        if self.__getLabelFunc is None:
            self.__getLabelFunc = rospy.ServiceProxy('getLabel', GetLabel)
        for i,data in enumerate(test_data):
            getLabelResp = self.__getLabelFunc(data[0])
            label = getLabelResp.out_label.data
            if label!= data[1]:
                print(f" ID: {data[1]} predicted ID: {label} Test sample : {i+1}  Test Not Passed")
            else:
                print(f" ID: {data[1]} Test sample : {i+1}  Test Passed")

    def __cleanup(self): 
        """
        Remove test_case data
        """
        if(os.path.isfile(EMBEDDINGS_FILE)):
            os.remove(EMBEDDINGS_FILE)

    def __test_case(self,test_case_folder):
        """
        Compute test_case using data stored in test_case_folder
        Args:
            test_case_folder (String): test_case_folder where data are stored
        """
        print("----------------",test_case_folder.upper(),"----------------")
        test = os.path.join(TEST_PATH,test_case_folder)
        train_set = os.path.join(test,"train_set")
        test_set = os.path.join(test,"test_set")
        self.test(train_set,test_set)
        print()
        self.__cleanup()    

    def start(self): 
        rospy.init_node('test_reidentification', anonymous=True)

        test_cases = os.listdir(TEST_PATH)
        test_cases.sort()
        for test in test_cases:
            if not os.path.isfile(os.path.join(TEST_PATH,test)):
                self.__test_case(test)

if __name__ == "__main__":
    try:
        node = ReidentificationTest()
        node.start()
    except rospy.ROSInterruptException:
        pass