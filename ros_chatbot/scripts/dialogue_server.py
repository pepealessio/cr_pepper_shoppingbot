#!/usr/bin/env python3
import json
from ros_chatbot.srv import Dialogue, DialogueResponse

import rospy
import requests

class UserMessanger():
    """
    The aim of this class is to store username and id information about the current user that is using
    the bot
    """
    def __init__(self,username="",id=-1):
        self.username = username
        self.id = id

    def setID(self,id):
        self.id = id

    def setUsername(self,username):
        self.username = username
    
    def getID(self):
        return self.id

    def getUsername(self):
        return self.username
    
    def isUserDefined(self):
        """
        Checks if there is already an user defined
        """
        return "" != self.username

    def reset(self):
        """
        Resets fields of this class
        """
        self.username=""
        self.id=-1


class DialogueServer:
    """Ros Node to interact with the RASA server."""

    def __init__(self):
        """Just init the node."""
        self.user_messanger = UserMessanger()
    
    def _set_id_username_slot(self, username, id):
        """ Sets id and name of the user through post request
        """
        if username is "":
            username = None
        
        url = 'http://localhost:5002/conversations/bot/tracker/events?include_events=NONE'
        msg = ({"event":"slot","name":"slot_id","value":str(id),"timestamp":0},{"event":"slot","name":"username","value":username,"timestamp":0})
        self.user_messanger.setID(id)
        self.user_messanger.setUsername(username)

        # set slots
        r = requests.post(url, json=msg)    

    def _get_id_username_slot(self):
        """
        Returns id and name from the bot through a get request
        """
         # get info slot username
        url = "http://localhost:5002/conversations/bot/tracker"
        r1 = requests.get(url=url)
        username = r1.json()["slots"]["username"]
        id = r1.json()["slots"]["slot_id"] 
        
        return username, id

    def _handle_dialogue(self,req):
        """
        Allows comunication with the bot. sets name and label of the user . Returns response, 
        label and names of the user.
        """
        input_text = req.input_text  
        username = req.username
        id = req.id

        # Get answer        
        get_answer_url = 'http://localhost:5002/webhooks/rest/webhook'
        message = {
            "sender": 'bot',
            "message": input_text
        }

        # If there is a new user in the scene  we set his id and username 
        #( an user is new if before there is no user or the last user is different)
        self._set_id_username_slot(username,id)        
        # send message
        r = requests.post(get_answer_url, json=message)
    
        response = DialogueResponse()
        response.answer = ""
        response.bot_username = ""
        response.bot_id = -1
        #get answer
        for i in r.json():
            response.answer += i['text'] + ' ' if 'text' in i else ''

        username,id = self._get_id_username_slot()
        # if user is defined and arrived a goodbye message means that no user information should be stored
        if (self.user_messanger.isUserDefined()):
            if username is None:
                self.user_messanger.reset()

        # return everytime username information 
        if username is not None:
            response.bot_username = username
        if id is not None:
            response.bot_id = int(id)
  
        return response

    def start(self):
        rospy.init_node('dialogue_service')
        rospy.Service('dialogue_server', Dialogue, self._handle_dialogue)
        rospy.logdebug('Dialogue server READY.')
        print("""
______  ___   _____  ___   ______ _____  ___ ________   __
| ___ \/ _ \ /  ___|/ _ \  | ___ \  ___|/ _ \|  _  \ \ / /
| |_/ / /_\ \\ `--./ /_\ \ | |_/ / |__ / /_\ \ | | |\ V / 
|    /|  _  | `--. \  _  | |    /|  __||  _  | | | | \ /  
| |\ \| | | |/\__/ / | | | | |\ \| |___| | | | |/ /  | |  
\_| \_\_| |_/\____/\_| |_/ \_| \_\____/\_| |_/___/   \_/  
                                                          
                                                          """)
        print('[CHATBOT INTERFACE] ready.')
        rospy.spin()
    
if "__main__" == __name__:
    try:
        d = DialogueServer()
        d.start()
    except rospy.ROSInterruptException as e:
        pass
        