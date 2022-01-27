#!/usr/bin/env python3
import json
from ros_chatbot.srv import Dialogue, DialogueResponse

import rospy
import requests

class UserMessanger():
    
    def __init__(self,username="",id=-1,text=''):
        self.username = username
        self.id = id
        self.text = text

    def setID(self,id):
        self.id = id

    
    def setUsername(self,username):
        self.username = username
    
    def getID(self):
        return self.id

    def getUsername(self):
        return self.username
    
    
    def isUsernameDefined(self):
        return "" != self.username

    def reset(self):
        self.username=""
        self.id=-1
    
    

user_messanger = UserMessanger()

def handle_service(req):
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
    if (not user_messanger.isUsernameDefined() and username != '') or (user_messanger.getID()!=id and username != ''):

        url_test = 'http://localhost:5002/conversations/bot/tracker/events?include_events=NONE'
        
        msg = ({"event":"slot","name":"slot_id","value":str(id),"timestamp":0},{"event":"slot","name":"username","value":username,"timestamp":0})
        user_messanger.setID(id)
        user_messanger.setUsername(username)
        # set slots
        r1 = requests.post(url_test, json=msg)
    
    # send message
    r = requests.post(get_answer_url, json=message)
  
    response = DialogueResponse()
    response.answer = ""
    response.bot_username = ""
    response.bot_id = -1
    #get answer
    for i in r.json():
        response.answer += i['text'] + ' ' if 'text' in i else ''

    # get info slot username
    url = "http://localhost:5002/conversations/bot/tracker"
    r1 = requests.get(url=url)
    username = r1.json()["slots"]["username"]
    id = r1.json()["slots"]["slot_id"] #check if it is an int
    # if user is defined and arrived a goodbye message means that no user information should be stored
    if (user_messanger.isUsernameDefined()):
        if username is None:
            user_messanger.reset()

    # return everytime username information 
    if username is not None:
        response.bot_username = username
    if id is not None:
        response.bot_id = int(id)

  
    return response

def main():
    # Server Initialization
    rospy.init_node('dialogue_service')

    s = rospy.Service('dialogue_server',
                        Dialogue, handle_service)

    rospy.logdebug('Dialogue server READY.')
    rospy.spin()


if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException as e:
        pass

