#!/usr/bin/env python3

import rospy
from ros_chatbot.srv import Dialogue, DialogueResponse

class TerminalInterface:
    '''Class implementing a terminal i/o interface. 

    Methods
    - get_text(self): return a string read from the terminal
    - set_text(self, text): prints the text on the terminal

    '''

    def get_text(self):
        return input("[IN]:  ") 

    def set_text(self,text):
        print("[OUT]:",text)

def main():
    rospy.init_node('writing')
    rospy.wait_for_service('dialogue_server')
    dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)

    terminal = TerminalInterface()

    while not rospy.is_shutdown():
        message = terminal.get_text()
        user = "alfonso"
        id = 1
        if message == 'exit': 
            break
        try:
            bot_answer = dialogue_service(message,user,id)
            terminal.set_text(bot_answer.answer)
            terminal.set_text(bot_answer.bot_username)
            terminal.set_text(bot_answer.bot_id)

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass
