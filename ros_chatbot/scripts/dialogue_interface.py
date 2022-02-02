#!/usr/bin/env python3

import rospy
from ros_chatbot.srv import Dialogue

class TerminalInterface:
    """Class implementing a terminal i/o interface. 
    """

    def get_text(self):
        """Take the paraemter from command line.

        Returns:
            Touple[int, str, str]: the label associated to the identity to simulate, 
            the person name to simulate, the request to send to the chatbot.
        """
        return int(input("[IN] id > ")), input('[IN] name >'), input('[IN] text > ')

    def set_text(self, id, name, text):
        """Print the output text and the identity state and the bot answer.

        Args:
            id (int): the label associated to the identity to simulate.
            name (str): the person name to simulate.
            text (str): the request to send to the chatbot.
        """
        print(f"[OUT] id={id}, name={name}, text={text}")

def main():
    """This function start the simulation. Will repeteadly ask for the three parameters 
    and prints the new parameters from the chatbot.
    """
    rospy.init_node('writing')
    rospy.wait_for_service('dialogue_server')
    dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)

    terminal = TerminalInterface()

    while not rospy.is_shutdown():
        id, user, message = terminal.get_text()
        if message == 'exit': 
            break
        try:
            bot_answer = dialogue_service(message,user,id)
            terminal.set_text(bot_answer.bot_id, bot_answer.bot_username, bot_answer.answer)

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass
