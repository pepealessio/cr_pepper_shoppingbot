from typing import Any, Text, Dict, List

from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import SlotSet

import pickle
import logging
from word2number import w2n
from pattern.text.en import singularize
import pathlib

logging.getLogger(__name__)


ONLY_BOT = False
FILE_PATH = str(pathlib.Path(__file__).parent.resolve()) + "/../database/data.pk"


def hash_functions(s):
    i =0
    for j in range(len(s)):
        i+=ord(s[j])
    return i


def load_data(file_path=FILE_PATH):
    """If the data exist load the data from a db (represented by a dict(str[name] -> int[quantity])).
    If not, return an empty dict."""
    try:
        with open(file_path,'rb') as fh:
            data = pickle.load(fh)
    except FileNotFoundError:
        data = {}
    
    return data


def save_data(data, file_path=FILE_PATH):
    """Save the data in the datapath prodided."""
    with open(file_path, 'wb') as f:
        pickle.dump(data,f)
    

class ActionShow(Action):
    """RASA Action. Load the data from the database; compose the message in string;
    send a message to the user."""

    def name(self) -> Text:
        return "action_show"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        data = None
        data_all_user = load_data()
        
        username = tracker.get_slot("username")
        username = username.lower()

        if not ONLY_BOT:
            id = int(tracker.get_slot("slot_id"))
        else:
            id = hash_functions(username)
        
        if username is None:
            dispatcher.utter_message(text="Please give me your name") 
            return []

        if data_all_user.get(id) is not None: #is not empty
            # Compose the show message
            data = data_all_user[id]
            
            text = "Your shopping list:\n "
            for k in data.keys():
                v = data[k]
                text += str(v) + " " + k +  ","       
        else:
            text = "Your shopping list is empty!"      
        
        dispatcher.utter_message(text=text) 
        return [SlotSet("item_name", None), SlotSet("item_quantity", None)]
        
     
class ActionAdd(Action):
    """RASA action. Recive the item quantity and the item name. Load the data; update the data; 
    save the data; send an info message.

    Attention: item_quantity can be digits or word. That will be converted in digits with
    the word2num package.
    Attention: item_name can be singular or plural. That will be converted in singular with
    the singularize metod from pattern. Only English is supported."""

    def name(self) -> Text:
        return "action_add"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        try:
            item_quantity = w2n.word_to_num(tracker.get_slot("item_quantity"))
        except ValueError:
            # Managment when NLU detect a word that is not a number like item_quantity
            dispatcher.utter_message(text="Please can you reprhase?") 
            return [SlotSet("item_name", None), SlotSet("item_quantity", None)]

        item_name = singularize(tracker.get_slot("item_name")) # item name be converted in singular
                  
        data = load_data(FILE_PATH)

        username = tracker.get_slot("username")
        if username is None:
            dispatcher.utter_message(text="Please give me your name") 
            return []
        username = username.lower()

        if not ONLY_BOT:
            id = int(tracker.get_slot("slot_id"))
        else:
            id = hash_functions(username)

        if data.get(id) is None:
            data[id] = {}

        try:
            data[id][item_name] += item_quantity
        except KeyError:
            data[id][item_name] = item_quantity

        save_data(data)

        msg = f"The item {item_name} is correctly added to your shopping list. Now you have {data[id][item_name]} {item_name}"
        
        dispatcher.utter_message(text=msg) 
        return [SlotSet("item_name", None), SlotSet("item_quantity", None)]


class ActionRemove(Action):
    """RASA action. Recive the item quantity and the item name. Load the data; update the data; 
    save the data; send an info message.

    Attention: item_quantity can be digits or word. That will be converted in digits with
    the word2num package.
    Attention: item_name can be singular or plural. That will be converted in singular with
    the singularize metod from pattern. Only English is supported."""

    def name(self) -> Text:
        return "action_remove"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        try:
            item_quantity = w2n.word_to_num(tracker.get_slot("item_quantity"))
        except ValueError:
            # Managment when NLU detect a word that is not a number like item_quantity
            dispatcher.utter_message(text="Please can you reprhase?") 
            return [SlotSet("item_name", None), SlotSet("item_quantity", None)]

        item_name = singularize(tracker.get_slot("item_name")) # item name be converted in singular
        data = load_data()

        username = tracker.get_slot("username")
        if username is None:
            dispatcher.utter_message(text="Please give me your name") 
            return []
        username = username.lower()

        if not ONLY_BOT:
            id = int(tracker.get_slot("slot_id"))
        else:
            id = hash_functions(username)

        try:
            data[id][item_name] -= item_quantity
            if data[id][item_name] <= 0:
                del data[id][item_name]
                msg = f"Now in your shopping list there are no {item_name} left"
            else:
                msg = f"In your shopping list there are {data[id][item_name]} {item_name} left"

        except KeyError:
            msg = f"In your shopping list there isn't any {item_name} items"

        save_data(data)
        
        dispatcher.utter_message(text=msg)
    
        return [SlotSet("item_name",None),SlotSet("item_quantity",None)]
        

class ActionUpdate(Action):
    """RASA action. Recive the item quantity and the item name. Load the data; update the data; 
    save the data; send an info message.

    Attention: item_quantity can be digits or word. That will be converted in digits with
    the word2num package.
    Attention: item_name can be singular or plural. That will be converted in singular with
    the singularize metod from pattern. Only English is supported."""

    def name(self) -> Text:
        return "action_update"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        try:
            item_quantity = w2n.word_to_num(tracker.get_slot("item_quantity"))
        except ValueError:
            # Managment when NLU detect a word that is not a number like item_quantity
            dispatcher.utter_message(text="Please can you reprhase?") 
            return [SlotSet("item_quantity", None)]

        item_name = singularize(tracker.get_slot("item_name")) # item name be converted in singular
        data = load_data(FILE_PATH)

        username = tracker.get_slot("username")
        if username is None:
            dispatcher.utter_message(text="Please give me your name") 
            return []
        username = username.lower()

        if not ONLY_BOT:
            id = int(tracker.get_slot("slot_id"))
        else:
            id = hash_functions(username)

        if data.get(id) is None:
            data[id] = {}
        
        data[id][item_name] = item_quantity

        save_data(data)

        msg = f"The item {item_name} is correctly updated to your shopping list. Now you have {data[id][item_name]} {item_name}"
        
        dispatcher.utter_message(text=msg) 
        return [SlotSet("item_name", None), SlotSet("item_quantity", None)]


class ActionEmpty(Action):
    """RASA action. Recive the username. Load the data; clean out the data of the user; 
     send an info message.
    """

    def name(self) -> Text:
        return "action_empty"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        data = load_data()

        username = tracker.get_slot("username")
        if username is None:
            dispatcher.utter_message(text="Please give me your name") 
            return []
        username = username.lower()

        if not ONLY_BOT:
            id = int(tracker.get_slot("slot_id"))
        else:
            id = hash_functions(username)
        
        try:
            data[id].clear()
            msg = f"Your list is empty"
        except KeyError:
            msg = f"Your list is empty"

        save_data(data)
        
        dispatcher.utter_message(text=msg)
    
        return [SlotSet("item_name",None),SlotSet("item_quantity",None)]
