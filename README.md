# Cognitive Robotics

Final project - Pepper list assistant

## Unit test

### Audio detector

Requirements:

1. Detect the audio in a controlled environment (with silence) and publish that on a topic.

Tests:

This module don't need unit testing because it's just an adapter for pyAudio.

### Speech 2 Text

Requirements:

1. Received a raw audio wave, convert that in text.

This module don't need unit testing because it's just an adapter for google speech services.

### RASA chatbot

Requirements:

1. The bot have to manage shopping list (insert item, remove item, update an item, show the list, and empty the list), one for each user.

The testing of the chatbot was made in the midterm project.

### Re-identification

Requirements:

1. The module must identify knowed people from an audio waveform.

2. The module must understand if a person is unknown.

3. The module must construct it's people database dinamically.

4. The data of the module need to be persistant.

For each person we use a dataset of recorded audio with pepper micophone.

* The training set contains 5 audio for people.
* The test set contains 2 audio for people (one of that is a greetings).

The test maded are the following:

* [ ] One person (3)
* [ ] Two person (3 - 3)
* [ ] Two person (5 - 3)
* [ ] Three person (5 - 5 - 3)
* [ ] Three person (5 - 5 - 5)
* [ ] Four person (5 - 5 - 5 - 3)
* [ ] Four person (5 - 5 - 5 - 5)

### People detector

Requirements:

1. The module must understand if a person is present in the video stream.

2. The module must manage ghost situation.

Test entering and exiting the scene. Testing also covering the camera for a moment to test detector non properly working.

## Integration Test

### RASA and ROS

Requirements:

1. The chatbot must be called by ROS passing the request text, the person name if known, and the person id.

2. If the person in unknown the chatbot have to ask for an identity (just the name, the id have to be setted externally).

3. If the person is known, the chatbot mustn't ask the identity.

4. The chatbot must reply to the request with a text response and, if knowen, the identity of the person.

Try to call rasa passing the arguments (person_id, person_name and request_text) and get the required input (person_id, person_name and request_text).

### Core application

Using all the nodes

1. This module have to change it's state if a person is present.

2. This module have to enable audio detection if a person is present.

3. This module have to disable audio detection if a person is present.

4. If the person speack, the audio must be use to obtain the text answer (to use the chatbot), to identify (if knowed) the person using the reidentification module, and provide the answer by the Pepper TTS.
