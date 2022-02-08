# Cognitive Robotics

Final project - Pepper list assistant

In this file you can see how to launch all things described in the file report.pdf.

## Application Demo
[![Application Demo](https://img.youtube.com/vi/UyyMqou_CBE/0.jpg)](https://www.youtube.com/watch?v=UyyMqou_CBE "Application Demo")

## Authors:
* Alessio Pepe - 0622701463 - a.pepe108@studenti.unisa.it
* Alfonso Comentale - 0622701438 - a.comentale9@studenti.unisa.it
* Giuseppina Di Paolo - 0622701510 - g.dipaolo5@studenti.unisa.it
* Teresa Tortorella - 0622701507 - t.tortorella3@studenti.unisa.it

## How to run the application

1. Find the michrophone index using the code 
    ```
    import sounddevice as sd
    sd.query_devices()
    ```

2. Set the index just found in the config file (line 41) as:
    ```
    # pyAudio microphone idex. That need to be changed basing on the device. 
    MICROPHONE_INDEX = 2
    ```

3. Launch the chatbot server. This take about 60 seconds to start.
    ```
    # go in your ros workspace
    source devel/setup.bash
    roslaunch ros_chatbot chatbot.launch
    ```

4. Launch the pepper nodes.
    ```
    # go in your ros workspace
    source devel/setup.bash
    roslaunch pepper_nodes pepper.launch
    ```

4. Launch the core application (wait until the other two are ready).
    ```
    # go in your ros workspace
    source devel/setup.bash
    roslaunch fp_audio core.launch
    ```

Alternatively to the step 3, 4, and 5, you can launch as

```
# go in your ros workspace
source devel/setup.bash
roslaunch fp_audio all.launch
```

## How to run the application (simulating pepper)
1. Set the index as default in the config file (line 41) as:
    ```
    # pyAudio microphone idex. That need to be changed basing on the device. 
    MICROPHONE_INDEX = None
    ```
    and, at line 45,
    ```
    # Flag: If False, the services who need Pepper are not called. Useful for debug.
    # -- In the use must be True.
    ON_PEPPER = False
    ```

2. Launch the chatbot server. This take about 60 seconds to start.
    ```
    # go in your ros workspace
    source devel/setup.bash
    roslaunch ros_chatbot chatbot.launch
    ```

3. Launch the core application that simulate the pepper webcam (wait until the other two are ready).
    ```
    # go in your ros workspace
    source devel/setup.bash
    roslaunch fp_audio core_s.launch
    ```

## How to run the tests

To run the re-identification test, use the following commands:

```
# go in your ros workspace
source devel/setup.bash
roslaunch fp_audio test_reidentification.launch
```

To run the detector test, use the following commands:

```
# go in your ros workspace
source devel/setup.bash
roslaunch fp_audio test_detector.launch
```

To run th integration test ROS and RASA

```
# go in your ros workspace
source devel/setup.bash
roslaunch ros_chatbot dialogue.xml
```
