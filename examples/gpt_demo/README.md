# QTrobot GPT-Demo

-# Implementing a simple speech-based chatbot with OpenAI and QTrobot

## Requirements 
- *qt_gspeech_app* running for speech recognition, check setup insturctions [here](https://github.com/luxai-qtrobot/software/tree/master/apps/qt_gspeech_app)
- *openai credentials* for accessing OpenAI GPT models 

## Installation 
Install the required python packages:

```
sudo pip3 install -r requirements.txt
```

## Configuration
Add OpenAI key to config.json file:

```bash
"OPENAI_KEY":""
```

To use OpenAI *gpt-3.5-turbo* model set *OPENAI_GPT_MODEL* to True or if you want to use *text-davinci-003* set it to False.

### Build 
make a link (or copy) to `gpt_demo` in catkin workspace source directory and build it. for example, 

```
$ cd ~/catkin_ws/src
$ ln -s ~/robot/code/tutorials/examples/gpt_demo ./
$ cd ~/catkin_ws/
$ catkin_make
```

## Lunching QTrobot GPT Demo rosnode
```
$ roslaunch gpt_demo gpt_demo.launch
```

