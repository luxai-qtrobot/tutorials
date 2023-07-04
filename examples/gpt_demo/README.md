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
Add OpenAI key to gpt_bot.yaml file:

```bash
"OPENAI_KEY":""
```

To use OpenAI *gpt-3.5-turbo* model set *OPENAI_CHATGPT_MODEL* to True or if you want to use *text-davinci-003* set it to False.

### Character configuration

There are 5 pre-configured characters for 'gpt-3.5-turbo' you can use (qtrobot, fisherman, astronaut, therapist and gollum). Simply change in the 'gpt_bot.yaml' character parameter on line 12 and have fun talking with them.

To use custom character prompt. Change the parameter 'use_prompt' to true and write your 'prompt' instead of the template that is in the 'gpt_bot.yaml'. QTrobot will then take your prompt to get reposnes from GPT.

### Build 
make a link (or copy) to `gpt_demo` in catkin workspace source directory and build it.

```
$ cd ~/catkin_ws/src
$ ln -s ~/robot/code/tutorials/examples/gpt_demo ./
$ cd ~/catkin_ws/
$ catkin_make
```

## Launching QTrobot GPT Demo rosnode
```
$ roslaunch gpt_demo gpt_demo.launch
```

