# QTrobot Offline Conversation

*===========================================================================================*
**[NOTICE]**

*This demo is outdated and is maintained for learning purposes only* 

*For the latest version, please refer to [QTrobot: Your AI Data Assistant](../../demos/qt_ai_data_assistant).* 

*===========================================================================================*

## Requirements:
 - *Fastchat Vicuna* with offline llm model (e.g. `models--lmsys--vicuna-13b-v1.5`)
 - *Offline ASR* such as:
    - `qt_riva_asr_app` using Nvidia Riva **(recommended)**: this requires *Nvida Riva ASR software*! 
    - `qt_vosk_app` using Vosk (already installed)


## Installing Nvida Riva ASR software for `qt_riva_asr_app`:

1. Signin to your [Nvidia NGC](https://ngc.nvidia.com/signin) portal (signup if you do not have an account)
   
2. Follow the instruction for [Generating Your NGC API Key](https://docs.nvidia.com/ngc/gpu-cloud/ngc-user-guide/index.html#generating-api-key) to obtain an API key.

3. Use your API key to login to the NGC registry by entering the following command and following the prompts. 
   **NOTE:** `ngc` cli is already installed on the QTrobot and you do not need to install it.
```bash
ngc config set
```

4. Download the Embedded Riva start scripts in `~/robot` folder using following command: 
 ```bash
 cd ~/robot/
 ngc registry resource download-version nvidia/riva/riva_quickstart_arm64:2.13.0
 ```

5. Modify `config.sh` in `~/robot/riva_quickstart_arm64_v2.13.0/config.sh` to disable unncessary services and keeps only `service_enabled_asr` enabled. the relevant part in `config.sh` shoul look like this:

```bash
# ...
# Enable or Disable Riva Services
# For any language other than en-US: service_enabled_nlp must be set to false
service_enabled_asr=true
service_enabled_nlp=false
service_enabled_tts=false
service_enabled_nmt=false
# ...
```

6. Initialize the Riva ASR container. This may take some time, depending on the speed of your Internet connection.
```bash
cd ~/robot/riva_quickstart_arm64_v2.13.0
bash ./riva_init.sh
```

7. Restart the robot



## Running QTrobot Offline conversation with Riva ASR:
1. Start Riva ASR container and wait untill it loaded. It may takes 1 to 2 minutes. 
```bash
cd ~/robot/riva_quickstart_arm64_v2.13.0
bash ./riva_start.sh ./config.sh -s
```

2. Run `qt_riva_asr_app` node:
```bash
roslaunch qt_riva_asr_app qt_riva_asr_app.launch 
```

3. Run `offline_conversation` node. this also automaically starts the *Fastchat* server with *vicuna-13b-v1.5* model. (it make take up to a minute to load the model in GPU)
```bash
roslaunch offline_conversation offline_conversation.launch
```


## Running QTrobot Offline conversation with Vosk ASR:

1. Run `qt_vosk_app` node:
```bash
roslaunch roslaunch qt_vosk_app qt_vosk_app.launch
```

2. Run `offline_conversation` node. this also automaically starts the *Fastchat* server with *vicuna-13b-v1.5* model. (it make take up to a minute to load the model in GPU)
```bash
roslaunch offline_conversation offline_conversation.launch
```






