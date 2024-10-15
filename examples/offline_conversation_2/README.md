# QTrobot Offline Conversation 2

*===========================================================================================*
**[NOTICE]**

*This demo is outdated and is maintained for learning purposes only* 

*For the latest version, please refer to [QTrobot: Your AI Data Assistant](../../demos/qt_ai_data_assistant).* 

*===========================================================================================*


This simple example demonstrates how to implement human-level conversational interaction with QTrobot using offline ASR and LLM (llama3). Everything used in this demo runs offline on **QTrobot RD-V2 AI@Edge**.

## Requirements:
- **QTrobot RD-V2 AI@Edge**
- **Ollama** with the `llama3` model (e.g., llama3 8b)
- **Nvidia Riva ASR software**: `qt_riva_asr_app` is our ROS wrapper for Nvidia Riva

## Installation
1. Install required python package for the demo
2. Install the Nvidia Riva ASR container.
3. Pull the `dustynv/ollama:r36.2.0` Docker container for running Ollama.
4. Download the `llama3` model using the Ollama interface.



### Install required python pip packages for our demo
1. create a python3 virtual environment and install the packages:
open a terminal and enter the following commands, one by one. 
```bash 
cd ~/robot/code/tutorials/examples/offline_conversation_2
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```



### Installing Nvida Riva ASR software for `qt_riva_asr_app` (IF IT HAS NOT BEEN INSTALED):
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


### Installing Ollama docker container 
1. pull the ollama docker container for Jetson orin:
```bash 
docker pull dustynv/ollama:r36.2.0
```

2. create a folder to download and keep ollama models: 
```bash
mkdir /home/qtrobot/robot/ollama
```

3. run the ollama docer contaner in interactive mode: 
```bash
docker run -it --runtime nvidia --rm --network=host -v /home/qtrobot/robot/ollama:/ollama -v /home/qtrobot/robot/ollama:/data/logs -e OLLAMA_MODELS=ollama dustynv/ollama:r36.2.0
```

4. download the `llama3` model for ollama: 
After running the Ollama Docker container in interactive mode, you can download the llama3 model from within the container. Run the following command in the Ollama container terminal:

```bash
ollama pull llama3
```

the above command, download the `llama3` model and permanently store it in `/home/qtrobot/robot/ollama`. 

5. test the llama3 model using ollama interface
run the follwoing command in ollama container terminal:

```bash 
ollama run llama3 
```
you can chat with the model from terminal.

6. exit to stop the docker
run the follwoing command in ollama container terminal:
```bash
exit
```

## Running QTrobot Offline conversation with Riva ASR:
1. open a termianl and start Riva ASR container. Wait untill it's intialized. It may take 1 to 2 minutes. 
```bash
cd ~/robot/riva_quickstart_arm64_v2.13.0
bash ./riva_start.sh ./config.sh -s
```

2. in another terminal Run `qt_riva_asr_app` node:
```bash
roslaunch qt_riva_asr_app qt_riva_asr_app.launch 
```

3. in another terminal run ollama docker container (IF IT IS NOT ALREADY RUNNING): 
```bash
docker run -it --runtime nvidia --rm --network=host -v /home/qtrobot/robot/ollama:/ollama -v /home/qtrobot/robot/ollama:/data/logs -e OLLAMA_MODELS=ollama dustynv/ollama:r36.2.0
```

4. Run `offline_conversation_2` example:
open another terminal and run the following commands:
```bash
cd ~/robot/code/tutorials/examples/offline_conversation_2
source venv/bin/activate
python main.py
```

