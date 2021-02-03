# QTrobot Google Speech-To-Text ROS Service

!!! question ""
    Get the full code in our [github tutorial repository.](ttps://github.com/luxai-qtrobot/tutorials/blob/master/examples/qt_gspeech_interface/README.md)

!!! warning "Notice"
    *Everything should be installed on QTRP (head) - 192.168.100.1*

----
## **1. QTrobot with pre-installed google speech interface**

!!! warning "Notice"
    *Please follow the instruction bellow if your QTrobot came with pre-installed google speech recognition*

### ***1.1 Setup Google Cloud and download your private key***

 - Follow the 1. Step from [instructions](https://cloud.google.com/speech-to-text/docs/quickstart-client-libraries#before-you-begin) to setup your Google Console and SpeechToText
 - From 1.Step ***Download a private key as JSON*** and save it inside ***"~/robot/code/tutorials/examples/qt_gspeech_interface"*** folder you will need it to run the application.

### ***1.2 Connect to QTrobot hotspot and ssh to QTrobot and initalize Google cloud***

```bash
ssh qtrobot@192.168.100.1
```

```bash
gcloud init
```

!!! info ""
    When initializing google cloud just follow the steps from command line (login to your google account, select your google-cloud-server)

### ***1.3 Enable qt_gspeech_interface***

```bash
sudo nano ~/robot/autostart/autostart_screens.sh
```

!!! info ""
    Uncomment the last line in autostart script "run_script "start_qt_gspeech_interface.sh"". Save it (Ctrl+O), exit (Ctrl+X) and reboot the QTrobot.

### ***1.4 Running the QTrobot Google Speech Ros service***

- **To check if it is running you can list all rosservices**

```bash
rosservice list
```

!!! info ""
    You should see /qt_robot/speech/recognize

- **To run:**

```bash
rosservice call /qt_gspeech_service "language:'' options: - '' timeout:10"
```

----

## **2. Full installation of google speech interface**

-  **Connect to QTrobot hotspot and ssh to QTrobot**
```bash
ssh qtrobot@192.168.100.1
```

### ***2.1 Prepare QTrobot (get files and setup the environment)***


- **Clone the github repository into "robot/code" folder**

```bash
cd ~/robot/code && git clone https://github.com/luxai-qtrobot/tutorials.git
```


- **Copy the autostart script to autostart folder**

```bash
cp ~/robot/code/tutorials/examples/qt_gspeech_interface/autostart/start_qt_gspeech_interface.sh ~/robot/autostart
```

- **Enable the qt_gspeech_interface in autostart_screen.sh**

```bash
sudo nano ~/robot/autostart/autostart_screens.sh
```

```bash
run_script "start_qt_gspeech_interface.sh"
```

!!! info ""
    Below the last command *"run_script"* copy and paste the command above. Save it (Ctrl+O) and exit (Ctrl+X).


### ***2.2 Install python3 virtualenv and portaudio***

```bash
sudo apt-get update && sudo apt-get install python3-venv portaudio19-dev
```

### ***2.3 Create Python3 virtualenv and install requirements***

!!! info ""
    Install everything inside project folder (qt_gspeech_interface)

```bash
python3 -m venv .venv && source .venv/bin/activate
```

```bash
pip3 install --upgrade "pip < 21.0"
```

```bash
pip3 install -r requirements.txt
```



### ***2.4 Setup Google Cloud***

 - Follow the [instructions](https://cloud.google.com/speech-to-text/docs/quickstart-client-libraries) to setup your Google Console and SpeechToText
 - From 1.Step ***Download a private key as JSON*** and save it inside ***"~/robot/code/tutorials/examples/qt_gspeech_interface"*** folder you will need it to run the application.
 - Install [Google Cloud SDK](https://cloud.google.com/sdk/docs/install) and Google Cloud Python SDK (4.step)

### ***2.4 Link your folder and build catkin qt_gspeech_interface package***

```bash
cd ~/catkin_ws/src && ln -s ~/robot/code/tutorials/examples/qt_gspeech_interface .
```

```bash
cd ~/catkin_ws && catkin_make --pkg qt_gspeech_interface
```

### ***2.5 Running the QTrobot Google Speech Ros service***

!!! info ""
    Make sure that you have uncommented "run_script "start_qt_gspeech_interface.sh"" in autostart_screen.sh.
    After that just reboot the QTrobot.

- **To check if it is running you can list all rosservices**

```bash
rosservice list
```

!!! info ""
    You should see /qt_robot/speech/recognize

- **To run:**

```bash
rosservice call /qt_gspeech_service "language:'' options: - '' timeout:10"
```

### ***2.6 Additional***

*If you want to call this service outside of QTRP environment then you should copy content of qtpc_gspeech to your catkin workspace*

- **copy qtpc_gspeech to your PC or QTPC**

!!! info ""
    These commands include "robot/code" folder just for the purpose of the example. Include your folder name in the command.

```bash
cd ~/robot/code && git clone https://github.com/luxai-qtrobot/tutorials.git
```

```bash
cd ~/catkin_ws/src/ && ln -s ~/robot/code/tutorials/examples/qt_gspeech_interface/qtpc_gspeech qt_gspeech_interface
```

```bash
cd ~/catkin_ws && catkin_make --pkg qt_gspeech_interface
```
