# QTrobot Google Speech-To-Text ROS Service

!!! question ""
    Get the full code in our [github tutorial repository.](ttps://github.com/luxai-qtrobot/tutorials/blob/master/examples/qt_gspeech_interface/README.md)

!!! warning "Notice"
    ***Everything should be installed on QTRP (head) - 192.168.100.1***

## 1.Install python3 modules

```bash
sudo apt-get install python3-pip python3-yaml
pip3 install rospkg pyaudio six
```

## 2.Prepare QTrobot (get files and setup the environment)

-  **Connect to QTrobot hotspot and ssh to QTrobot**

```bash
ssh qtrobot@192.168.100.1
```

- **Clone the github repository into "robot/code" folder**

```bash
cd ~/robot/code
```
```bash
git clone https://github.com/luxai-qtrobot/tutorials.git
```


- **Copy the autostart script to autostart folder**

```bash
cp ~/robot/code/tutorials/examples/qt_gspeech_interface/autostart/start_qt_gspeech_interface.sh ~/robot/autostart
```

- **Enable the script in autostart_screen.sh**

```bash
nano ~/robot/autostart/autostart_screen.sh
```

Below the last "run_script" copy and paste next text:

```bash
run_script "start_qt_gspeech_interface.sh"
```

Save it (Ctrl+O) and exit (Ctrl+X).


## 3.Setup Google Cloud

 - Follow the [instructions](https://cloud.google.com/speech-to-text/docs/quickstart-client-libraries) to setup your Google Console and SpeechToText
 - From 1.Step "Download a private key as JSON." save it inside "~/robot/code/tutorials/examples/qt_gspeech_interface/instance" folder you will need it to run the application.
 - Install [Google Cloud SDK](https://cloud.google.com/sdk/docs/install) and Google Cloud Python SDK (4.step)
 - Install [Client Library](https://cloud.google.com/speech-to-text/docs/quickstart-client-libraries#install_the_client_library)

## 4.Linking your folder and building catkin workspace

```bash
cd ~/catkin_ws/src && ln -s ~/robot/code/tutorials/examples/qt_gspeech_interface .
cd ~/catkin_ws && catkin_make
```

## 5.Running the QTrobot Google Speech Ros service

Make sure that you have uncommented "run_script "start_qt_gspeech_interface.sh"" in autostart_screen.sh.
After that just reboot the QTrobot.

To check if it is running you can list all rosservices and check for "/qt_gspeech_service".

```bash
rosservice list
```

To run:

```bash
rosservice call /qt_gspeech_service "options: - '' timeout:10"
```

For better understanding for speech context you can provide an array of words, like this example:

```bash
rosservice call /qt_gspeech_service "options: - '' timeout:10"
```

## qt_gspeech_headers to catkin workspace
If you want to call this service outside of QTRP environment then you should copy content of qt_gspeech_headers to your catkin workspace

```bash
cd ~/robot/code/tutorials/examples/qt_gspeech_interface/qt_gspeech_headers
cp -r * ~/catkin_ws/devel/
```
