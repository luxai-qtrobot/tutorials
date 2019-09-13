## Microphone - Voice Interaction

Check it in this video:
---
[![Voice Interaction](http://img.youtube.com/vi/ol54IzyZOR0/0.jpg)](http://www.youtube.com/watch?v=ol54IzyZOR0 "Far Field Microphone Array - Voice Interaction")


NOTICE: You must read and fully understand Snips voice system before starting this!!!!

Please read the documentation of snips to understand it https://console.snips.ai/login

This demo is already installed on your QTrobot. you can find it under ~/robot/code/qt_app

*Snips already installed on QTRP (head)*

* how to run the snip servers:
```
sudo systemctl start snips-asr.service snips-audio-server.service  snips-dialogue.service snips-hotword.service snips-nlu.service
```
* how to run qt_voice_app demo
```
rosrun qt_voice_app qt_voice_app.py
```


* Voice commands
- Hey QT, show me your happy emotion
- Hey QT, play happy gesture

* Snips configuration file
```
/etc/snips.toml
```
