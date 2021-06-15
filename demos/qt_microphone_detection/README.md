## Microphone - Voice Activity & Direction Detection

Check it in this video:
---

[![Voice Activity & Direction Detection](http://img.youtube.com/vi/ua63cgrO0oU/0.jpg)](http://www.youtube.com/watch?v=ua63cgrO0oU "Far Field Microphone Array - Voice Activity & Direction Detection")

NOTICE: You must run this demo on QTRP (head)!!!!

### Setting up and installing the requiremnts  
This demo needs some prerequisites. let's first create python virtual environment to keep everyting clean.
```
python3 -m venv venv
source venv/bin/activate
```
install required packages using pip in your virtual environment 
```
pip install -r requirements.txt
```

To run the demo go to *qt_microphone_detection* folder and run *voice_direction* python script:
```
python voice_direction.py
```
Now you can speak or sing around the QTrobot and he will follow your voice.


### PyUSB permission error 

PyUSB needs root privileges. if you run the above example, you may get the following error:
```
usb.core.USBError: [Errno 13] Access denied (insufficient permissions)
```

To fix this error we need to set up a udev rule file for the microphone to be able to access it with normal user.

* Create a file called *90-mic.rules* in */etc/udev/rules.d/*:

```
sudo nano /etc/udev/rules.d/90-mic.rules
```
* and add the following content
* 
```
ACTION=="add", SUBSYSTEMS=="usb", ATTRS{idVendor}=="2886", ATTRS{idProduct}=="0018", MODE="660", GROUP="plugdev"
```
* Reload udev system to see your changes:
```
sudo udevadm control --reload
sudo udevadm trigger
```

* Reboot QTrobot:
```
sudo reboot
```

