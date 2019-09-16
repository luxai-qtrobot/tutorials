## Microphone - Voice Activity & Direction Detection

Check it in this video:
---

[![Voice Activity & Direction Detection](http://img.youtube.com/vi/ua63cgrO0oU/0.jpg)](http://www.youtube.com/watch?v=ua63cgrO0oU "Far Field Microphone Array - Voice Activity & Direction Detection")

NOTICE: You must run this demo on QTRP (head)!!!!

This demo needs some prerequisites:
```
sudo apt-get update
sudo pip install pyusb click
```
*OR*
```
sudo apt-get update
sudo `which pip` install pyusb click
```

PyUSB needs root privileges, if you run the script without root you will see something like this:
```
usb.core.USBError: [Errno 13] Access denied (insufficient permissions)
```
To fix this error we need to set up a udev rule file for the microphone to be able to access it with normal user.
* Create a udev rules file:
```
ACTION=="add", SUBSYSTEMS=="usb", ATTRS{idVendor}=="2886", ATTRS{idProduct}=="0018", MODE="660", GROUP="plugdev"
```
Create this file in folder */etc/udev/rules.d/*. For example usual structure of the file name can be *Number-Name.rules*

* Add the user to the *plugdev* group:
```
adduser username plugdev
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

To run the demo go to *qt_microphone_detection* folder and run *voice_direction* python script:
```
cd qt_microphone_detection/
python voice_direction.py
```
Now you can speak or sing around the QTrobot and he will follow your voice.
