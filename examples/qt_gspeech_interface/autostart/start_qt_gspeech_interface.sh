# !/bin/bash

source /home/qtrobot/robot/autostart/qt_robot.inc
SCRIPT_NAME="start_qt_gspeech_interface"
LOG_FILE=$(prepare_logfile "$SCRIPT_NAME")

{
prepare_ros_environment
wait_for_ros_node "/rosout" 60

exec echo "qtrobot" | sudo -kSi GOOGLE_APPLICATION_CREDENTIALS="/home/qtrobot/robot/code/tutorials/examples/qt_gspeech_interface/service.json" PYTHONPATH=${PYTHONPATH}:/home/qtrobot/.local/lib/python3.5/site-packages/:/home/qtrobot/catkin_ws/devel/lib/python2.7/dist-packages/  python3 /home/qtrobot/robot/code/tutorials/examples/qt_gspeech_interface/src/qt_gspeech_service.py

} &>> ${LOG_FILE}

