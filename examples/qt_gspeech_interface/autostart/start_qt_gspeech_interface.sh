# !/bin/bash

source /home/qtrobot/robot/autostart/qt_robot.inc
export GOOGLE_APPLICATION_CREDENTIALS="/home/qtrobot/robot/code/tutorials/examples/qt_gspeech_interface/instance/service.json"

SCRIPT_NAME="start_qt_gspeech_interface"
LOG_FILE=$(prepare_logfile "$SCRIPT_NAME")

{
prepare_ros_environment
wait_for_ros_node "/rosout" 60
rosrun qt_gspeech_interface qt_gspeech_service.py

} &>> ${LOG_FILE}
