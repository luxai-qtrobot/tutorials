# !/bin/bash

source /home/qtrobot/robot/autostart/qt_robot.inc

SCRIPT_NAME="start_riva_server"
LOG_FILE=$(prepare_logfile "$SCRIPT_NAME")

{

cd ~/robot/riva_quickstart_arm64_v2.14.0
bash ./riva_start.sh ./config.sh -s

} &>> ${LOG_FILE}
