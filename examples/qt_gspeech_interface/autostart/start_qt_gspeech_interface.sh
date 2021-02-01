# !/bin/bash

source /home/qtrobot/robot/autostart/qt_robot.inc
SCRIPT_NAME="start_qt_gspeech_interface"
LOG_FILE=$(prepare_logfile "$SCRIPT_NAME")

{
prepare_ros_environment
wait_for_ros_node "/rosout" 60


read -d '' SPEECHENV << EOF 
source /home/qtrobot/catkin_ws/src/qt_gspeech_interface/.venv/bin/activate;
export GOOGLE_APPLICATION_CREDENTIALS="/home/qtrobot/robot/code/tutorials/examples/qt_gspeech_interface/service.json";
export PYTHONPATH="${PYTHONPATH}:/home/qtrobot/catkin_ws/devel/lib/python2.7/dist-packages";
python /home/qtrobot/catkin_ws/src/qt_gspeech_interface/src/qt_gspeech_service.py;
EOF

exec echo "qtrobot" | sudo -kSi bash -c "$SPEECHENV"

} &>> ${LOG_FILE}



