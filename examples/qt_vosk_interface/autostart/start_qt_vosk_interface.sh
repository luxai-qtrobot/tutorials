# !/bin/bash

source /home/qtrobot/robot/autostart/qt_robot.inc
SCRIPT_NAME="start_qt_vosk_interface"
LOG_FILE=$(prepare_logfile "$SCRIPT_NAME")

{
prepare_ros_environment
wait_for_ros_node "/rosout" 60


read -d '' SPEECHENV << EOF
source /home/qtrobot/robot/autostart/qt_robot.inc;
prepare_ros_environment;
source /home/qtrobot/catkin_ws/src/qt_vosk_interface/venv/bin/activate;
export PYTHONPATH="${PYTHONPATH}:/home/qtrobot/catkin_ws/devel/lib/python2.7/dist-packages";
python /home/qtrobot/catkin_ws/src/qt_vosk_interface/src/qt_vosk_insterface.py;
EOF

exec echo "qtrobot" | sudo -kSi bash -c "$SPEECHENV"

} &>> ${LOG_FILE}
