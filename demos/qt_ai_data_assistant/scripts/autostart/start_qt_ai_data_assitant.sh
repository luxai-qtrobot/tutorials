# !/bin/bash

source /home/qtrobot/robot/autostart/qt_robot.inc

SCRIPT_NAME="start_qt_ai_data_assitant"
LOG_FILE=$(prepare_logfile "$SCRIPT_NAME")

{
prepare_ros_environment
wait_for_ros_node "/rosout" 60
wait_for_ros_topic "/qt_robot/emotion/show" 60
wait_for_ros_topic "/qt_robot/gesture/play" 60
wait_for_ros_topic "/qt_robot/head_position/command" 60

echo "waiting for some time for Riva Server docker to be available..."
sleep 100
echo "Running qt_ai_data_assistant..."

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1

cd ~/robot/code/tutorials/demos/qt_ai_data_assistant/
source venv/bin/activate
python src/qt_ai_data_assistant.py --enable-scene

} &>> ${LOG_FILE}

