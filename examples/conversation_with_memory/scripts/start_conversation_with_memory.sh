#!/bin/bash

source ~/catkin_ws/src/conversation_with_memory/.venv/bin/activate

echo "Running GPT Demo Rag..."
python3 ~/catkin_ws/src/conversation_with_memory/src/GPTBot.py &
PID1=$!


echo "waiting ..."
wait $PID1 
kill $PID1 


sleep 5
kill -9 $PID1 
echo "stopped"