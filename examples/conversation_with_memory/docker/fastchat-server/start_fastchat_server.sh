#!/bin/bash


echo "Running fastchat.serve.controller..."
python3 -m fastchat.serve.controller &
PID1=$!


echo "Running fastchat.serve.model_worker with $MODEL_NAME"
yes y | python3 -m fastchat.serve.model_worker --model-path /data/$MODEL_NAME &
PID2=$!

python3 -m fastchat.serve.openai_api_server --host localhost --port 6000 &
PID3=$!

echo "waiting ..."
wait $PID1 $PID2 $PID3
#kill $PID1 $PID2 $PID3
#sleep 5
#kill -9 $PID1 $PID2 $PID3
echo "stopped"
