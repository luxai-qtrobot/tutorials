# Build and use docker for Fastchat server

## building Fastchat server docker 
*If you have already build the Fastchat server docker image, you can ignore this step!*

From the current folder (where Dockerfile located), run the following 

```bash
docker build --tag fastchat-server .
```

## launching Fastchat server docker with vicuna-13b-v1.5 model 
```bash
docker run -d --net=host -e "MODEL_NAME=models--lmsys--vicuna-13b-v1.5" -v /home/qtrobot/robot/huggingface/models:/data fastchat-server
```

## stoping Fastchat server docker 
first find the fastchat-server docker container ID 
```bash 
docker ps | grep fastchat-server
```

stop the docker: 
```bash
docker stop <container ID>
```


## launching Fastchat server docker with other models (e.g. openbmb/MiniCPM) 
- You need to download your model in `/home/qtrobot/robot/huggingface/models` folder if it has not been downloaded. 
- Run the docker and provide the model file name in the paramater

```bash
docker run -d --net=host -e "MODEL_NAME=models--openbmb--MiniCPM-2B-dpo-bf16" -v /home/qtrobot/robot/huggingface/models:/data fastchat-server
```

