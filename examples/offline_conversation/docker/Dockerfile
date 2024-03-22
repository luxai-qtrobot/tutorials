# syntax=docker/dockerfile:1

# base python image for custom image
# FROM python:3.8.10-slim-buster
FROM nvcr.io/nvidia/l4t-pytorch:r35.2.1-pth2.0-py3

ENV SHELL /bin/bash
ENV LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1


# create workdir
WORKDIR /fastchat-server
COPY requirements.txt requirements.txt
COPY start_fastchat_server.sh start_fastchat_server.sh

# install requirements
#ENV TORCH_INSTALL=https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
#RUN apt update
#RUN apt install -y gcc python3-dev
#RUN python3 -m pip install --upgrade pip
#RUN python3 -m pip install numpy
#RUN python3 -m pip install --no-cache https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl

RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install -r requirements.txt

COPY . .
CMD /fastchat-server/start_fastchat_server.sh

