FROM ubuntu:20.04

LABEL maintainer='Suxing Liu, Wes Bonelli'

COPY ./ /opt/code
WORKDIR /opt/code


RUN apt update
RUN DEBIAN_FRONTEND="noninteractive" apt-get install -y \
    build-essential \
    aptitude \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    libboost-all-dev \
    python3-setuptools \
    python3-pip \
    python3 \
    libsm6 \
    libxext6 \
    cmake-gui \
    software-properties-common \
    nano \
    xorg-dev

RUN pip3 install --upgrade pip && \
    pip3 install numpy \
    matplotlib \
    pytest \
    open3d \
    openpyxl \
    PyYAML \
    imutils




RUN chmod +x /opt/code/shcmd.sh

ENV PYTHONPATH=$PYTHONPATH:/opt/code/
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/code/


