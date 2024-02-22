FROM nvidia/cuda:11.7.1-cudnn8-devel-ubuntu20.04

ENV PYTHONUNBUFFERED=TRUE \
    PYTHONDONTWRITEBYTECODE=TRUE \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=utility,compute,graphics \
    PATH=$PATH:/opt/conda/bin \
    ROS_DISTRO=noetic \
    DEBIAN_FRONTEND=noninteractive

# Cuda Arch 61 - 10X0 / TITAN Xp
ENV TCNN_CUDA_ARCHITECTURES 61

## Set timezone as it is required by some packages.
RUN apt-get update && apt-get install -y --no-install-recommends \
        tzdata \
        locales \
        apt-utils && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get purge --auto-remove && \
    apt-get clean

RUN ln -fs /usr/share/zoneinfo/Europe/Budapest /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata

RUN locale-gen en_US.UTF-8

ENV TZ=Europe/Budapest \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en

# Set the ROS root directory
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

# Install required packages
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

# Add ROS repository and key
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && chmod 644 /etc/apt/sources.list.d/ros-latest.list

# Add Gazebo repository and key
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-latest.list \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key D2486D2DD83DB69272AFE98867170598AF249743 \
    && chmod 644 /etc/apt/sources.list.d/gazebo-latest.list

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        git \
        python3-catkin-tools \
        python3-pip \
        ros-$ROS_DISTRO-foxglove-bridge \
        nano \
        curl \
        wget \
        net-tools \
        mesa-utils \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        x11-apps \
        ros-${ROS_DISTRO}-rviz \
        ros-${ROS_DISTRO}-rqt \
        ros-${ROS_DISTRO}-rqt-common-plugins \
        ros-${ROS_DISTRO}-desktop-full \
        gazebo11 \
        ros-${ROS_DISTRO}-gazebo-ros-pkgs \
        ros-${ROS_DISTRO}-gazebo-ros-control \
        python3-rosdep \
        firefox \
        libyaml-cpp-dev \
        libpcap-dev \
        libnvinfer8 \
        libnvinfer-plugin8 && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get purge --auto-remove && \
    apt-get clean

RUN cd /usr/lib/x86_64-linux-gnu && \
    ln -s libnvinfer.so.8 libnvinfer.so.7 && \
    ln -s libnvinfer_plugin.so.8 libnvinfer_plugin.so.7

# Add the source setup.bash command to the user's bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

# Uncomment and run rosdep update and rosdep install if needed
RUN rosdep init && rosdep update --rosdistro $ROS_DISTRO

# Create a Catkin workspace
RUN /bin/bash -c '. /opt/ros/${ROS_DISTRO}/setup.bash; mkdir -p ~/catkin_ws/src; cd ~/catkin_ws; catkin_make'

# Set the workspace as the working directory
WORKDIR /root/catkin_ws

# Source the workspace in each shell
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /root

ENV PIP_ROOT_USER_ACTION=ignore

ENV CUDA_HOME=/usr/local/cuda-11.7
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}
ENV LIBRARY_PATH=${CUDA_HOME}/lib64/stubs:${LIBRARY_PATH}

RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
    chmod +x Miniconda3-latest-Linux-x86_64.sh && \
    mkdir /root/.conda && \
    bash Miniconda3-latest-Linux-x86_64.sh -b && \
    rm -f Miniconda3-latest-Linux-x86_64.sh

ENV PATH="/root/miniconda3/bin:${PATH}"
ARG PATH="/root/miniconda3/bin:${PATH}"

RUN conda update -y conda

RUN conda create -n nerfstudio -y python=3.8

RUN conda run --no-capture-output -n nerfstudio python -m pip install --upgrade pip

RUN conda run --no-capture-output -n nerfstudio pip install torch==2.0.1+cu117 torchvision==0.15.2+cu117 --extra-index-url https://download.pytorch.org/whl/cu117

RUN conda run --no-capture-output -n nerfstudio conda install -c "nvidia/label/cuda-11.7.1" cuda-toolkit

RUN conda run --no-capture-output -n nerfstudio pip install ninja git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch

RUN conda run --no-capture-output -n nerfstudio pip install nerfstudio

RUN mkdir /root/code

WORKDIR /root/code