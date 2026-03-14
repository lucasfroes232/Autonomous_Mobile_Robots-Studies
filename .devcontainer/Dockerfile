FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# 1. Dependências de sistema e utilitários (sudo adicionado)
RUN apt-get update && apt-get install -y \
    software-properties-common curl gnupg2 lsb-release sudo \
    python3 python3-pip python3-dev \
    build-essential cmake git \
    terminator nano gedit \
    libgl1 libglib2.0-0 libxext6 libxrender1 libxtst6 python3-tk \
    && rm -rf /var/lib/apt/lists/*

# 2. Configuração do PPA do ROS 2 (fornecido pela MRS) e instalação
RUN curl -sSL https://ctu-mrs.github.io/ppa2-stable/add_ros_ppa.sh | bash && \
    apt-get update && apt-get install -y \
    ros-jazzy-desktop-full \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# 3. Configuração do PPA da MRS e instalação do uav_system
RUN curl -sSL https://ctu-mrs.github.io/ppa2-stable/add_ppa.sh | bash && \
    apt-get update && apt-get install -y \
    ros-jazzy-mrs-uav-system-full \
    && rm -rf /var/lib/apt/lists/*
