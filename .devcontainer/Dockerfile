FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# 1. Dependências de sistema e utilitários (apenas pacotes padrão do Ubuntu)
RUN apt-get update && apt-get install -y \
    software-properties-common curl gnupg2 lsb-release sudo \
    python3 python3-pip python3-dev \
    build-essential cmake git libasio-dev \
    terminator nano gedit \
    libgl1 libglib2.0-0 libxext6 libxrender1 libxtst6 python3-tk \
    && rm -rf /var/lib/apt/lists/*

# 2. Configuração do PPA do ROS 2, instalação básica e dependências do Scout
RUN curl -sSL https://ctu-mrs.github.io/ppa2-stable/add_ros_ppa.sh | bash && \
    apt-get update && apt-get install -y \
    ros-jazzy-desktop-full \
    ros-dev-tools \
    python3-rosdep \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-xacro \
    ros-jazzy-ros-gz \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-ament-lint-auto \
    ros-jazzy-ament-lint-common \
    && rm -rf /var/lib/apt/lists/*

# 3. Configuração do PPA da MRS e instalação do uav_system
RUN curl -sSL https://ctu-mrs.github.io/ppa2-stable/add_ppa.sh | bash && \
    apt-get update && apt-get install -y \
    ros-jazzy-mrs-uav-system-full \
    && rm -rf /var/lib/apt/lists/*

# 4. Inicialização do rosdep no sistema base
RUN rosdep init || true

# 5. Compilação do TurtleBot4 em um workspace isolado (/opt/turtlebot4_ws)
RUN mkdir -p /opt/turtlebot4_ws/src && \
    cd /opt/turtlebot4_ws/src && \
    git clone https://github.com/turtlebot/turtlebot4.git && \
    git clone https://github.com/turtlebot/turtlebot4_simulator.git && \
    cd /opt/turtlebot4_ws && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src -yi && \
    /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install" && \
    rm -rf /var/lib/apt/lists/*

# 6. Compilação do Scout em um workspace isolado (/opt/scout_ws)
RUN mkdir -p /opt/scout_ws && \
    cd /opt/scout_ws && \
    git clone https://github.com/vivaldini/scout_ws.git . && \
    apt-get update && \
    rosdep update || true && \
    rosdep install --from-paths src -yi --skip-keys "catkin gazebo_ros" && \
    /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install" && \
    rm -rf /var/lib/apt/lists/*
