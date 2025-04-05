FROM osrf/ros:humble-desktop-full

# Konfiguracja użytkownika (dopasowanie do hosta)
ARG USER_ID=1000
ARG GROUP_ID=1000

SHELL ["/bin/bash", "-c"]

# Setup timezone
ENV TZ=Etc/UTC
RUN echo $TZ > /etc/timezone && \
  ln -fs /usr/share/zoneinfo/$TZ /etc/localtime
  
# Tools necessary and useful during development
RUN apt update && \
 DEBIAN_FRONTEND=noninteractive \
 apt install --no-install-recommends -y \
        build-essential \
        atop \
	ca-certificates \
        cmake \
        cppcheck \
	curl \
        expect \
        gdb \
        git \
	gnupg2 \
        gnutls-bin \
	iputils-ping \
        libbluetooth-dev \
        libccd-dev \
        libcwiid-dev \
	libeigen3-dev \
        libfcl-dev \
        libgflags-dev \
        libgles2-mesa-dev \
        libgoogle-glog-dev \
        libspnav-dev \
        libusb-dev \
	lsb-release \
	net-tools \
	pkg-config \
        protobuf-compiler \
        python3-dbg \
        python3-empy \
        python3-numpy \
        python3-setuptools \
        python3-pip \
        python3-venv \
	ruby \
        software-properties-common \
	sudo \
        vim \
	wget \
	xvfb \
 && apt clean -qq

# Setup locale
RUN sudo apt update && sudo apt install locales \
  && sudo locale-gen en_US en_US.UTF-8 \
  && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ARG ROSDIST=humble

# Set up repo to install Ignition
RUN /bin/sh -c 'wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null'

RUN apt-get update && apt-get upgrade -y

  # Install some 'standard' ROS packages and utilities.
RUN apt update \
  && apt install -y --no-install-recommends \
     python3-colcon-common-extensions \
     python3-vcstool \
     python3-sdformat13 \
     ros-${ROSDIST}-ackermann-msgs \
     ros-${ROSDIST}-actuator-msgs \
     ros-${ROSDIST}-ament-cmake-pycodestyle \
     ros-${ROSDIST}-image-transport \
     ros-${ROSDIST}-image-transport-plugins \
     ros-${ROSDIST}-joy-teleop \
     ros-${ROSDIST}-joy-linux \
     ros-${ROSDIST}-mavros-msgs \
     ros-${ROSDIST}-navigation2 \
     ros-${ROSDIST}-nav2-bringup \
     ros-${ROSDIST}-turtlebot3 \
     ros-${ROSDIST}-radar-msgs \
     ros-${ROSDIST}-ros-gzgarden \
     ros-${ROSDIST}-rqt-graph \
     ros-${ROSDIST}-rqt-image-view \
     ros-${ROSDIST}-rqt-plot \
     ros-${ROSDIST}-rqt-topic \
     ros-${ROSDIST}-rviz2 \
     ros-${ROSDIST}-vision-msgs \
     ros-${ROSDIST}-xacro \
  && sudo rm -rf /var/lib/apt/lists/* \
  && sudo apt clean -qq


  # NAV2
RUN apt update \
  && apt install -y --no-install-recommends \
        ros-${ROSDIST}-navigation2 \
        ros-${ROSDIST}-nav2-bringup \
 && sudo rm -rf /var/lib/apt/lists/* \
 && sudo apt clean -qq

# 1. Instalacja Gazebo Garden
RUN apt-get update && apt-get install -y \
    curl \
    lsb-release \
    gnupg \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg | gpg --dearmor > /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update \
    && apt-get install -y gz-garden \
    && rm -rf /var/lib/apt/lists/*

# 2. Konfiguracja użytkownika ROS
RUN groupadd -g ${GROUP_ID} rosuser && \
    useradd -u ${USER_ID} -g rosuser -m rosuser && \
    echo "rosuser:rosuser" | chpasswd && \
    mkdir -p /home/rosuser/ros_ws && \
    chown rosuser:rosuser /home/rosuser/ros_ws && \
    echo "rosuser ALL=(ALL) ALL" > /etc/sudoers.d/rosuser && \
    chmod 0440 /etc/sudoers.d/rosuser   

RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    xterm \
    python3-sdformat13 \
    ros-humble-joy-teleop \
    nano \
    vim \
    ros-humble-ros-gzgarden ros-humble-xacro

RUN mkdir -p /home/rosuser/vrx_ws/src && \
    cd /home/rosuser/vrx_ws/src && \
    git clone https://github.com/osrf/vrx.git
RUN cd /home/rosuser/vrx_ws && source /opt/ros/humble/setup.bash && colcon build --merge-install
RUN chown -R rosuser /home/rosuser

USER rosuser
WORKDIR /home/rosuser/ros_ws

# 4. Konfiguracja środowiska (opcjonalne)
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc