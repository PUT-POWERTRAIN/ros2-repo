FROM ros:humble-ros-base-jammy

SHELL ["bin/bash", "-c"]

ARG USER_ID=1000
ARG GROUP_ID=1000

# Set environment variables
ENV GZ_IP=10.8.0.1
ENV GZ_PARTITION=powertrain-server

# Update the package list and install necessary packages
RUN apt-get update && \
    apt-get install -y lsb-release curl gnupg && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && \
    apt-get install -y gz-garden && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    apt install -y software-properties-common && \
    add-apt-repository universe

ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y ros-humble-ros-base ros-humble-ros-gzgarden ros-humble-xacro ros-humble-joy-teleop python3-sdformat13 && \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    rm -rf /var/lib/apt/lists/* 

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

RUN mkdir -p /home/rosuser/vrx_ws/src && \
    cd /home/rosuser/vrx_ws/src && \
    git clone https://github.com/osrf/vrx.git

# Build the vrx workspace
RUN cd /home/rosuser/vrx_ws && \
    source /opt/ros/humble/setup.bash && \
    colcon build --merge-install

RUN groupadd -g ${GROUP_ID} rosuser && \
    useradd -u ${USER_ID} -g rosuser -m rosuser && \
    echo "rosuser:rosuser" | chpasswd && \
    mkdir -p /home/rosuser/ros_ws && \
    chown rosuser:rosuser /home/rosuser/ros_ws && \
    echo "rosuser ALL=(ALL) ALL" > /etc/sudoers.d/rosuser && \
    chmod 0440 /etc/sudoers.d/rosuser

RUN chown -R rosuser /home/rosuser
USER rosuser

# Set the default command to run Gazebo simulation with specified options
CMD ["gz", "sim", "-s", "-v", "4"]
