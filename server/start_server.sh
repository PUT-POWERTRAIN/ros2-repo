#!/bin/bash

# Check if an IP address is provided as an argument
if [ -z "$1" ]; then
  echo "Usage: $0 <client_ip>"
  exit 1
fi

# Assign the provided IP address to a variable
CLIENT_IP=$1
PARTITION=powertrain

# Start the Docker container with the specified settings
docker run --rm --net=host -e GZ_RELAY=$CLIENT_IP -e GZ_PARTITION=$PARTITION --volume="$(pwd)/..:/home/rosuser/ros_ws" powertrain-gz bash -c "
  cd /home/rosuser/ros_ws &&
  source ~/vrx_ws/install/setup.bash &&
  ros2 launch vrx_gz competition.launch.py world:=sydney_regatta urdf:=./params/wamv_target.urdf extra_gz_args:="-s" &
  # python3 ./robot_localization.launch.py &
  # ros2 launch slam_toolbox online_async_launch.py slam_params_file:=params/mapper_params_online_async.yaml
  wait
"