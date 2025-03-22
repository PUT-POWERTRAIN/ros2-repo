source /opt/ros/humble/setup.bash
source ~/vrx_ws/install/setup.bash

#stop any running gazebo instances
pkill -9 ruby

TERMINALS=(
    "konsole -e"
    "gnome-terminal -- bash -c"
    "xfce4-terminal -e"
    "mate-terminal -e"
    "x-terminal-emulator -e"
    "xterm -e"
)

if ! dpkg -s ros-humble-joy-teleop &>/dev/null; then
  sudo apt install ros-humble-joy-teleop
fi

COMMANDS=(
    "ros2 launch vrx_gz competition.launch.py world:=sydney_regatta urdf:=./params/wamv_target.urdf"
    "rviz2"
    "python3 ./robot_localization.launch.py"
    "ros2 launch vrx_gz usv_joy_teleop.py"
)

# Try to find an available terminal emulator
for term in "${TERMINALS[@]}"; do
    TERM_CMD=$(echo "$term" | awk '{print $1}')
    if command -v "$TERM_CMD" &> /dev/null; then
        break
    fi
done

echo "$term"

for command in "${COMMANDS[@]}"; do
    $term $command &
done

