# ROS2 + Gazebo Docker Setup 🤖

**For my beloved colleagues who enjoy both robotics and shitposting** 🚀

## Client Setup 🖥️

### What's in the box?
- `Dockerfile` - Our container blueprints
- `run_ros_container.sh` - The magic launch script

### Launch Procedure (For Non-Windows Enjoyers)
```bash
# 1. Build the thing
# 2. Make it go brrrr
./run_ros_container.sh
```

### Important Notes ⚠️
- **GUI Requirement**: Must use **Docker Engine** (Desktop users get 🖕 instead of GUI)
- **Windows Survivors** (we salute you 🫡):
  ```bash
  # Ask your friendly neighborhood LLM:
  # "xorg x11 server windows install how to fast no yapping"
  ```

---

## Toolbox Section 🧰
Choose your weapon (both require VPN connection):

### Gazebo GUI 🔥
```bash
export GZ_RELAY=<server_ip_in_VPN>
export GZ_TOPIC=<server_topic>  #NEED TO BE SAME ON SERVER AND ON CLIENT, for sevrer default is `powertrain`
export GZ_IP=<your_vpn_ip>           

# Start simulation
gz sim -g  # '-g' stands for 'give me graphics or give me death'
```

### RVIZ2 🎯
```bash
rviz2  # Just works™ (if VPN connected and the robot gods are pleased)
```

**Pro Death Wish Tip™:** Skip VPN to become expert at staring contests with blank screens 💀

---

## Server Setup 🖥️🔥

### Contents
- `Dockerfile` - Our server temple
- `build_image.sh` - Construction crew
- `start_server.sh` - The big red button, starts with `GZ_TOPIC=powertrain`

### Server Startup Ritual
```bash
# 1. Build holy container
./build_image.sh

# 2. Summon the daemon
./start_server.sh <client_ip_in_VPN>  # Replace with target's VPN IP
```

---

## Final Words of Wisdom 🧠
This setup cost me:
- 3 mental breakdowns 
- ∞ LLM queries *(perplexity and deepseek my beloved 💙)*
- 1 will to live

**May the containers be ever in your favor!** 🐳✨