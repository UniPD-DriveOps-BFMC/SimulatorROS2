# 0.1. Install ROS2 Jazzy (assumed already installed)

# 0.2. Install Gazebo Harmonic
sudo apt install gz-harmonic

# 0.3. Install ROS2-Gazebo bridge for Jazzy
sudo apt install ros-jazzy-ros-gz

# 0.4. Install just
curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to ~/.local/bin
export PATH=$PATH:~/.local/bin

1 - git clone https://github.com/GIU-F1Tenth/ECC-BFMC-Simulator.git ~/BFMC/SimulatorROS2

2 - sed -i 's/this->hasPendingCommand = false;/\/\/ this->hasPendingCommand = false;  \/\/ Keep applying last command/' ~/BFMC/SimulatorROS2/src/plugins_pkgs/car_plugin/src/carlikerobot_ros_plugin.cpp

3 - for f in $(find ~/BFMC/SimulatorROS2/src -name "*.launch"); do
    xmlline=$(grep -n "<?xml" "$f" | head -1 | cut -d: -f1)
    if [ ! -z "$xmlline" ] && [ "$xmlline" != "1" ]; then
        xmldecl=$(sed -n "${xmlline}p" "$f")
        sed -i "${xmlline}d" "$f"
        sed -i "1i\\${xmldecl}" "$f"
    fi
done

4 - sed -i 's|<env name="GZ_SIM_RESOURCE_PATH" value="\$(find-pkg-share models_pkg)" />|<env name="GZ_SIM_RESOURCE_PATH" value="$(find-pkg-share models_pkg)" />\n    <env name="GZ_GUI_CONFIG_PATH" value="/home/hisham/.gz/sim/8/gui.config" />|' ~/BFMC/SimulatorROS2/src/sim_pkg/launch/gazebo.launch

5 - mkdir -p ~/.gz/sim/8

6 - download gui.config and place it there

7 - sudo chown -R $USER:$USER ~/.gz
rm -rf ~/.gz/rendering/ogre-rtshader/*
mkdir -p ~/.gz/rendering/ogre-rtshader/hisham-rtshaderlibcache

8 - cd SimulatorROS2
source /opt/ros/jazzy/setup.bash
source ../install/setup.bash
just car

9 - from another terminal : 
cd ~/BFMC/SimulatorROS2
source /opt/ros/jazzy/setup.bash
source ../install/setup.bash

10 - ros2 topic pub /automobile/command std_msgs/msg/String "data: '{\"action\": \"2\", \"steerAngle\": 0.0}'" --once
ros2 topic pub /automobile/command std_msgs/msg/String "data: '{\"action\": \"1\", \"speed\":0.05}'" --once
