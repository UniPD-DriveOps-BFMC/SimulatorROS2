# 0.1. Install ROS2 Jazzy (assumed already installed)

# 0.2. Install Gazebo Harmonic
sudo apt install gz-harmonic

# 0.3. Install ROS2-Gazebo bridge for Jazzy
sudo apt install ros-jazzy-ros-gz

# 0.4. Install just
curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to ~/.local/bin
export PATH=$PATH:~/.local/bin

1 - git clone https://github.com/UniPD-DriveOps-BFMC/SimulatorROS2.git

2 - mkdir -p ~/.gz/sim/8

3 - download gui.config and place it there

4 - sudo chown -R $USER:$USER ~/.gz
rm -rf ~/.gz/rendering/ogre-rtshader/*
mkdir -p ~/.gz/rendering/ogre-rtshader/hisham-rtshaderlibcache

5 - cd SimulatorROS2
source /opt/ros/jazzy/setup.bash
source ../install/setup.bash
just car

6 - from another terminal : (I'm assuming the directory above SimulatorROS2 is called BFMC, change it if it's different for you)
cd ~/BFMC/SimulatorROS2
source /opt/ros/jazzy/setup.bash
source ../install/setup.bash

7 - ros2 topic pub /automobile/command std_msgs/msg/String "data: '{\"action\": \"2\", \"steerAngle\": 0.0}'" --once
ros2 topic pub /automobile/command std_msgs/msg/String "data: '{\"action\": \"1\", \"speed\":0.05}'" --once
