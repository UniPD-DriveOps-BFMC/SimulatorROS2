# BFMC Simulator: Autonomous Vehicle Development Platform

<div align="center">

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Gazebo Sim 8](https://img.shields.io/badge/Gazebo-Sim%208%20(Harmonic)-orange?logo=gazebo&logoColor=white)](https://gazebosim.org/docs/harmonic/)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-purple?logo=ubuntu&logoColor=white)](https://ubuntu.com/)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10+-green?logo=python&logoColor=white)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-BFMC-red)](LICENSE)

**Complete simulation environment for the Bosch Future Mobility Challenge**

[Features](#key-features) •
[Installation](#installation) •
[Usage](#usage) •
[Documentation](#additional-resources) •
[Contributing](#contributing)

</div>

---

<div align="center">
<img src="https://github.com/ECC-BFMC/Simulator/blob/main/Picture1.png" width="70%" alt="BFMC 2024 Track">

**BFMC 2024 Competition Track**
</div>

---

## Overview

This repository contains the complete BFMC (Bosch Future Mobility Challenge) simulator, successfully migrated to modern Gazebo Sim 8 (Harmonic) and ROS 2 Humble. The simulator provides a comprehensive environment for autonomous vehicle development, featuring realistic physics, sensor integration, and a detailed competition track.

### Key Features

**Vehicle Control**
- Ackermann steering geometry implementation
- Thread-safe command handling with real-time responsiveness
- Differential wheel speed control for accurate turning

**Sensor Suite**
- IMU (BNO055) - Orientation tracking (roll, pitch, yaw)
- GPS - Position estimation with realistic noise modeling
- Camera - Visual perception at `/automobile/image_raw`

**Environment**
- Detailed competition track with PBR materials
- Traffic lights and road signs
- Pedestrian objects
- Realistic physics simulation using ODE engine

### Current Status

The simulator is fully operational with the following components tested and working:

- **Core Functionality**: Vehicle physics, joint control, and Ackermann steering
- **Communication**: ROS 2 topics bridged via gz-ros
- **Sensors**: IMU, GPS, and camera streams
- **Control Interface**: Keyboard and programmatic control

Components requiring additional testing:
- Traffic light system
- All-objects launch timing (workaround available in justfile)

### Use Cases

This simulator is designed for:
- Competition track familiarization and strategy development
- Vehicle state machine development and testing
- Path planning algorithm validation
- Control system configuration and tuning
- Sensor fusion and localization testing

Note: For computer vision tasks, physical hardware testing is recommended due to simulation-to-reality gap.

---

## Installation

### Prerequisites

Ensure your system meets the following requirements:

- **Operating System**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill
- **Gazebo**: Sim 8 (Harmonic)
- **Python**: 3.10 or higher
- **Build Tools**: colcon, cmake

### Step 1: Install Gazebo Sim 8

```bash
sudo apt-get update
sudo apt-get install gz-harmonic
```

### Step 2: Install ROS 2 Humble

If not already installed:

```bash
sudo apt-get install ros-humble-desktop
```

### Step 3: Install Gazebo-ROS Bridge

```bash
sudo apt-get install ros-humble-ros-gzharmonic
```

### Step 4: Clone and Build

```bash
# Clone the repository
git clone https://github.com/GIU-F1Tenth/ECC-BFMC-Simulator.git
cd ECC-BFMC-Simulator

# Install Python dependencies
pip install -r requirements.txt

# Build the workspace (builds in parent directory)
cd ..
colcon build --symlink-install
cd ECC-BFMC-Simulator

# Source the environment
source /opt/ros/humble/setup.bash
source ../install/setup.bash
```

**Note**: The build system uses the parent directory for colcon workspace.

Alternatively, use the provided build script:

```bash
just build  # Automatically handles paths and sourcing
```

---

## Usage

### Quick Start

Launch the simulator with the vehicle:

```bash
just car
```

In a separate terminal, start the keyboard control:

```bash
just control_example
```

**Control Keys:**
- `W` - Accelerate forward
- `S` - Brake
- `A` - Steer left
- `D` - Steer right
- `ESC` - Exit

### Launch Options

#### Available Just Commands

| Command | Description |
|---------|-------------|
| `just build` | Build all packages in parent workspace |
| `just car` | Launch simulator with vehicle (GUI mode) |
| `just run` | Launch simulator + keyboard control (integrated) |
| `just headless` | Launch simulator without GUI (faster) |
| `just all-objects` | Launch with traffic lights, signs, pedestrians |
| `just control_example` | Start keyboard control interface |
| `just camera_example` | Start camera viewer |
| `just visualize` | Launch RViz2 for visualization |

**Integrated Launch** (simulator + control):
```bash
just run
```
Waits 12 seconds for simulator initialization before launching control.

**Headless Mode** (no GUI, for faster simulation):
```bash
just headless
```
Useful for automated testing or resource-constrained systems.

**Full Environment** (with traffic lights, signs, pedestrians):
```bash
just all-objects
```
*Note: Uses 5-second delay for proper object spawning. Includes SIGINT propagation for clean shutdown.*

**Camera Stream**:
```bash
just camera_example
```
Access camera feed at ROS topic: `/automobile/image_raw`

**Visualization**:
```bash
just visualize
```
Launches RViz2 for sensor data and robot state visualization.

---

## ROS 2 Interface

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/automobile/feedback` | `std_msgs/String` | Command acknowledgment |
| `/automobile/localisation` | `geometry_msgs/Pose` | Vehicle position with noise |
| `/automobile/IMU` | `geometry_msgs/Vector3` | Orientation (roll, pitch, yaw) |
| `/automobile/image_raw` | `sensor_msgs/Image` | Camera stream |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/automobile/command` | `std_msgs/String` | Vehicle control commands |

### Command Format

Commands are sent as JSON strings:

**Speed Control:**
```json
{"action": "1", "speed": 5.0}
```

**Steering Control:**
```json
{"action": "2", "steerAngle": 15.0}
```

**Brake:**
```json
{"action": "3", "steerAngle": 0.0}
```

---

## Architecture

### Gazebo Sim Migration

This simulator has been migrated from Gazebo Classic to Gazebo Sim 8, incorporating modern architectural patterns:

**Entity-Component-System (ECS)**
- Modular plugin architecture using `gz::sim::System` interface
- Efficient component-based state management

**Transport Layer**
- Internal messaging via `gz-transport`
- ROS 2 integration through `ros-gz-bridge`

**Rendering**
- PBR (Physically Based Rendering) materials for visual accuracy
- Support for Ogre1 and Ogre2 rendering engines

**Physics**
- ODE physics engine with configurable parameters
- Real-time joint control using velocity and position commands

### Plugin Implementation

All vehicle control plugins follow thread-safe patterns:

- Commands are buffered in callback functions
- Application occurs during `PreUpdate()` phase
- Joint control uses `JointVelocityCmd` and `JointPositionReset` components

Vehicle dynamics implement Ackermann steering geometry with differential wheel speeds for realistic car-like motion.

---

## Configuration

### Speed Adjustment

Default speed commands are scaled by 3x for responsive control. To modify this behavior:

Edit `src/plugins_pkgs/car_plugin/src/carlikerobot.cpp`:

```cpp
// Current: speed * 3.0
float adjustedSpeed = speed * 3.0;

// Adjust multiplier as needed
float adjustedSpeed = speed * YOUR_MULTIPLIER;
```

Rebuild after changes:
```bash
just build
```

### Resource Paths

The simulator automatically configures resource paths via launch files. If models fail to load, verify:

```bash
echo $GZ_SIM_RESOURCE_PATH
```

Should include the path to `models_pkg`.

### Changing the Track Map

The simulator includes multiple track configurations. To change the map:

**Available Maps:**
- `2024_Small.png` - Compact track (default)
- `2024_Medium.png` - Medium-sized track
- `2024_Big.png` - Full competition track
- `2021_Small.png`, `2021_Medium.png`, `2021_Big.png` - Previous year tracks

**To change the track texture:**

1. Edit the world file:
```bash
nano src/sim_pkg/worlds/world_with_separators.world
```

2. Locate the track material definition (around line 96):
```xml
<pbr>
  <metal>
    <albedo_map>model://track/materials/textures/2024_Small.png</albedo_map>
  </metal>
</pbr>
```

3. Change the texture filename:
```xml
<albedo_map>model://track/materials/textures/2024_Big.png</albedo_map>
```

4. Rebuild and relaunch:
```bash
just build
just car
```

**Track Dimensions:**
- Small: Optimized for quick testing
- Medium: Balanced for development
- Big: Full competition scale (14.68m x 14.99m)

---

## Troubleshooting

### Textures Not Displaying

**Symptom**: Track appears white or gray instead of showing road markings.

**Solution**: 
- Ensure GPU supports OpenGL 3.3 or higher
- Verify PBR material support in your rendering engine
- Check that `GZ_SIM_RESOURCE_PATH` includes model directories

### Vehicle Not Responding

**Symptom**: Car doesn't move when commands are sent.

**Solution**:
- Check plugin initialization in console: Look for "Configured successfully" messages
- Verify topic bridging: `ros2 topic list | grep automobile`
- Confirm joint discovery: Console should show "Successfully found all joints"

### Performance Issues

**Symptom**: Simulation runs slowly or stutters.

**Solutions**:
- Use headless mode: `just headless`
- Reduce real-time factor in world file
- Close unnecessary GUI panels in Gazebo

### Build Errors

**Symptom**: Compilation fails during `colcon build`.

**Solution**:
- Ensure all dependencies are installed
- Clean build: `colcon build --cmake-clean-cache`
- Check ROS 2 and Gazebo versions match requirements

---

## Additional Resources

**Official Documentation**: [BFMC Documentation](https://bosch-future-mobility-challenge-documentation.readthedocs-hosted.com/data/simulator.html)

**Repository**: [GitHub - ECC-BFMC-Simulator](https://github.com/GIU-F1Tenth/ECC-BFMC-Simulator)

**ROS 2 Humble**: [ROS 2 Documentation](https://docs.ros.org/en/humble/)

**Gazebo Sim**: [Gazebo Documentation](https://gazebosim.org/docs/harmonic/getstarted/)

---

## Contributing

Contributions are welcome! Please ensure:
- Code follows existing architectural patterns
- Thread-safe plugin implementations
- Documentation updates for new features
- Testing on Ubuntu 22.04 with specified dependencies

---

## License

This project is part of the Bosch Future Mobility Challenge. Please refer to the original license terms.

---

## Acknowledgments

### Migration to Gazebo Sim 8 & ROS 2 Humble

**Migration Engineer**: Mohammed Azab  
**Institution**: German International University (GIU)  
**Team**: GIU F1Tenth Racing Team  
**Date**: November 2025

#### Migration Achievements

- Complete port from Gazebo Classic to Gazebo Sim 8 (Harmonic)
- Updated ROS 1 architecture to ROS 2 Humble
- Implemented thread-safe plugin architecture with proper ECS patterns
- Fixed segmentation faults through command buffering and PreUpdate phases
- Converted materials from Ogre scripts to PBR materials
- Implemented Ackermann steering with differential wheel control
- Achieved 3x speed multiplier for responsive vehicle control
- Resolved SDF deprecation warnings (removed frame attributes)
- Established gz-transport to ROS 2 bridge integration

#### Original Project

- **Source**: Bosch Future Mobility Challenge (BFMC)
- **Original Team**: Team Gradient (BFMC 2024)
- **Documentation**: [BFMC Official Docs](https://bosch-future-mobility-challenge-documentation.readthedocs-hosted.com/)

#### Special Thanks

- BFMC organizing team for the original simulator framework
- GIU F1Tenth team for testing and validation
- Open Robotics for Gazebo Sim and ROS 2 development

---

**Status**: Fully operational with modern Gazebo architecture ✓
