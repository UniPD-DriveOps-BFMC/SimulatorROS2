Code made available by Team Gradient, in the BFMC2024... The code was not tested by the organizing team!

The following mentions are done by the team:
•	Was not able to test traffic light, may or may not work.
•	Camera node path changed from /camera/image_raw to /automobile/image_raw
•	There are many warnings on launch, but the majority seem to have been there prior to the port; 
•	map_with_all_objects.launch is a bit broken. I detail the issue in the justfile and provide a hacky workaround, but the root cause should be dealt with.
•	The port is to Gazebo Classic, not Ignition. I don't know how much work would be required to Ignition, but it's likely a good amount. Though Gazebo Classic is much easier to install than ROS 1, so I still consider this a win.


<img src="https://github.com/ECC-BFMC/Simulator/blob/main/Picture1.png" width=30% height=30%>

# BFMC Simulator Project

The project contains the entire Gazebo simulator. 
- It can also be used to get a better overview of how the competition is environment looks like
- It can be used in to develop the vehicle state machine
- It can be used to simulate the path planning
- It can be used to set-up a configuration procedure for the real track
- Not suggested for image processing
- Try not to fall in the "continuous simulator developing" trap

Tips on how to install and work on it, can be found in the 


## The documentation is available in details here:
[Documentation](https://bosch-future-mobility-challenge-documentation.readthedocs-hosted.com/data/simulator.html)

## Running

Dependencies:
- `gazebo` 11 (classic)
- `ros2-humble`
- `ros2-humble-gazebo-{ros,msgs,dev,plugins}`
- `python3-pip`
- `direnv` (optional)
- `just`

### Installation

1. Install Python dependencies:
```bash
pip install -r requirements.txt
```

2. Build the ROS2 workspace:
```bash
colcon build
```

3. Source the environment:
```bash
source /opt/ros/humble/setup.bash
source ./install/local_setup.bash
```

`direnv` is useful for automating the environment setup with `.envrc` instead of sourcing manually.
The first time `.envrc` is sourced, the file `./install/local_setup.bash` doesn't exist so it will 
throw a warning. After running `colcon build`, this should be fine. The `.envrc` has only been 
tested on arch linux. Modify it according to your system, if necessary.

Then exit and enter the directory to source the new environment.

### Launch Commands

You can use the following `just` commands to run the simulator:

**Launch car only (simplest option):**
```bash
just car-only
```

**Launch with all objects (traffic lights, signs, pedestrians, etc.):**
```bash
just all-objects
```

**Run control example:**
```bash
just control_example
```

**Run camera example:**
```bash
just camera_example
```
