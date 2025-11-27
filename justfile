# BFMC Simulator - Build and Launch Commands
# 
# This justfile provides convenient commands for building and running
# the Bosch Future Mobility Challenge (BFMC) simulator with Gazebo Sim 8
# and ROS 2 Humble.
#
# Requirements:
#   - ROS 2 Humble
#   - Gazebo Sim 8 (Harmonic)
#   - All dependencies from requirements.txt installed
#
# Usage:
#   just build          - Build all packages
#   just car            - Launch simulator with GUI and car
#   just run            - Launch simulator and control interface together
#   just control_example - Launch keyboard control interface
#   just headless       - Launch simulator without GUI (server only)

# Build all ROS 2 packages with symlink-install
build:
    cd .. && colcon build --symlink-install

# Launch simulator without GUI (headless/server-only mode)
# Useful for CI/CD, automated testing, or resource-constrained environments
headless:
    bash -c "source ../install/setup.bash && ros2 launch sim_pkg map_with_car.launch gui:=false"

# Launch simulator with GUI (default mode)
# Includes the complete track, car model, and all environment objects
# Note: Uses Ogre1 rendering engine (configured in ~/.gz/sim/8/gui.config)
# This avoids Ogre2 texture loading issues in some Gazebo versions
car:
    bash -c "source ../install/setup.bash && ros2 launch sim_pkg map_with_car.launch"

# Launch simulator with all dynamic objects (traffic lights, pedestrians, etc.)
# Uses a delayed launch to ensure Gazebo is fully initialized before spawning objects
all-objects:
    #! /bin/sh
    # Delayed object spawning to avoid queue overflow issues
    # The trap ensures proper cleanup when interrupted
    trap "trap - TERM && kill -- -$$" INT TERM EXIT
    bash -c "source ../install/setup.bash && ros2 launch sim_pkg map_with_car.launch gui:=true" &
    sleep 5
    bash -c "source ../install/setup.bash && ros2 launch sim_pkg all_objects.launch" &
    wait

# Launch the keyboard-based control interface
# Controls: w/a/s/d for movement, ESC to stop and exit
control_example:
    bash -c "source ../install/setup.bash && ros2 run example control"

# Launch the camera visualization example
camera_example:
    bash -c "source ../install/setup.bash && ros2 run example camera"

# Launch RViz2 for visualization of ROS 2 topics
visualize:
    bash -c "source ../install/setup.bash && rviz2"

# Launch simulator and control interface together (recommended for quick start)
# Starts the simulator in background, waits for initialization,
# then launches the control interface in foreground
run:
    #! /bin/sh
    trap "trap - TERM && kill -- -$$" INT TERM EXIT
    echo "Starting simulator..."
    bash -c "source ../install/setup.bash && ros2 launch sim_pkg map_with_car.launch" &
    sleep 12
    echo "Starting control interface..."
    echo "Use W/A/S/D to drive, ESC to exit"
    bash -c "source ../install/setup.bash && ros2 run example control"
