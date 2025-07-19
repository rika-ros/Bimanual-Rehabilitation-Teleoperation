# Bimanual Rehabilitation Teleoperation (ROS1-Novint Falcon)

This repository contains a ROS1 based implementation of a bimanual assist-as-needed rehabilitation system using Novint Falcon haptic devices. The setup is used for simulating and controlling upper-limb rehab tasks with haptic feedback.


## Features

- Bimanual control using dual Novint Falcons
- Assist-as-needed (AAN) shared control scheme based on game theory
- Realistic physics simulation with Gazebo
- ROS1 (Melodic) compatible workspace
- Adaptive human input estimation (sensorless)
- Trajectory tracking, error minimization, and gravity compensation

## Project Structure
```
bimanual_ws/
├── src/
│ ├── haptic_pkg/ # Main package
│ │ ├── launch/ # Launch files
│ │ ├── urdf/ # Object models
│ │ ├── scripts/ # Python/C++ control scripts
│ │ └── config/ # YAML configuration files
├── ros_falcon/ # Novint Falcon package
├── build/
├── devel/
└── README.md
```

## Requirements

- Ubuntu 18.04 + ROS1 Melodic
- Novint Falcon drivers (`libnifalcon`)
- Gazebo (v9 or compatible)
- ROS packages:
  - `roscpp`, `rospy`, `gazebo_ros`, `geometry_msgs`, `sensor_msgs`
  - `controller_manager`, `joint_state_publisher`, etc.

## Setup Instructions

1. Clone the repo

```
git clone https://github.com/rh-ch/Bimanual-Rehabilitation-Teleoperation.git
```

2. Build and source the ws

```
cd ~/bimanual_ws
catkin_make
source devel/setup.bash
```

## Running the Simulation

1. Launch Gazebo and haptic setup

```
roslaunch haptic_pkg box_with_falcon_mainLaunch.launch
```

2. Run the control script in another terminal

```
source devel/setup.bash
cd ~/bimanual_ws/src/haptic_pkg/scripts/
python <script>.py
```
## Contact

For queries:

📧 rchps.05@gmail.com/ashikachandavarkar@gmail.com

📂 GitHub: [rika-ros](https://github.com/rika-ros)
 
