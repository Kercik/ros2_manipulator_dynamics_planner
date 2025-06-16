# ROS2 Manipulator Dynamics Planner with Docker

This ROS2 package provides a comprehensive solution for simulating and controlling a UR5 manipulator in Gazebo with depth cameras for perception. The system integrates MoveIt for motion planning and includes Docker support for easy setup.

## Demo
![GIF of presentation](demo.gif)

## Features

* Gazebo simulation with UR5 manipulator
* Depth camera environment with two RealSense D435 cameras
* MoveIt integration for motion planning
* Interactive planning interface with multiple planner options
* Octomap-based collision avoidance using depth camera data
* Docker container for easy setup and reproducibility

## Prerequisites for docker container

* Linux system with NVIDIA GPU
* Docker installed
* NVIDIA Container Toolkit installed

## Installation

### 1. Clone the Repository
```bash
git clone https://github.com/Kercik/ros2_manipulator_dynamics_planner.git ./src
```

### 2. Clone Required Package
```bash
git clone https://github.com/iKrishneel/octomap_server2.git ./src
```

### 3. Build the Workspace
```bash
colcon build --symlink-install
source install/setup.bash
```

## Running with Docker

### 1. Build the Docker Image
```bash
docker build -t miapr:humble -f Dockerfile .
```

### 2. Start the Container
```bash
./start.sh
```
### Inside the Container

1. Build the workspace:
```bash
cd /root/shared_workspace/src
git clone https://github.com/Kercik/ros2_manipulator_dynamics_planner.git
git clone https://github.com/iKrishneel/octomap_server2
cd ..
colcon build --symlink-install
source install/setup.bash
```
## Usage
### Launch the Simulation

#### Run these commands in separate terminals inside the container:

1. Launch the UR5 simulation with MoveIt:

```bash
ros2 launch ros2_manipulator_dynamics_planner ur_sim_moveit.launch.py
```
2. Launch the depth camera environment:

```bash
ros2 launch ros2_manipulator_dynamics_planner depth_camera_environment.launch.py
```
3. Launch the static transforms for cameras:

```bash
ros2 launch ros2_manipulator_dynamics_planner fix.launch.py
```
#### Run the Planner

##### After launching the simulation, run:
```bash
ros2 run ros2_manipulator_dynamics_planner planner
```
### Interactive Planning Interface

#### The planner provides an interactive interface for:

* Selecting motion planners (RRTConnect, PRM, RRT, TRRT)

* Choosing motion types (linear or point-to-point)

* Specifying target positions and orientations

* Executing planned trajectories

### System Architecture

#### The system consists of:

1. Gazebo Simulation: Physics-based environment with UR5 manipulator

2. Depth Cameras: Two RealSense D435 cameras for environment perception

3. MoveIt: Motion planning framework with OMPL integration

4. Planning Node: Interactive interface for trajectory planning

5. Octomap: Collision avoidance system using point cloud data


## Key Files
### Docker Configuration

* Dockerfile: Defines the container environment with all ROS2 dependencies

* start.sh: Script to launch the Docker container with proper permissions

### Launch Files

* ur_sim_moveit.launch.py: Main simulation launch file

* depth_camera_environment.launch.py: Sets up depth cameras

* fix.launch.py: Configures camera transforms

### Core Components

* sensors_3d.yaml: Configures depth camera sensors for Octomap

* camera.urdf.xacro: URDF description of RealSense D435 cameras

* planning.cpp: Interactive planning node

## Troubleshooting

#### X Server issues:
```bash
xhost +local:root
```
#### Camera topics not visible:

* Verify cameras are spawned in Gazebo

* Check TF tree with ros2 run tf2_tools view_frames

#### MoveIt planning fails:

* Ensure Octomap is receiving point cloud data

* Check collision objects in RViz

#### Future Improvements

* Add dynamic obstacle avoidance

* Implement force-torque control

* Integrate AI-based motion planning

* Add ROS2 services for programmatic control

* Implement trajectory optimization

#### License

BSD 3-Clause License - see LICENSE for details.
