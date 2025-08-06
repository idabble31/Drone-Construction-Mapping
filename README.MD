# <Project Title> — Drone Simulation & Mapping (WIP)

A ROS/Gazebo workspace for simulating a multi-sensor drone (LiDAR + fisheye cameras) and basic teleop along with mapping.
> **Status:** Work in progress, active freelance project. Interfaces, topics, and file paths may change.

---

## Prerequisites

- **OS:** Ubuntu 20.04 (recommended)
- **ROS:** Noetic
- **Gazebo:** 11.x
- **Build tools:** CMake, catkin (`catkin_make`)
- **ROS packages (apt):**
  - `ros-noetic-xacro`
  - `ros-noetic-robot-state-publisher`
  - `ros-noetic-joint-state-publisher`
  - `ros-noetic-teleop-twist-keyboard`
  - `ros-noetic-hector-gazebo-plugins`
- **Built from source (in this workspace):**
  - `hector_quadrotor_gazebo_plugins` (provides the simple quadrotor controller)

---

## Quick Start

```bash
# 1) Clone into a catkin workspace
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone <this-repo-url>
cd ..

# 2) Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 3) Build & source
catkin_make
source devel/setup.bash

# 4) Launch Gazebo
roslaunch drone_description drone_sim_hector.launch

# 5) Teleop (in a new terminal)
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
