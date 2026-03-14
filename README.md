Multi-View 6D Pose Estimation for Autonomous Warehouse Logistics
BEng Mechatronics and Robotics (Industrial) | University of Leeds

Author: Affan Ahmed Khan Mohammed

Student ID: 201621240

Tools: NVIDIA Isaac Sim, ROS 2, NVIDIA DOPE
📌 Project Overview

This project develops a Multi-View 6D Pose Estimation system optimized for robotic pick-and-pack operations in warehouse environments.

Standard single-view perception often struggles with occlusions and sensor noise. This system leverages a Franka Panda robot arm within NVIDIA Isaac Sim to capture an object (e.g., a Ketchup bottle) from multiple strategic viewpoints. The data is then processed through a fusion pipeline to refine pose accuracy and minimize spatial uncertainty.
🚀 Key Features

    Deep Perception: Real-time 6D pose estimation using NVIDIA DOPE (Deep Object Pose Estimation).

    Simulation-First Workflow: High-fidelity physics and photorealistic rendering in NVIDIA Isaac Sim 5.1.0.

    Modular ROS 2 Architecture: Containerized nodes (Humble/Jazzy) for vision, motion control, and data fusion.

    Multi-View Strategy: Automated trajectory coordination to observe objects from four optimal orbits.

🏗 System Architecture

The system utilizes a decentralized node architecture to ensure low-latency communication and modular testing.
1. Vision Node (Deep Perception)

Performs deep learning inference to extract 6D poses from the end-effector camera stream.

    Inference Engine: TensorRT/ONNX.

    Input: RGB-D data from Isaac Sim.

    Output: geometry_msgs/PoseStamped.

2. Movement Node (Trajectory Coordinator)

The "Choreographer" of the robot arm. It executes a multi-view sequence to maximize information gain.

    Control Type: Joint-space control via /joint_command.

    Logic: Asynchronous coordination between motion and "Snap" commands.

3. Fusion Node (Kalman Filter)

Status: In Development
The system's "Brain," responsible for spatial filtering.

    Algorithm: Extended Kalman Filter (EKF).

    Function: Merges noisy pose estimates from multiple viewpoints into a singular, high-confidence grasping coordinate.

🛠 Setup & Installation
Prerequisites

    NVIDIA Isaac Sim 5.1.0

    ROS 2 (Humble or Jazzy) (Docker/Containerized recommended)

    NVIDIA GPU (With TensorRT support)

Installation

    Clone the Repository:
    Bash

    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/your-username/multi_view_pkg.git

    Install Dependencies:
    Bash

    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y

    Build the Package:
    Bash

    colcon build --packages-select multi_view_pkg
    source install/setup.bash

🎮 Usage
Launching the Full Pipeline

To start the vision, movement, and fusion nodes simultaneously:
Bash

ros2 launch multi_view_pkg start_system.launch.py

Calibrating Viewpoints

To manually find optimal joint angles for specific warehouse layouts, use the Joint State GUI:
Bash

ros2 run joint_state_publisher_gui joint_state_publisher_gui [PATH_TO_URDF]/franka.urdf --ros-args -r /joint_states:=/joint_command

📚 References
Code snippet

@inproceedings{tremblay2018dope,
  author    = {Tremblay, Jonathan and To, Thang and Sundaralingam, Balakumar and Xiang, Yu and Fox, Dieter and Birchfield, Stan},
  title     = {Deep Object Pose Estimation for Semantic Robotic Grasping of Household Objects},
  booktitle = {Conference on Robot Learning (CoRL)},
  year      = {2018}
}
