# Multi-View Pose Estimation with Franka Panda for Picking and Packing in Warehouses

![python](https://img.shields.io/badge/python-3.10-blue)
![platform](https://img.shields.io/badge/platform-Ubuntu--22.04-green)
![IsaacSim](https://img.shields.io/badge/IsaacSim-5.1.0-red)
![ROS2](https://img.shields.io/badge/ROS2-Humble-orange)

## 📌 Project Overview
This project implements a **Multi-View 6D Pose Estimation system** designed for warehouse pick-and-pack operations. By utilizing a **Franka Panda** robot arm in **NVIDIA Isaac Sim**, the system captures a target object from multiple strategic viewpoints to refine pose accuracy and reduce uncertainty using a stochastic fusion approach.

## 🎥 Demo

https://github.com/user-attachments/assets/cc959a7c-9ba0-4f62-a6a7-cfd177b745d9

*Figure 1: After a sweep search finds the target, the Franka Panda moves to object-centric viewpoints around it, pausing at each one to capture a pose observation.*

https://github.com/user-attachments/assets/d240e825-621f-406e-9de9-fe9e78ec48bd

*Figure 2: The same run from the RealSense D435 mounted on the gripper, showing what the camera sees from each viewpoint.*

## 🏗 System Architecture

The system is split into three ROS 2 nodes that communicate over topics, which keeps perception, motion and fusion modular and easy to test.

1. **Vision Node (Deep Perception):** Runs DOPE on the end-effector camera stream to estimate the target's 6D pose, then transforms it into the world frame using TF. It also scores each observation with a **depth-residual uncertainty metric**, which compares DOPE's predicted depth with the RealSense depth measurement. Poses are captured on demand, only when the arm is stationary, which cut translation RMSE by 32.5%, 26.6% and 47.4% along X, Y and Z compared with continuous sampling.
   
2. **Movement Node (Trajectory Coordinator):** Runs a sweep search to find the object, then uses custom **object-centric viewpoint planning algorithm** to compute camera viewpoints. It computes front, top, left and right end-effector poses relative to the detected object, solves them with TRAC-IK, and captures an observation at each reachable viewpoint. Unlike fixed joint-space viewpoints, it adapts to wherever the object is.
   
3. **Fusion Node (Pose Fusion):** Combines the observations into one refined pose using **uncertainty-weighted fusion**. Each observation gets a weight inversely proportional to its uncertainty score, so unreliable views count for less. Translations are fused by weighted averaging, and rotations by weighted **Markley quaternion averaging**, which avoids the sign ambiguity of simple quaternion averaging.

## 🎖️Results

Evaluated in simulation on objects from the HOPE dataset on across 50 samples with varying clutter. Our solution shows significant improvements as displayed below:

| Metric | Single-view | Fused multi-view | Improvement |
| --- | --- | --- | --- |
| Mean translation error | 7.55 cm | 0.96 cm | 87.3% lower |
| Median translation error | 1.45 cm | 0.77 cm | 46.9% lower |
| Mean rotation error | 17.90° | 6.13° | 65.8% lower |
| Median rotation error | 5.81° | 3.20° | 44.9% lower |

## 🔗 Dependencies & Quick Links

Install these in order, following each official guide.

| Step | Component | Purpose | Official Resource |
| :--- | :--- | :--- | :--- |
| 1 | **ROS 2 Humble** | Robotics middleware | [Installation Guide](https://docs.ros.org/en/humble/Installation.html) |
| 2 | **NVIDIA Isaac Sim** | Simulation, physics and rendering | [Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html) |
| 3 | **NVIDIA DOPE** | 6D pose estimation | [Isaac ROS DOPE](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_dope/index.html) |
| 4 | **TRAC-IK** | IK solver for trajectory planning | [Install guide](https://moveit.picknik.ai/main/doc/how_to_guides/trac_ik/trac_ik_tutorial.html#install) |

Isaac Sim needs an NVIDIA RTX GPU. After installing it, enable the **ROS 2 bridge** extension so the simulator can talk to ROS 2.

## 🛠 Setup & Installation

### 1. Clone the repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Minex101/MV6D_Franka.git
```

### 2. Build the package

```bash
cd ~/ros2_ws
colcon build --packages-select multi_view_pkg
source install/setup.bash
```

### 3. Run

1. Open Isaac Sim and load the scene 1.usd provided in isaac_sim_assets
2. In a terminal where you've sourced the workspace, launch the full system:

```bash
ros2 launch multi_view_pkg pipeline.launch.py
```
