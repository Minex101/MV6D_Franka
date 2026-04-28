# Multi-View Pose Estimation with Franka Panda for Picking and Packing in Warehouses
### BEng Mechatronics and Robotics (Industrial) | University of Leeds
**Author:** Affan Ahmed Khan Mohammed  

---

## 📌 Project Overview
This project implements a **Multi-View 6D Pose Estimation system** designed for warehouse pick-and-pack operations. By utilizing a **Franka Panda** robot arm in **NVIDIA Isaac Sim**, the system captures a target object from multiple strategic viewpoints to refine pose accuracy and reduce uncertainty using a stochastic fusion approach.

---

## 🏗 System Architecture
The system is divided into three functional nodes to ensure modularity and real-time performance:

1. **Vision Node (Deep Perception):** Detects the target object and estimates its 6D pose from the end-effector camera stream using a TensorRT-accelerated DOPE model.
2. **Movement Node (Trajectory Coordinator):** Executes an automated "Multi-View" sequence, followed by object search, exposing front, top and lateral viewpoints.
3. **Fusion Node (Pose Fusion):** Fuses multiple 6D pose estimates to compute a refined object pose using Weighted Translation and Weighted Markley's Quaternion Averaging.

---

## 🔗 Dependencies & Quick Links

| Component | Purpose | Official Resource |
| :--- | :--- | :--- |
| **NVIDIA Isaac Sim** | Physics & Rendering | [Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html) |
| **NVIDIA DOPE** | 6D Pose Estimation | [GitHub Repository](https://github.com/NVlabs/Deep_Object_Pose_Estimation) |
| **ROS 2 Humble** | Robotics Middleware | [Installation Guide](https://docs.ros.org/en/humble/Installation.html) |
| **Trac-IK** | IK Solver | [Trac-IK Py Bindings](https://github.com/m-andreas/trac_ik_python) |

---

## 🛠 Setup & Installation

### 1. Clone the Repository
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Minex101/MV6D_Franka.git
```

2. Install Python Dependencies
```Bash

pip install numpy pykdl tracikpy
```
3. Build the Package
```Bash

cd ~/ros2_ws
colcon build --packages-select multi_view_pkg
source install/setup.bash
```
🎮 Usage
Launching the Full System

To start the perception, movement, and fusion nodes simultaneously:
```Bash

ros2 launch multi_view_pkg pipeline.launch.py
```
