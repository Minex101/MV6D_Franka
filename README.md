University of Leeds

Project: Multi-View Pose Estimation with Franka Panda for picking and packing in Warehouses

Author: Affan Ahmed Khan Mohammed

Student ID: 201621240

Course: BEng Mechatronics and Robotics (Industrial)

---

📌 **Project Overview**

This project implements a Multi-View 6D Pose Estimation system designed for warehouse pick-and-pack operations. By utilizing a Franka Panda robot arm in NVIDIA Isaac Sim, the system captures the target object (Ketchup bottle) from multiple strategic viewpoints to refine pose accuracy and reduce uncertainty using a Kalman Filter-based fusion approach.

Key Components:

    Simulation: NVIDIA Isaac Sim 5.1.0

    Vision Core: NVIDIA Deep Object Pose Estimation (DOPE)

    Middleware: ROS 2 Humble / Jazzy (Containerized)

---

🏗 **System Architecture**

The system is divided into three functional nodes to ensure modularity and real-time performance:

1. Vision Node (Deep Perception)

Uses the NVIDIA DOPE model to detect the object and estimate its 6D pose from the robot's end-effector camera.

    Input: RGB-D camera stream from Isaac Sim.

    Process: Deep Learning inference using an ONNX/TensorRT engine.

    Output: PoseStamped messages for the detected object.

2. Movement Node (Trajectory Coordinator)

Manages the robot's "Multi-View" sequence. It moves the Franka Panda through four pre-defined viewpoints to observe the object from different angles, overcoming self-occlusion and sensor noise.

    Control: Publishes JointState commands to /joint_command.

    Strategy: Strategic orbits around the workspace center.

3. Fusion Node (Kalman Filter) [NOT DEVELOPED YET]

The "Brain" of the system. It subscribes to the noisy pose estimates from the Vision Node and fuses them over time.

    Process: Applies an Extended Kalman Filter (EKF) to merge multiple viewpoints.

    Result: A stabilized, high-confidence 6D pose used for final grasping.

---

🛠 **Calibration Tools**

1. Setup Franka Viewpoints

To find the optimal joint angles for your warehouse scene, use the joint_state_publisher_gui. Run this on your host machine (Humble):

ros2 run joint_state_publisher_gui joint_state_publisher_gui ~/Documents/isaacsim/franka.urdf --ros-args -r /joint_states:=/joint_command

---

📚 **References**

@inproceedings{tremblay2018corl:dope,
 author = {Jonathan Tremblay and Thang To and Balakumar Sundaralingam and Yu Xiang and Dieter Fox and Stan Birchfield},
 title = {Deep Object Pose Estimation for Semantic Robotic Grasping of Household Objects},
 booktitle = {Conference on Robot Learning (CoRL)},
 year = 2018
}
---

@inproceedings{tremblay2018dope,
  author    = {Tremblay, Jonathan and To, Thang and Sundaralingam, Balakumar and Xiang, Yu and Fox, Dieter and Birchfield, Stan},
  title     = {Deep Object Pose Estimation for Semantic Robotic Grasping of Household Objects},
  booktitle = {Conference on Robot Learning (CoRL)},
  year      = {2018}
}

---
