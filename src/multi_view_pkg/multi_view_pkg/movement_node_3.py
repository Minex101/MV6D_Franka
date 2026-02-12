"""
MovementCoordinator: Orchestrates robot search patterns and IK execution.
Uses the TRAC-IK inverse kinematics solver for robust pose reachability.

Reference:
P. Beeson and B. Ames, "TRAC-IK: An open-source library for improved 
solving of generic inverse kinematics," 2015 IEEE-RAS 15th International 
Conference on Humanoid Robots (Humanoids), Seoul, 2015, pp. 928-935.
doi: 10.1109/HUMANOIDS.2015.7363472.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

import asyncio
import PyKDL
import numpy as np
from tracikpy import TracIKSolver

class MovementCoordinator(Node):

    """
    Movement Node that coordinates the robot's search pattern and movement towards the detected object.
        - Subscribes to /joint_states for current joint positions.
        - Subscribes to /fusion/status for updates from the Vision Node.
        - Subscribes to /object/pose_raw for the detected object's pose.
        - Publishes joint commands to /joint_command.
        - Publishes "SNAP" commands to /fusion/command to trigger the Vision Node to look for the object.   
    """

    def __init__(self):
        super().__init__('movement_coordinator')
        
        # -- ROS Publishers --
        self.joint_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.cmd_pub = self.create_publisher(String, '/fusion/command', 10)

        # -- ROS Subscribers --
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 10)
        self.status_sub = self.create_subscription(String, '/fusion/status', self.status_cb, 10)
        self.object_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/object/pose_raw', self.object_pose_cb, 10)

        # -- Robot Joints --
        self.panda_joints = [
            "panda_joint1", "panda_joint2", "panda_joint3",
            "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7",
        ]   

        # -- Search Viewpoints (Predefined Joint Configurations) --
        self.search_viewpoints = [
            [1.551, -0.696, -0.203, -1.076, 0.016, 1.002, 0.579],
            [-0.016, -0.696, -0.203, -1.076, 0.016, 1.002, 0.579],
            [-1.519, -0.696, -0.203, -1.076, 0.016, 1.002, 0.579],
        ]

        # -- TRAC-IK Setup --
        self.urdf_path = "/home/affan/Documents/FY_Project/isaac_sim_assets/franka.urdf"
        self.base_link = "World" 
        self.ee_link = "franka_panda_hand" 
        self.InitializeTRACIK()

        # -- State Variables --
        self.waiting_for_vision = False
        self.object_found = False
        self.current_joints = None
        self.found_pose = None

    def joint_states_cb(self, msg):

        """
        Extract the current joint positions from the incoming JointState message.
        We look for the indices of our robot's joints in the message and store their positions.
        """

        try:
            positions = []
            for name in self.panda_joints:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
            self.current_joints = np.array(positions)
        except (ValueError, IndexError):
            pass

    def status_cb(self, msg):

        """
        Listen for status updates from the Vision Node.
        When we receive "SUCCESS", it means the Vision Node has found the object and published its pose.
        We then set our internal state to reflect that we have the object's pose and can proceed with IK and movement.
        """

        if msg.data == "SUCCESS": # Vision Node found the object and published its pose
            self.object_found = True
            self.waiting_for_vision = False

    def object_pose_cb(self, msg):

        """
        This callback is triggered when the Vision Node publishes the detected object's pose.
        We store the pose in self.found_pose and set self.object_found to True, which signals that we can proceed with IK and movement.
        """

        self.found_pose = msg.pose.pose
        self.object_found = True
        self.get_logger().info(f"📍 Target Pose Received")
    
    def InitializeTRACIK(self):
        
        """
        Initialize the TRAC-IK solver with the robot's URDF, base link, and end-effector link.
        We set a short timeout and a small epsilon for precision. We also log the number of joints that TRAC-IK recognizes for our robot, which should match our expectations based on the URDF.
        """

        self.ik_solver = TracIKSolver(
            self.urdf_path,
            self.base_link, 
            self.ee_link, 
            timeout=0.20, # Time in seconds to wait for a solution before giving up 
            epsilon=1e-5, # The precision of the solution
            solve_type="Distance" # Minimize the distance to the target pose
        )

        self.n_joints = len(self.ik_solver.joint_names) # Number of joints that TRAC-IK recognizes for our robot
        self.get_logger().info(f"TRAC-IK initialized with {self.n_joints} joints")

    def InverseKinematics(self, offset):

        """
        This function computes the inverse kinematics for the detected object's pose using the TRAC-IK solver.
        It constructs a desired end-effector pose that looks at the object from the left side (+Y direction) and is pitched up 90 degrees to look down at the object.
        """

        if self.found_pose is None:
            return None


        seed_state = self.current_joints if self.current_joints is not None else np.zeros(7)
        pos = self.found_pose.position

        final_rot = PyKDL.Rotation.EulerZYX(offset[0], offset[1], offset[2])

        ee_pose = np.eye(4)
        for r in range(3):
            for c in range(3):
                ee_pose[r, c] = final_rot[r, c]
        
        ee_pose[0, 3] = float(pos.x) + offset[3]
        ee_pose[1, 3] = float(pos.y) + offset[4]
        ee_pose[2, 3] = float(pos.z) + offset[5] 

        try:
            qout = self.ik_solver.ik(ee_pose, qinit=seed_state)

            if qout is not None:
                return qout.tolist()
            else:
                self.get_logger().error("❌ IK Failed: Check if 0.3m offset is within reach")
                return None
            
        except Exception as e:
            self.get_logger().error(f"❌ IK Solver Error: {str(e)}")
            return None
        
    async def search_for_object(self):

        """
        This function implements the search pattern for the object. It iterates through a predefined list of joint configurations (viewpoints), 
        commands the robot to move to each viewpoint, and after arriving, it publishes a "SNAP" command to trigger the Vision Node to look for the object. 
        It waits for a short period to allow the Vision Node to process the image and update the status. If the object is found at any viewpoint, it breaks out of the loop and returns True. 
        If it exhausts all viewpoints without finding the object, it returns False.
        """

        self.get_logger().info("🔍 Starting Sweep")
        for _, pose in enumerate(self.search_viewpoints):
            await self.move_to_viewpoint(pose)
            # self.cmd_pub.publish(String(data="SNAP"))
            await asyncio.sleep(2.0) # Give vision time to process
            if self.object_found:
                break
        return self.object_found

    async def move_to_viewpoint(self, pose):

        """
        This function commands the robot to move to a specified joint configuration (viewpoint).
        It publishes the desired joint positions to the /joint_command topic and then waits until the robot has arrived at the target configuration within a specified threshold. 
        It uses the current joint states to calculate the error and determine when the robot has arrived. After arriving, 
        it waits for an additional second to ensure stability before proceeding.
        """

        target = np.array(pose)
        msg = JointState(name=self.panda_joints, position=pose)
        arrival_threshold = 0.015 
        arrived = False

        while not arrived and rclpy.ok():
            self.joint_pub.publish(msg)
            if self.current_joints is not None:
                error = np.max(np.abs(self.current_joints - target))
                if error < arrival_threshold:
                    arrived = True
                else:
                    await asyncio.sleep(0.1)
            else:
                await asyncio.sleep(0.5)
        await asyncio.sleep(1.0)


    async def run_scan_pattern(self):

        """
        This is the main function that runs the scan pattern. It first calls the search_for_object function to execute the predefined search pattern.
        If the object is found, it then calls the InverseKinematics function to compute the joint configuration needed to look at the object from the left side.
        If the IK solution is valid, it commands the robot to move to that configuration using the move_to_viewpoint function.
        """

        found = await self.search_for_object()

        view_offset = {
            # "front" : [0, -1.800, 0, -0.3, 0.0, 0.10],
            "left" : [1.5708, 0, 0, 0.0, 0.35, 0.30],
            "top" : [0, -1.5708, 0, 0.0, 0, 0.45],
            "right" : [-1.5708, 0, 0, 0.0, -0.35, 0.10],
        }

        if found:
            
            for view, offset in view_offset.items():

                self.get_logger().info(f"🔍 Attempting {view} side view")
                pose = self.InverseKinematics(offset)

                if pose:
                    await self.move_to_viewpoint(pose)
                    self.waiting_for_vision = True
                    self.cmd_pub.publish(String(data="SNAP"))

            self.cmd_pub.publish(String(data="SOLVE"))

# --- Main Function ---


async def run_node(node):

    """
    This function runs the main loop of the node using asyncio.
    """

    movement_task = asyncio.create_task(node.run_scan_pattern())
    while rclpy.ok() and not movement_task.done():
        rclpy.spin_once(node, timeout_sec=0)
        await asyncio.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = MovementCoordinator()
    try:
        asyncio.run(run_node(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()