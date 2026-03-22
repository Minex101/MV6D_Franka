import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import String

import asyncio
import numpy as np

class MovementCoordinator(Node):
    def __init__(self):
        super().__init__('movement_coordinator')
        
        # Publisher for joint commands, fusion commands and subscriber for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.cmd_pub = self.create_publisher(String, '/fusion/command', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 10)

        # Subscriber for fusion status
        self.status_sub = self.create_subscription(String, '/fusion/status', self.status_cb, 10)
        
        self.waiting_for_vision = False

        # Franka Panda joint names
        self.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]

        # Predefined viewpoints (joint positions)
        self.viewpoints = [
            # [0.266, 0.124, -0.235, -2.228, 0.266, 3.100, 0.611],
            # [2.647, -0.943, -2.897, -1.141, 0.767, 2.244, -0.830],
            # [0.486, 1.058, -0.016, -0.686, -0.830, 1.817, 2.333],
            # # [0.000, 0.000, 0.000, -1.571, 0.078, 1.939, 0.736], # bad
            # # [-0.642, 0.105, 0.486, -2.098, 0.579, 2.672, -0.329], # bad
            # # [0.486, 0.295, -0.141, -1.660, -0.548, 2.061, 1.989],

            # [0.266, 0.124, -0.235, -2.228, 0.266, 3.100, 0.611],
            # [2.647, -0.943, -2.897, -1.141, 0.767, 2.244, -0.830],
            [0.486, 1.058, -0.016, -0.686, -0.830, 1.817, 2.333],
            [0.000, 0.000, 0.000, -1.571, 0.078, 1.939, 0.736], # bad
            [-0.642, 0.105, 0.486, -2.098, 0.579, 2.672, -0.329], # bad
            # [0.486, 0.295, -0.141, -1.660, -0.548, 2.061, 1.989],
        ]

        # Current joint state
        self.current_joints = None

    def joint_states_cb(self, msg):
        """Update current joint positions, ensuring correct name mapping."""
        try:
            positions = []
            for name in self.joint_names:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
            self.current_joints = np.array(positions)
        except (ValueError, IndexError):
            pass

    def status_cb(self, msg):
        """Handle handshake from Vision Node indicating capture success."""
        if msg.data == "SUCCESS":
            self.get_logger().info("Handshake Received: Vision captured successfully!")
            self.waiting_for_vision = False

    async def move_to_viewpoint(self, pose):
        """Move robot to the specified joint pose."""
        target = np.array(pose)
        msg = JointState(name=self.joint_names, position=pose)
        
        arrival_threshold = 0.015 # Radians (0.8 degrees)
        arrived = False
        
        self.get_logger().info("Moving to target...")

        # Wait for physical arrival
        while not arrived and rclpy.ok():
            self.joint_pub.publish(msg)
            
            if self.current_joints is not None:
                error = np.max(np.abs(self.current_joints - target))
                if error < arrival_threshold:
                    arrived = True
                else:
                    await asyncio.sleep(0.1)
            else:
                self.get_logger().warn("Waiting for robot to reach viewpoint...")
                await asyncio.sleep(0.5)

        self.get_logger().info("Arrived. Waiting 2 seconds for camera settle...")
        await asyncio.sleep(0.0)

        self.get_logger().info("Robot settled.")

    async def run_scan_pattern(self):
        """This is the core logic loop."""
        self.get_logger().info("Starting Multi-View Scan...")
        for i, pose in enumerate(self.viewpoints):
            self.get_logger().info(f"\nMoving to Viewpoint {i + 1}/{len(self.viewpoints)}")
            
            await self.move_to_viewpoint(pose)

            self.get_logger().info("Triggering SNAP...")
            self.waiting_for_vision = True
            self.cmd_pub.publish(String(data="SNAP"))

            # Handshake wait
            while self.waiting_for_vision and rclpy.ok():
                await asyncio.sleep(0.1) 

        self.get_logger().info("All shots taken. Triggering SOLVE...")
        self.cmd_pub.publish(String(data="SOLVE"))
        self.get_logger().info("Scan Complete.")

async def run_node(node):
    """A helper function to manage the task and the ROS spin together."""
    movement_task = asyncio.create_task(node.run_scan_pattern())
    
    # Continue spinning while the movement task is running
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