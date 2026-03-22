import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import asyncio
import numpy as np
import time

class MovementCoordinator(Node):
    def __init__(self):
        super().__init__('movement_coordinator')
        
        self.joint_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.cmd_pub = self.create_publisher(String, '/fusion/command', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 10)
        self.status_sub = self.create_subscription(String, '/fusion/status', self.status_cb, 10)
        
        self.waiting_for_vision = False
        self.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]

        self.viewpoints = [
            [0.454, 0.162, 0.000, -2.260, -0.517, 2.061, 2.710],
            # [0.00, 0.00, 0.00, -1.871, -0.110, 1.613, 0.00],
            [-0.266, 0.143, -0.235, -2.017, 0.360, 1.837, -1.582],
            [-1.989, 0.124, 0.486, -2.699, 0.924, 1.674, -2.083],

            [-1.613, 0.715, 0.861, -1.887, 0.298, 1.674, -1.613],
            # [0.830, 0.448, 0.00, -1.709, -0.611, 1.429, 2.897],
            # [0.141, -0.295, -0.110, -1.563, -0.047, 1.225, 0.798],
            # [-1.989, 0.124, 0.486, -2.699, 0.924, 1.674, -2.083],
        ]

        self.current_joints = None

    def joint_states_cb(self, msg):
        try:
            positions = []
            for name in self.joint_names:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
            self.current_joints = np.array(positions)
        except (ValueError, IndexError):
            pass

    def status_cb(self, msg):
        if msg.data == "SUCCESS":
            self.get_logger().info("Handshake Received: Vision captured successfully!")
            self.waiting_for_vision = False

    async def move_to_viewpoint(self, pose):
        target = np.array(pose)
        msg = JointState(name=self.joint_names, position=pose)
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

        await asyncio.sleep(2.0)

    async def run_scan_pattern(self):
        start_time = self.get_clock().now()

        for i, pose in enumerate(self.viewpoints):
            await self.move_to_viewpoint(pose)

            self.waiting_for_vision = True
            self.cmd_pub.publish(String(data="SNAP"))

            wait_start = time.time()
            timeout = 5.0
            
            while self.waiting_for_vision and rclpy.ok():
                elapsed = time.time() - wait_start
                if elapsed > timeout:
                    self.get_logger().warn(f"TIMEOUT: No object detected at Viewpoint {i+1}. Moving on...")
                    self.waiting_for_vision = False
                    break
                await asyncio.sleep(0.1) 

        self.cmd_pub.publish(String(data="SOLVE"))

        end_time = self.get_clock().now()
        total_duration = (end_time - start_time).nanoseconds / 1e9
        
        self.get_logger().info(f"Scan Complete. Total Time: {total_duration:.2f}s")

async def run_node(node):
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