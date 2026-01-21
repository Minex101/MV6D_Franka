import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

import numpy as np

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        # Subscribe to raw poses from Vision Node
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, 
                                                     '/object/pose_raw', 
                                                     self.store_callback, 
                                                     10)
        
        # Listen for the SOLVE command from Movement Node
        self.cmd_sub = self.create_subscription(String, 
                                                '/fusion/command', 
                                                self.command_callback, 
                                                10)

        self.position = []
        self.orientation = []

    def store_callback(self, msg):
        """Store incoming poses for later fusion."""
        # Store position
        pos = msg.pose.pose.position
        self.position.append([pos.x, pos.y, pos.z])

        self.get_logger().info(f"Buffered snapshot {len(self.position)}")
        
    def command_callback(self, msg):
        """Trigger fusion upon receiving SOLVE command."""
        if msg.data == "SOLVE":
            if len(self.position) == 0:
                self.get_logger().error("Cannot solve: No poses captured!")
                return

            self.get_logger().info("🧮 Solving for Average Pose...")
            self.calculate_and_publish()

    def calculate_and_publish(self):
        """Calculate the average pose"""
        # Average Position
        avg_pos = np.mean(self.position, axis=0)

        self.get_logger().warn(f"Avg Position: x={avg_pos[0]:.3f}, y={avg_pos[1]:.3f}, z={avg_pos[2]:.3f}")

        # Reset for the next multi-view sequence
        self.position = []
        self.orientation = []

def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()