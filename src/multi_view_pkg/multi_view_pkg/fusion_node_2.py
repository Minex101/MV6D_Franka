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
        # Publisher for fused pose
        self.publisher = self.create_publisher(PoseWithCovarianceStamped,
                                                '/object/pose_fused',
                                                10)

        self.position = []
        self.orientation = []

    def store_callback(self, msg):
        """Store incoming poses for later fusion."""
        # Store position
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        self.position.append([pos.x, pos.y, pos.z])
        self.orientation.append([ori.x, ori.y, ori.z, ori.w])

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
        
        # Average Orientation (Markley et al. method)
        avg_ori = self.average_quarternions_markley(self.orientation)

        self.get_logger().warn(f"Fused Pos: {avg_pos}")
        self.get_logger().warn(f"Fused Ori: {avg_ori}")

        # 3. Create and publish message
        fused_msg = PoseWithCovarianceStamped()
        fused_msg.header.stamp = self.get_clock().now().to_msg()
        fused_msg.header.frame_id = "world"
        
        fused_msg.pose.pose.position.x = avg_pos[0]
        fused_msg.pose.pose.position.y = avg_pos[1]
        fused_msg.pose.pose.position.z = avg_pos[2]

        fused_msg.pose.pose.orientation.x = avg_ori[0]
        fused_msg.pose.pose.orientation.y = avg_ori[1]
        fused_msg.pose.pose.orientation.z = avg_ori[2]
        fused_msg.pose.pose.orientation.w = avg_ori[3]

        self.publisher.publish(fused_msg)
        self.get_logger().info("✅ High-accuracy fused pose published.")

        # Reset buffers
        self.position = []
        self.orientation = []

    def average_quarternions_markley(self, quaternions):
        """Average quarternions using Markley et al. method.
        Finds the optimal average quaternion by solving the 
        eigenvalue problem of the second-moment matrix of the quaternions.
        """
        # Convert to matrice
        Q = np.array(quaternions)

        # Compute the symmetric accumulator matrix (M)
        M = np.dot(Q.T, Q)

        # Normalize by the samples
        M /= len(quaternions)

        # Solve the eigendecomposition
        eigenvalues, eigenvectors = np.linalg.eigh(M)

        # Return the eigenvector with largest eigenvalue
        avg_quat = eigenvectors[:, np.argmax(eigenvalues)]

        # Ensure the result is a unit quaternion
        return avg_quat / np.linalg.norm(avg_quat)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()