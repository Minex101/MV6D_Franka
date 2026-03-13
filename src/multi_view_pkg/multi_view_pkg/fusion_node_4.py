"""
Fusion Node for Multi-View Object Pose Estimation.
This node integrates multiple 3D detections to compute a refined object pose.

The position is fused using weighted translation average, 
while orientation is fused using weighted chordal L2 quaternion averaging.

Weights are based on per-view uncertainty (depth residuals from /robot/depth_residual).
Views with lower residuals contribute more to the fused pose.

Reference:
- Smith, R., & Cheeseman, P. (1986). On the Representation and Estimation of Spatial Uncertainty. IJRR.
- Pomerleau, F., Colas, F., Siegwart, R. (2015). A Review of Point Cloud Registration Algorithms for Mobile Robotics.
- Hartley, R., Trumpf, J., Dai, Y., & Li, H. (2013). Rotation Averaging. IJCV.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String, Float32

import numpy as np

class FusionNode(Node):
    def __init__(self):

        """
        ROS 2 Node that performs multi-view pose fusion.
        Subscribes to raw pose estimates, depth residuals, and commands, then publishes the fused pose.
        """

        super().__init__('fusion_node')

        # -- ROS Publishers --
        self.publisher = self.create_publisher(PoseWithCovarianceStamped,'/object/pose_fused',10)

        # -- ROS Subscribers --
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, '/object/pose_raw', self.store_pose_callback, 10)
        self.residual_sub  = self.create_subscription(Float32, '/robot/depth_residual', self.store_residual_callback, 10)
        self.cmd_sub       = self.create_subscription(String, '/fusion/command', self.command_callback, 10)

        # -- State Variables --
        self.position = []
        self.orientation = []
        self.residuals = []  # per-view residuals
        self.depth_threshold = 0.005  # maximum expected normalised depth variance (0.0-1.0)

    def store_pose_callback(self, msg):

        """
        Store incoming poses for later fusion when SOLVE command is received.
        """

        pos = msg.pose.pose.position # Extract position
        ori = msg.pose.pose.orientation # Extract orientation (quaternion)

        self.position.append([pos.x, pos.y, pos.z])
        self.orientation.append([ori.x, ori.y, ori.z, ori.w])

        self.get_logger().info(f"⟳ Buffered snapshot: {len(self.position)}")

    def store_residual_callback(self, msg):

        """
        Store depth residual for the corresponding pose.
        """

        self.residuals.append(float(msg.data))

    def command_callback(self, msg):

        """
        Trigger fusion calculation when SOLVE command is received.
        """

        if msg.data == "SOLVE": # Check for "SOLVE" command
            if len(self.position) == 0 or len(self.residuals) == 0:
                self.get_logger().error("❌ Cannot solve: No poses or residuals captured!")
                return

            # Ensure lists match
            min_len = min(len(self.position), len(self.residuals), len(self.orientation))
            self.position = self.position[:min_len]
            self.orientation = self.orientation[:min_len]
            self.residuals   = self.residuals[:min_len]

            self.get_logger().info("🛡️ Solving for Weighted Fusion Pose.")
            self.calculate_and_publish() # Calculate average pose and publish

    def calculate_and_publish(self):

        """
        Calculate the weighted average position and chordal L2 quaternion orientation
        from buffered poses using residual-based weights, and publish the fused result.
        """

        positions = np.array(self.position)
        quats = np.array(self.orientation)
        residuals = np.array(self.residuals)

        # Convert residuals to weights (low residual = high weight)
        weights = np.exp(-residuals / (self.depth_threshold + 1e-6))
        print(f"Raw weights: {weights}")
        if np.sum(weights) == 0:
            self.get_logger().warn("⚠️ All residuals exceed threshold, using uniform weights")
            weights = np.ones_like(weights)
        weights /= np.sum(weights)  # normalize

        # ---- Outlier Rejection ----
        median_pos = np.median(positions, axis=0)
        distances = np.linalg.norm(positions - median_pos, axis=1)
        inlier_mask = distances < 0.15  # reject poses > 15cm from median
        min_views = max(1, len(positions) // 2 + 1) 
        if np.sum(inlier_mask) >= min_views:
            positions = positions[inlier_mask]
            quats = quats[inlier_mask]
            weights = weights[inlier_mask]
            weights /= np.sum(weights)  # renormalize
        else:
            self.get_logger().warn("⚠️ Outlier rejection skipped: would remove too many views, using all")

        # ---- Weighted Translation Average ----
        avg_pos = np.average(positions, axis=0, weights=weights)

        # ---- Weighted Chordal L2 Quaternion Fusion ----
        M = np.zeros((4,4))
        for q, w in zip(quats, weights):
            M += w * np.outer(q,q)
        eigenvals, eigenvecs = np.linalg.eigh(M)
        avg_quat = eigenvecs[:, np.argmax(eigenvals)]
        avg_quat /= np.linalg.norm(avg_quat)

        self.get_logger().warn(f"🛡️ Fused Pos: {avg_pos}")
        self.get_logger().warn(f"🛡️ Fused Ori: {avg_quat}")

        # ---- Publish fused pose ----
        fused_msg = PoseWithCovarianceStamped() # Create message for publishing
        fused_msg.header.stamp = self.get_clock().now().to_msg()
        fused_msg.header.frame_id = "World"
        
        fused_msg.pose.pose.position.x = avg_pos[0]
        fused_msg.pose.pose.position.y = avg_pos[1]
        fused_msg.pose.pose.position.z = avg_pos[2]

        fused_msg.pose.pose.orientation.x = avg_quat[0]
        fused_msg.pose.pose.orientation.y = avg_quat[1]
        fused_msg.pose.pose.orientation.z = avg_quat[2]
        fused_msg.pose.pose.orientation.w = avg_quat[3]

        self.publisher.publish(fused_msg)
        self.get_logger().info("✅ Fused pose published.")

        # ---- Clear buffers ----
        self.position = []
        self.orientation = []
        self.residuals = []

# -- Main Function --

def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()