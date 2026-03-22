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
        super().__init__('fusion_node')

        # -- Publishers --
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/object/pose_fused', 10)

        # -- Subscribers --
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, '/object/pose_raw',        self.store_pose_callback,     10)
        self.residual_sub = self.create_subscription(Float32,                   '/robot/depth_residual',   self.store_residual_callback, 10)
        self.cmd_sub      = self.create_subscription(String,                    '/fusion/command',         self.command_callback,        10)

        # -- State --
        self.position  = []
        self.orientation = []
        self.residuals = []

        self.depth_threshold    = 0.15
        self.outlier_threshold  = 0.15

        self.get_logger().info('🔀 Fusion Node Ready')

    def store_pose_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.position.append([pos.x, pos.y, pos.z])
        self.orientation.append([ori.x, ori.y, ori.z, ori.w])
        self.get_logger().info(f'⟳ Buffered snapshot: {len(self.position)}')

    def store_residual_callback(self, msg):
        self.residuals.append(float(msg.data))

    def command_callback(self, msg):
        if msg.data == 'SOLVE':
            if len(self.position) == 0 or len(self.residuals) == 0:
                self.get_logger().error('❌ Cannot solve: No poses or residuals captured!')
                return
            self.get_logger().info('🛡️ Solving for Weighted Fusion Pose.')
            self.calculate_and_publish()

    def calculate_and_publish(self):
        positions  = np.array(self.position)
        quats      = np.array(self.orientation)
        residuals  = np.array(self.residuals)

        min_len    = min(len(positions), len(residuals), len(quats))
        positions  = positions[:min_len]
        quats      = quats[:min_len]
        residuals  = residuals[:min_len]

        # Depth residual weights
        weights = np.exp(-residuals / (self.depth_threshold + 1e-6))
        if np.sum(weights) == 0:
            self.get_logger().warn('⚠️ All weights zero, using uniform weights')
            weights = np.ones(min_len)
        weights /= np.sum(weights)

        # Consensus reweighting
        median_pos      = np.median(positions, axis=0)
        distances       = np.linalg.norm(positions - median_pos, axis=1)
        consensus_w     = np.exp(-distances / (self.outlier_threshold + 1e-6))
        consensus_w    /= np.sum(consensus_w)

        # Combine depth residual weight and consensus weight equally
        combined_w      = 0.5 * weights + 0.5 * consensus_w
        combined_w     /= np.sum(combined_w)

        self.get_logger().info(f'Depth weights:     {np.round(weights, 3)}')
        self.get_logger().info(f'Consensus weights: {np.round(consensus_w, 3)}')
        self.get_logger().info(f'Combined weights:  {np.round(combined_w, 3)}')

        # Outlier rejection
        inlier_mask = distances < self.outlier_threshold
        min_views   = max(1, len(positions) // 2 + 1)
        if np.sum(inlier_mask) >= min_views:
            positions  = positions[inlier_mask]
            quats      = quats[inlier_mask]
            combined_w = combined_w[inlier_mask]
            combined_w /= np.sum(combined_w)
            self.get_logger().info(f'✂️ Outlier rejection: kept {np.sum(inlier_mask)}/{min_len} views')
        else:
            self.get_logger().warn('⚠️ Outlier rejection skipped: would remove too many views')

        # Weighted translation average
        avg_pos = np.average(positions, axis=0, weights=combined_w)

        # Weighted chordal L2 quaternion fusion
        M = np.zeros((4, 4))
        for q, w in zip(quats, combined_w):
            M += w * np.outer(q, q)
        eigenvals, eigenvecs = np.linalg.eigh(M)
        avg_quat = eigenvecs[:, np.argmax(eigenvals)]
        avg_quat /= np.linalg.norm(avg_quat)

        self.get_logger().warn(f'🛡️ Fused Pos: {np.round(avg_pos, 4)}')
        self.get_logger().warn(f'🛡️ Fused Ori: {np.round(avg_quat, 4)}')

        fused_msg                          = PoseWithCovarianceStamped()
        fused_msg.header.stamp             = self.get_clock().now().to_msg()
        fused_msg.header.frame_id          = 'World'
        fused_msg.pose.pose.position.x     = float(avg_pos[0])
        fused_msg.pose.pose.position.y     = float(avg_pos[1])
        fused_msg.pose.pose.position.z     = float(avg_pos[2])
        fused_msg.pose.pose.orientation.x  = float(avg_quat[0])
        fused_msg.pose.pose.orientation.y  = float(avg_quat[1])
        fused_msg.pose.pose.orientation.z  = float(avg_quat[2])
        fused_msg.pose.pose.orientation.w  = float(avg_quat[3])

        self.publisher.publish(fused_msg)
        self.get_logger().info('✅ Fused pose published.')
        
        self.position    = []
        self.orientation = []
        self.residuals   = []


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()