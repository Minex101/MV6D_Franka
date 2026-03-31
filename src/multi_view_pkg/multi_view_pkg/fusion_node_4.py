import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String, Float32
import numpy as np


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        # -- Publisher --
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/object/pose_fused',
            10
        )

        # -- Subscribers --
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/object/pose_raw',
            self.store_pose_callback,
            10
        )
        self.residual_sub = self.create_subscription(
            Float32,
            '/robot/depth_residual',
            self.store_residual_callback,
            10
        )
        self.cmd_sub = self.create_subscription(
            String,
            '/fusion/command',
            self.command_callback,
            10
        )

        # -- State --
        self.positions = []
        self.orientations = []
        self.residuals = []

        self.eps = 1e-3

        self.get_logger().info('🔀 Fusion Node Ready (Depth-Weighted Only)')

    def store_pose_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        self.positions.append([pos.x, pos.y, pos.z])
        self.orientations.append([ori.x, ori.y, ori.z, ori.w])

        self.get_logger().info(f'⟳ Buffered snapshot: {len(self.positions)}')

    def store_residual_callback(self, msg):
        self.residuals.append(float(msg.data))

    def command_callback(self, msg):
        if msg.data == 'SOLVE':
            if len(self.positions) == 0 or len(self.residuals) == 0:
                self.get_logger().error('❌ Cannot solve: no poses or residuals captured!')
                return

            self.get_logger().info('🛡️ Solving using depth-based weighted fusion.')
            self.calculate_and_publish()

    def calculate_and_publish(self):
        positions = np.array(self.positions, dtype=np.float64)
        quats = np.array(self.orientations, dtype=np.float64)
        residuals = np.array(self.residuals, dtype=np.float64)

        min_len = min(len(positions), len(quats), len(residuals))
        positions = positions[:min_len]
        quats = quats[:min_len]
        residuals = residuals[:min_len]

        if min_len == 0:
            self.get_logger().error('❌ No valid buffered samples after alignment.')
            return

        # Convert uncertainty to confidence weight
        # Lower residual -> higher weight
        weights = 1.0 / (residuals + self.eps)
        weights_sum = np.sum(weights)

        if weights_sum <= 0 or not np.isfinite(weights_sum):
            self.get_logger().warn('⚠️ Invalid weights, falling back to uniform weights')
            weights = np.ones(min_len, dtype=np.float64)
            weights_sum = np.sum(weights)

        weights /= weights_sum

        self.get_logger().info(f'Residuals: {np.round(residuals, 4)}')
        self.get_logger().info(f'Weights:   {np.round(weights, 4)}')

        # Weighted translation average
        avg_pos = np.average(positions, axis=0, weights=weights)

        # Weighted quaternion fusion (chordal L2 / Markley-style)
        M = np.zeros((4, 4), dtype=np.float64)
        for q, w in zip(quats, weights):
            q = q / np.linalg.norm(q)
            M += w * np.outer(q, q)

        eigenvals, eigenvecs = np.linalg.eigh(M)
        avg_quat = eigenvecs[:, np.argmax(eigenvals)]
        avg_quat /= np.linalg.norm(avg_quat)

        self.get_logger().warn(f'🛡️ Fused Pos: {np.round(avg_pos, 4)}')
        self.get_logger().warn(f'🛡️ Fused Ori: {np.round(avg_quat, 4)}')

        fused_msg = PoseWithCovarianceStamped()
        fused_msg.header.stamp = self.get_clock().now().to_msg()
        fused_msg.header.frame_id = 'World'

        fused_msg.pose.pose.position.x = float(avg_pos[0])
        fused_msg.pose.pose.position.y = float(avg_pos[1])
        fused_msg.pose.pose.position.z = float(avg_pos[2])

        fused_msg.pose.pose.orientation.x = float(avg_quat[0])
        fused_msg.pose.pose.orientation.y = float(avg_quat[1])
        fused_msg.pose.pose.orientation.z = float(avg_quat[2])
        fused_msg.pose.pose.orientation.w = float(avg_quat[3])

        self.publisher.publish(fused_msg)
        self.get_logger().info('✅ Fused pose published.')

        # Clear buffers
        self.positions = []
        self.orientations = []
        self.residuals = []


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()