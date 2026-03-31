import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
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
        self.cmd_sub = self.create_subscription(
            String,
            '/fusion/command',
            self.command_callback,
            10
        )

        # -- State --
        self.positions = []
        self.orientations = []

        self.get_logger().info('Fusion Node Ready')

    def store_pose_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        self.positions.append([pos.x, pos.y, pos.z])
        self.orientations.append([ori.x, ori.y, ori.z, ori.w])

        self.get_logger().info(f'Buffered snapshot: {len(self.positions)}')

    def command_callback(self, msg):
        if msg.data == 'SOLVE':
            if len(self.positions) == 0:
                self.get_logger().error('Cannot solve: no poses captured!')
                return

            self.get_logger().info('Solving using simple mean fusion.')
            self.calculate_and_publish()

    def calculate_and_publish(self):
        positions = np.array(self.positions)
        quats = np.array(self.orientations)

        # --- Simple mean for translation ---
        avg_pos = np.mean(positions, axis=0)

        # --- Naive quaternion fusion ---
        # Component-wise mean, then normalize
        avg_quat = np.mean(quats, axis=0)
        norm = np.linalg.norm(avg_quat)

        if norm == 0:
            self.get_logger().warn('Quaternion mean has zero norm, using default identity quaternion.')
            avg_quat = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            avg_quat = avg_quat / norm

        self.get_logger().info(f'Fused Pos: {np.round(avg_pos, 4)}')
        self.get_logger().info(f'Fused Ori: {np.round(avg_quat, 4)}')

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
        self.get_logger().info('Fused pose published.')

        # Clear buffer
        self.positions = []
        self.orientations = []


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()