import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        
        self.sub = self.create_subscription(PoseWithCovarianceStamped, '/object/pose_raw', self.callback, 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/object/pose_filtered', 10)

        # Kalman Parameters (Tuned for stationary bottle)
        self.R = 0.05      # Measurement Noise (Trust camera less)
        self.Q = 0.0001    # Process Noise (Bottle doesn't move)
        self.gate_threshold = 0.2  # Ignore jumps larger than 20cm

        self.x = None
        self.P = np.array([1.0, 1.0, 1.0])

    def callback(self, msg):
        z = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        # 1. Initialize
        if self.x is None:
            self.x = z
            self.get_logger().info("Filter Initialized!")
            return

        # 2. Outlier Rejection
        dist = np.linalg.norm(z - self.x)
        if dist > self.gate_threshold:
            self.get_logger().warn(f"Outlier ignored! Jumped {dist:.2f}m")
            return

        # 3. Kalman filter (independent XYZ)
        for i in range(3):
            p_minus = self.P[i] + self.Q
            k_gain = p_minus / (p_minus + self.R)
            self.x[i] = self.x[i] + k_gain * (z[i] - self.x[i])
            self.P[i] = (1 - k_gain) * p_minus

        # 4. Publish PoseWithCovarianceStamped
        out = PoseWithCovarianceStamped()
        out.header = msg.header

        out.pose.pose.position.x = float(self.x[0])
        out.pose.pose.position.y = float(self.x[1])
        out.pose.pose.position.z = float(self.x[2])

        # Pass-through orientation
        out.pose.pose.orientation = msg.pose.pose.orientation

        # Fill covariance (XYZ only)
        out.pose.covariance = [0.0] * 36
        out.pose.covariance[0]  = self.P[0]  # x
        out.pose.covariance[7]  = self.P[1]  # y
        out.pose.covariance[14] = self.P[2]  # z

        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()