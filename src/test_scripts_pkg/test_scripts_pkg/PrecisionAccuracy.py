"""
A simple ROS2 implementation to get the precision and accuracy of position estimates (X, Y, Z) and fused poses
compared to ground truth. 
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped

import matplotlib.pyplot as plt

class PrecisionAccuracyTester(Node):
    def __init__(self):
        super().__init__('precision_accuracy_tester')

        # Subscribe to raw poses from Vision Node
        self.poses = self.create_subscription(PoseWithCovarianceStamped, 
                                                     '/object/pose_raw', 
                                                     self.pose_callback, 
                                                     10)
        
        # Subscribe to fused poses from Fusion Node
        self.fused_poses = self.create_subscription(PoseWithCovarianceStamped,
                                                     '/object/pose_fused',
                                                     self.fused_pose_callback,
                                                     10)
        # Data storage
        self.raw_X, self.raw_Y, self.raw_Z = [], [], []
        self.fused_X, self.fused_Y, self.fused_Z = [], [], []

        # Setup Matplotlib for live plotting
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 10))

        # Timer to plot results after a certain duration
        self.timer = self.create_timer(0.1, self.plot_results)

    def pose_callback(self, msg):
        """Handle incoming raw poses for precision analysis."""
        self.raw_X.append(msg.pose.pose.position.x)
        self.raw_Y.append(msg.pose.pose.position.y)
        self.raw_Z.append(msg.pose.pose.position.z)

    def fused_pose_callback(self, msg):
        """Handle incoming fused poses for accuracy analysis."""
        self.fused_X.append(msg.pose.pose.position.x)
        self.fused_Y.append(msg.pose.pose.position.y)
        self.fused_Z.append(msg.pose.pose.position.z)
    
    def plot_results(self):
        """Plot the precision and accuracy results."""

        # Clear axes to redraw
        self.ax1.clear()
        self.ax2.clear()

        # Subplot 1: X - Y Position

        # Raw Positions
        self.ax1.scatter(self.raw_X, self.raw_Y, c="red", label='Raw Pose Estimates', alpha=0.5)
        # Fused Positions
        if self.fused_X and self.fused_Y:
            self.ax1.scatter(self.fused_X, self.fused_Y, c="blue", label='Fused Pose Estimate', alpha=0.8)
        # Ground truth
        self.ax1.scatter([0.871], [0.024], c="green", label='Ground Truth', s=100, marker='x')
        self.ax1.set_title('X-Y Position Precision and Accuracy')
        self.ax1.set_xlabel('X Position (m)')
        self.ax1.set_ylabel('Y Position (m)')
        self.ax1.legend()
        self.ax1.grid(True)

        # Subplot 2: Z Position
        
        indices = range(len(self.raw_Z))
        # Raw Z positions
        self.ax2.plot(indices, self.raw_Z, c="red", linestyle='--', label='Raw Pose z', alpha=0.4)
        # scatter raw Z points
        self.ax2.scatter(indices, self.raw_Z, c='red', s=10)
        # Fused Z positions
        # Draw a horizontal line for the final fused Z
        if self.fused_Z:
            self.ax2.axhline(y=self.fused_Z[-1], color='blue', linestyle='-', label='Fused Pose Z')
        # Ground Truth Z line
        self.ax2.axhline(y=0.069, color='green', linestyle='-', label='Ground Truth Z')
        self.ax2.set_title("Z Position Precision and Accuracy")
        self.ax2.set_xlabel("Snapshot Number")
        self.ax2.set_ylabel("Z (meters)")
        self.ax2.legend()
        self.ax2.grid(True)

        plt.draw()
        plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = PrecisionAccuracyTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()