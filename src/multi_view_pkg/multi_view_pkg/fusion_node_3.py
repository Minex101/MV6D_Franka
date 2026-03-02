"""
Fusion Node for Multi-View Object Pose Estimation.
This node integrates multiple 3D detections to compute a refined object pose.

The position is mean averaged, while the orientation 
averaging is based on the Singular Value Decomposition (SVD) method.

Reference:
Markley, F. L., Cheng, Y., Crassidis, J. L., & Oshman, Y. (2007). 
"Averaging Quaternions." Journal of Guidance, Control, and Dynamics, 
30(4), 1193-1197. https://doi.org/10.2514/1.28949
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

import numpy as np

class FusionNode(Node):
    def __init__(self):

        """
        ROS 2 Node that performs multi-view pose fusion.
        Subscribes to raw pose estimates and commands, and publishes the fused pose.
        """

        super().__init__('fusion_node')

        # -- ROS Publishers --
        self.publisher = self.create_publisher(PoseWithCovarianceStamped,'/object/pose_fused',10)

        # -- ROS Subscribers --
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, '/object/pose_raw', self.store_callback, 10)
        self.cmd_sub = self.create_subscription(String, '/fusion/command', self.command_callback, 10)

        # -- State Variables --
        self.position = []
        self.orientation = []

    def store_callback(self, msg):

        """
        Store incoming poses for later fusion when SOLVE command is received.
        """

        pos = msg.pose.pose.position # Extract position
        ori = msg.pose.pose.orientation # Extract orientation (quaternion)

        self.position.append([pos.x, pos.y, pos.z])
        self.orientation.append([ori.x, ori.y, ori.z, ori.w])

        self.get_logger().info(f"⟳ Buffered snapshot: {len(self.position)}")
        
    def command_callback(self, msg):

        """
        Trigger fusion calculation when SOLVE command is received.
        """

        if msg.data == "SOLVE": # Check for "SOLVE" command
            if len(self.position) == 0:
                self.get_logger().error("❌ Cannot solve: No poses captured!")
                return

            self.get_logger().info("🛡️ Solving for Average Pose.")
            self.calculate_and_publish() # Calculate average pose and publish

    def calculate_and_publish(self):

        """
        Calculate the average position and orientation from buffered poses and publish the fused result.
        """

        positions = np.array(self.position)

        # ---- Median ----
        avg_pos = np.median(positions, axis=0)

        # ---- Outlier Rejection + Markley ----
        filtered_quats = self.filter_quaternion_outliers(self.orientation, threshold_deg=20)
        avg_ori = self.average_quarternions_markley(filtered_quats)

        self.get_logger().warn(f"🛡️ Fused Pos: {avg_pos}")
        self.get_logger().warn(f"🛡️ Fused Ori: {avg_ori}")

        fused_msg = PoseWithCovarianceStamped() # Create message for publishing
        fused_msg.header.stamp = self.get_clock().now().to_msg()
        fused_msg.header.frame_id = "World"
        
        fused_msg.pose.pose.position.x = avg_pos[0]
        fused_msg.pose.pose.position.y = avg_pos[1]
        fused_msg.pose.pose.position.z = avg_pos[2]

        fused_msg.pose.pose.orientation.x = avg_ori[0]
        fused_msg.pose.pose.orientation.y = avg_ori[1]
        fused_msg.pose.pose.orientation.z = avg_ori[2]
        fused_msg.pose.pose.orientation.w = avg_ori[3]

        self.publisher.publish(fused_msg)
        self.get_logger().info("✅ Fused pose published.")

        self.position = [] # Clear buffers after publishing
        self.orientation = []

    def filter_quaternion_outliers(self, quaternions, threshold_deg=20):

        """
        Filter quaternion outliers based on angular distance from preliminary Markley average.
        """

        Q = np.array(quaternions)

        # Preliminary Markley average
        M = np.dot(Q.T, Q)
        M /= len(Q)
        eigenvalues, eigenvectors = np.linalg.eigh(M)
        avg_q = eigenvectors[:, np.argmax(eigenvalues)]
        avg_q /= np.linalg.norm(avg_q)

        # Angular distance computation
        dots = np.abs(np.dot(Q, avg_q))
        angles = 2 * np.arccos(np.clip(dots, -1.0, 1.0))
        angles_deg = np.degrees(angles)

        # Inlier selection
        inliers = Q[angles_deg < threshold_deg]

        # Fallback if all rejected
        if len(inliers) == 0:
            return Q

        return inliers

    def average_quarternions_markley(self, quaternions):

        """
        Average quarternions using Markley et al. method.
        Finds the optimal average quaternion by solving the 
        eigenvalue problem of the second-moment matrix of the quaternions.
        """

        Q = np.array(quaternions) # Convert list of quaternions to numpy array
        M = np.dot(Q.T, Q) # Compute the second-moment matrix
        M /= len(quaternions) # Normalize by the number of quaternions
        eigenvalues, eigenvectors = np.linalg.eigh(M) # Compute eigenvalues and eigenvectors (eighen decomposition)
        avg_quat = eigenvectors[:, np.argmax(eigenvalues)] # The eigenvector with the largest eigenvalue is the average quaternion

        return avg_quat / np.linalg.norm(avg_quat) # Normalize the average quaternion before returning


# -- Main Function --


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()