"""
PoseCollector_2: ROS 2 Node for Object Detection and Coordinate Transformation.
This node utilizes the Deep Object Pose Estimation (DOPE) framework.

Reference:
Tremblay, J., To, T., Sundaralingam, B., Xiang, Y., Fox, D., & Birchfield, S. (2018).
Deep Object Pose Estimation for Semantic Robotic Grasping of Household Objects.
In Conference on Robot Learning (CoRL). arXiv:1809.10790.
"""

import rclpy
from rclpy.node import Node
import rclpy.duration

from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class PoseCollector_2(Node):

    """
    Vision Node that listens for "SNAP" commands, waits for the next valid detection, transforms it to the target frame, and publishes the pose to the Movement Node.
        - Subscribes to /fusion/command for "SNAP" commands.
        - Subscribes to /detections for incoming detections.
        - Publishes transformed poses to /object/pose_raw.   
    """

    def __init__(self):
        super().__init__('vision_node')

        # -- ROS Publishers --
        self.status_pub = self.create_publisher(String, '/fusion/status', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/object/pose_raw', 10)
        self.pose_pub_sweep = self.create_publisher(PoseWithCovarianceStamped, '/object/pose_raw_sweep', 10)

        # -- ROS Subscribers -- 
        self.subscription = self.create_subscription(Detection3DArray, '/detections', self.detections_callback, 10)
        self.command_sub = self.create_subscription(String, '/fusion/command', self.command_callback, 10)
        
        # -- TF2 Setup --
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -- State Variables --
        self.target_frame = 'World'
        self.snap_requested = False
        self.sweep_snap_requested = False

        self.get_logger().info('👁️ Vision Node started. Waiting for "SNAP"')

    def command_callback(self, msg):

        """
        Handle incoming commands from Movement Node.
        """

        if msg.data == "SNAP":

            self.get_logger().info("📸 SNAP triggered! Waiting for next valid detection.")
            self.get_logger().info('1')
            self.snap_requested = True
        
        if msg.data == "SWEEP_SNAP":

            self.get_logger().info("📸 SWEEP SNAP triggered! Waiting for next valid detection.")
            self.sweep_snap_requested = True

    def detections_callback(self, msg):

        """
        Store the latest detection and transform it to the target frame when a SNAP command has been received.
        """
        
        if self.sweep_snap_requested:

            detection_header = msg.header

            for detection in msg.detections:
                for result in detection.results:

                    pose_stamped = PoseStamped()
                    pose_stamped.header = detection_header
                    pose_stamped.pose = result.pose.pose

                    try:
                        transformed_pose = self.tf_buffer.transform(pose_stamped, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.1))
                        fusion_msg = PoseWithCovarianceStamped()
                        fusion_msg.header = transformed_pose.header
                        fusion_msg.pose.pose = transformed_pose.pose
                        fusion_msg.pose.covariance = [0.0] * 36
                        self.pose_pub_sweep.publish(fusion_msg)
                        self.get_logger().info(f'✅ Pose published Successfully!')
                        self.sweep_snap_requested = False 
                        self.status_pub.publish(String(data="SUCCESS"))
                        return
                    
                    except Exception as e:

                        self.get_logger().error(f"❌ Transformation failed: {e}")
                        continue
        
        elif self.snap_requested:
            detection_header = msg.header

            for detection in msg.detections:
                for result in detection.results:
                    pose_stamped = PoseStamped()
                    pose_stamped.header = detection_header
                    pose_stamped.pose = result.pose.pose

                    try:
                        transformed_pose = self.tf_buffer.transform(
                            pose_stamped, 
                            self.target_frame, 
                            timeout=rclpy.duration.Duration(seconds=0.1)
                        )
                        fusion_msg = PoseWithCovarianceStamped()
                        fusion_msg.header = transformed_pose.header
                        fusion_msg.pose.pose = transformed_pose.pose
                        fusion_msg.pose.covariance = [0.0] * 36
                        self.pose_pub.publish(fusion_msg)
                        self.get_logger().info(f'✅ Pose published Successfully!')
                        self.snap_requested = False 
                        self.status_pub.publish(String(data="SUCCESS"))
                        return
                        
                    except Exception as e:
                        self.get_logger().warn(
                            f"⚠️ Transform failed (will retry): "
                            f"from '{detection_header.frame_id}' to '{self.target_frame}' - {e}"
                        )
                        continue 



# -- Main function --


def main(args=None):
    rclpy.init(args=args)
    node = PoseCollector_2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()