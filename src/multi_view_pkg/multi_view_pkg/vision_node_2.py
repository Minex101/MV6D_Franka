import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from tf2_ros import Buffer, TransformListener
import rclpy.duration
import tf2_geometry_msgs

class PoseCollector_2(Node):
    def __init__(self):
        super().__init__('vision_node')

        # Detection Subscriber
        self.subscription = self.create_subscription(Detection3DArray, 
                                                     '/detections', 
                                                     self.detections_callback, 
                                                     10)
        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 
                                              '/object/pose_raw', 
                                              10)
        
        # Handshake Setup
        self.command_sub = self.create_subscription(String, 
                                                    '/fusion/command', 
                                                    self.command_callback, 
                                                    10)
        self.status_pub = self.create_publisher(String, 
                                                '/fusion/status', 
                                                10)

        # State Variables
        self.target_frame = 'World'
        self.snap_requested = False
        self.get_logger().info('Vision Node started. Waiting for SNAP...')

    def command_callback(self, msg):
        """Handle incoming commands from Movement Node."""
        if msg.data == "SNAP":
            self.get_logger().info("📸 SNAP requested! Waiting for next valid detection...")
            self.snap_requested = True

    def detections_callback(self, msg):
        """Store and transform incoming detections."""
        # ONLY process if a SNAP was requested
        if not self.snap_requested:
            return

        detection_header = msg.header
        for detection in msg.detections:
            for result in detection.results:

                pose_stamped = PoseStamped()
                pose_stamped.header = detection_header
                pose_stamped.pose = result.pose.pose

                try:
                    # Transform the stamped pose to the target frame
                    transformed_pose = self.tf_buffer.transform(
                        pose_stamped,
                        self.target_frame,
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )

                    # Accessing position
                    p = transformed_pose.pose.position

                    # Accessing orientation (Quaternions: x, y, z, w)
                    q = transformed_pose.pose.orientation

                    self.get_logger().info(
                        f'Pos: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) | '
                        f'Ori: (x={q.x:.2f}, y={q.y:.2f}, z={q.z:.2f}, w={q.w:.2f})')

                    fusion_msg = PoseWithCovarianceStamped()
                    fusion_msg.header = transformed_pose.header
                    fusion_msg.pose.pose = transformed_pose.pose

                    cov = [0.0] * 36
                    pos_var = 0.0001 
                    rot_var = 0.01
                    cov[0]  = pos_var # X
                    cov[7]  = pos_var # Y
                    cov[14] = pos_var # Z
                    cov[21] = rot_var # Roll
                    cov[28] = rot_var # Pitch
                    cov[35] = rot_var # Yaw
                    fusion_msg.pose.covariance = cov
                    # Publish to the topic
                    self.pose_pub.publish(fusion_msg)
                    self.get_logger().info(f'Published pose to /object/pose_raw')
                    self.snap_requested = False 
                    self.status_pub.publish(String(data="SUCCESS"))
                    return
                
                except Exception as e:
                    self.get_logger().error(f"Transformation failed: {e}")
                    continue

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