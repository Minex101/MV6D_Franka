import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_geometry_msgs #

class PoseCollector(Node):

    def __init__(self):
        super().__init__('vision_node')

        # 1. Subscriber for DOPE detections
        self.subscription = self.create_subscription(
            Detection3DArray,
            '/detections',
            self.detections_callback,
            10)

        # 2. TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. Publisher for the raw (but transformed) poses
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/object/pose_raw', 
            10)
        
        # Target frame for transformations
        self.target_frame = 'World'
        self.get_logger().info('Node started, waiting for detections...')

    def detections_callback(self, msg):
        detection_header = msg.header
        for detection in msg.detections:
            for result in detection.results:
                
                # Create the 'Stamped' version of the pose
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
                        f'Ori: (x={q.x:.2f}, y={q.y:.2f}, z={q.z:.2f}, w={q.w:.2f})'
)
                    # 1. Initialize the message
                    fusion_msg = PoseWithCovarianceStamped()
                    # 2. Copy the Header (Frame and Timestamp)
                    fusion_msg.header = transformed_pose.header
                    # 3. Copy the Pose (Position and Orientation)
                    fusion_msg.pose.pose = transformed_pose.pose
                    # 4. Fill the Covariance Matrix (The 6x6 trust table)
                    cov = [0.0] * 36
                    pos_var = 0.0001 
                    rot_var = 0.01   # Orientation is usually a bit noisier
                    cov[0]  = pos_var # X
                    cov[7]  = pos_var # Y
                    cov[14] = pos_var # Z
                    cov[21] = rot_var # Roll
                    cov[28] = rot_var # Pitch
                    cov[35] = rot_var # Yaw
                    fusion_msg.pose.covariance = cov
                    # 5. PUBLISH to the topic
                    self.pose_pub.publish(fusion_msg)

                    self.get_logger().info(f'Published pose to /object/pose_raw')
                except Exception as e:
                    self.get_logger().error(f'Transformation failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    pose_collector = PoseCollector()
    try:
        rclpy.spin(pose_collector)
    except KeyboardInterrupt:
        pass
    pose_collector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()