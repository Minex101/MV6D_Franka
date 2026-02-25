import rclpy
from rclpy.node import Node
import rclpy.duration
from std_msgs.msg import String, Float32
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from cv_bridge import CvBridge
import numpy as np

class PoseCollector_2(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.status_pub = self.create_publisher(String, '/fusion/status', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/object/pose_raw', 10)
        self.pose_pub_sweep = self.create_publisher(PoseWithCovarianceStamped, '/object/pose_raw_sweep', 10)
        self.score_pub = self.create_publisher(Float32, '/robot/depth_residual', 10)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(Detection3DArray, '/detections', self.detections_callback, 10)
        self.command_sub = self.create_subscription(String, '/fusion/command', self.command_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera_info_rect', self.info_cb, 10)
        self.depth_sub = self.create_subscription(Image, '/depth', self.depth_cb, 10)

        self.target_frame = 'World'
        self.snap_requested = False
        self.sweep_snap_requested = False
        self.latest_depth = None
        self.intrinsic_matrix = None
        self.depth_threshold = 0.06

        self.get_logger().info('👁️ Vision Node: One-Shot Mode (5x5 Median Patch Active)')

    def info_cb(self, msg):
        self.intrinsic_matrix = np.array(msg.k).reshape(3, 3)

    def depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def command_callback(self, msg):
        if msg.data == "SNAP":
            self.get_logger().info("📸 SNAP requested")
            self.sweep_snap_requested = False
            self.snap_requested = True
        elif msg.data == "SWEEP_SNAP":
            self.get_logger().info("📸 SWEEP SNAP requested")
            self.snap_requested = False
            self.sweep_snap_requested = True

    def verify_depth(self, p):
        if self.latest_depth is None or self.intrinsic_matrix is None:
            return False, 0.0
        
        z_ai = p.z
        point_3d = np.array([p.x, p.y, p.z])
        pixel_coords = self.intrinsic_matrix @ point_3d
        
        if pixel_coords[2] == 0: return False, 0.0
        
        u_c = int(pixel_coords[0] / pixel_coords[2])
        v_c = int(pixel_coords[1] / pixel_coords[2])
        
        h, w = self.latest_depth.shape

        # Define 5x5 window boundaries
        half_win = 2 
        u_min, u_max = max(0, u_c - half_win), min(w, u_c + half_win + 1)
        v_min, v_max = max(0, v_c - half_win), min(h, v_c + half_win + 1)
        
        # Extract patch and filter invalid pixels
        depth_patch = self.latest_depth[v_min:v_max, u_min:u_max]
        valid_depths = depth_patch[~np.isnan(depth_patch) & (depth_patch > 0)]
        
        if valid_depths.size == 0:
            return False, 0.0
            
        # 5x5 Median logic
        z_sensor = np.median(valid_depths)
        
        residual = abs(z_ai - z_sensor)
        self.score_pub.publish(Float32(data=float(residual)))
        return (residual < self.depth_threshold), residual

    def detections_callback(self, msg):
        if not (self.snap_requested or self.sweep_snap_requested) or not msg.detections:
            return

        det = msg.detections[0]
        res = det.results[0]
        header = msg.header
        pos = res.pose.pose.position

        if self.snap_requested:
            self.snap_requested = False
            is_valid, residual = self.verify_depth(pos)
            
            if not is_valid:
                self.get_logger().warn(f"🚫 View Rejected: {residual:.4f}m")
                self.status_pub.publish(String(data="INSTABILITY_DETECTED"))
                return
            
            self.process_and_publish(res, header, False, residual)

        elif self.sweep_snap_requested:
            self.sweep_snap_requested = False
            self.process_and_publish(res, header, True)

    def process_and_publish(self, result, header, is_sweep, residual=0.05):
        try:
            pose_stamped = PoseStamped()
            pose_stamped.header = header
            pose_stamped.pose = result.pose.pose

            transformed_pose = self.tf_buffer.transform(
                pose_stamped, self.target_frame, 
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            fusion_msg = PoseWithCovarianceStamped()
            fusion_msg.header = transformed_pose.header
            fusion_msg.pose.pose = transformed_pose.pose
            pos_var = max(float(residual)**2, 0.0001)
    
            rot_var = 0.01

            cov = [0.0] * 36
            cov[0]  = pos_var # x
            cov[7]  = pos_var # y
            cov[14] = pos_var # z
            cov[21] = rot_var # roll
            cov[28] = rot_var # pitch
            cov[35] = rot_var # yaw
            
            fusion_msg.pose.covariance = cov

            if is_sweep:
                self.pose_pub_sweep.publish(fusion_msg)
            else:
                self.pose_pub.publish(fusion_msg)

            self.status_pub.publish(String(data="SUCCESS"))
            self.get_logger().info(f'✅ Pose Published')
        except Exception as e:
            self.get_logger().error(f"❌ TF Error: {e}")
            self.status_pub.publish(String(data="INSTABILITY_DETECTED"))

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