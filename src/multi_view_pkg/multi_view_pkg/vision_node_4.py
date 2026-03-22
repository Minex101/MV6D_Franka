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

        self.status_pub     = self.create_publisher(String,                    '/fusion/status',         10)
        self.pose_pub       = self.create_publisher(PoseWithCovarianceStamped, '/object/pose_raw',       10)
        self.pose_pub_sweep = self.create_publisher(PoseWithCovarianceStamped, '/object/pose_raw_sweep', 10)
        self.score_pub      = self.create_publisher(Float32,                   '/robot/depth_residual',  10)

        self.bridge      = CvBridge()
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(Detection3DArray, '/detections',       self.detections_callback, 10)
        self.command_sub  = self.create_subscription(String,           '/fusion/command',   self.command_callback,    10)
        self.info_sub     = self.create_subscription(CameraInfo,       '/camera_info_rect', self.info_cb,             10)
        self.depth_sub    = self.create_subscription(Image,            '/depth',            self.depth_cb,            10)

        self.target_frame         = 'World'
        self.snap_requested       = False
        self.sweep_snap_requested = False
        self.view_count           = 0
        self.latest_depth         = None
        self.intrinsic_matrix     = None

        self.patch_half           = 10
        self.max_residual         = 0.50

        self.get_logger().info('👁️ Vision Node: Depth Residual Uncertainty Active')

    def info_cb(self, msg):
        self.intrinsic_matrix = np.array(msg.k).reshape(3, 3)

    def depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def command_callback(self, msg):
        if msg.data == 'SNAP':
            self.get_logger().info('📸 SNAP requested')
            self.sweep_snap_requested = False
            self.snap_requested       = True
        elif msg.data == 'SWEEP_SNAP':
            self.get_logger().info('📸 SWEEP SNAP requested')
            self.snap_requested       = False
            self.sweep_snap_requested = True
        elif msg.data == 'CANCEL':
            self.snap_requested       = False
            self.sweep_snap_requested = False
            self.get_logger().warn('🚫 Snap cancelled')

    def get_uncertainty(self, pos):
        """
        Depth residual uncertainty with large patch median.
        21x21 window around projected pixel, median of all valid depths.
        uncertainty = |z_dope - z_depth_median| / max_residual, clamped 0-1.
        """
        if self.latest_depth is None or self.intrinsic_matrix is None:
            self.get_logger().warn('⚠️ No depth/intrinsics yet')
            return 0.5

        K  = self.intrinsic_matrix

        # Project DOPE position to pixel
        point = np.array([pos.x, pos.y, pos.z])
        pixel = K @ point
        if pixel[2] == 0:
            return 0.5

        u = int(pixel[0] / pixel[2])
        v = int(pixel[1] / pixel[2])
        h, w = self.latest_depth.shape

        if not (0 <= u < w and 0 <= v < h):
            return 0.5

        # Large patch
        hp = self.patch_half
        u_min, u_max = max(0, u - hp), min(w, u + hp + 1)
        v_min, v_max = max(0, v - hp), min(h, v + hp + 1)
        patch        = self.latest_depth[v_min:v_max, u_min:u_max]

        # only finite, positive values
        valid = patch[np.isfinite(patch) & (patch > 0)]

        if valid.size < 5:
            return 1.0

        z_depth     = float(np.median(valid))
        residual    = abs(pos.z - z_depth)
        uncertainty = float(np.clip(residual / self.max_residual, 0.0, 1.0))
        return uncertainty

    def detections_callback(self, msg):
        if not (self.snap_requested or self.sweep_snap_requested) or not msg.detections:
            return

        det    = msg.detections[0]
        res    = det.results[0]
        header = msg.header
        pos    = res.pose.pose.position

        if self.snap_requested:
            self.snap_requested = False
            uncertainty = self.get_uncertainty(pos)
            self.score_pub.publish(Float32(data=float(uncertainty)))

            self.process_and_publish(res, header, is_sweep=False, uncertainty=uncertainty)

        elif self.sweep_snap_requested:
            self.sweep_snap_requested = False
            uncertainty = self.get_uncertainty(pos)
            self.score_pub.publish(Float32(data=float(uncertainty)))
            self.process_and_publish(res, header, is_sweep=True, uncertainty=uncertainty)

    def process_and_publish(self, result, header, is_sweep, uncertainty=0.05):
        try:
            pose_stamped        = PoseStamped()
            pose_stamped.header = header
            pose_stamped.pose   = result.pose.pose

            transformed_pose = self.tf_buffer.transform(
                pose_stamped, self.target_frame,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            fusion_msg           = PoseWithCovarianceStamped()
            fusion_msg.header    = transformed_pose.header
            fusion_msg.pose.pose = transformed_pose.pose

            pos_var = max(float(uncertainty) * 0.1, 0.0001)
            rot_var = 0.01

            cov     = [0.0] * 36
            cov[0]  = pos_var  # x
            cov[7]  = pos_var  # y
            cov[14] = pos_var  # z
            cov[21] = rot_var  # roll
            cov[28] = rot_var  # pitch
            cov[35] = rot_var  # yaw

            fusion_msg.pose.covariance = cov

            if is_sweep:
                self.pose_pub_sweep.publish(fusion_msg)
            else:
                self.pose_pub.publish(fusion_msg)

            self.status_pub.publish(String(data='SUCCESS'))

            if not is_sweep:
                self.view_count += 1
                p = transformed_pose.pose.position
                o = transformed_pose.pose.orientation
                self.get_logger().info(f'✅ Pose Published | uncertainty={uncertainty:.4f}')
                self.get_logger().warn(
                    f'👁️ View {self.view_count} | '
                    f'Pos: [{p.x:.4f}, {p.y:.4f}, {p.z:.4f}] | '
                    f'Ori (XYZW): [{o.x:.4f}, {o.y:.4f}, {o.z:.4f}, {o.w:.4f}]'
                )
            else:
                self.get_logger().info(f'✅ Pose Published (sweep) | uncertainty={uncertainty:.4f}')

        except Exception as e:
            self.get_logger().error(f'❌ TF Error: {e}')
            self.status_pub.publish(String(data='INSTABILITY_DETECTED'))


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