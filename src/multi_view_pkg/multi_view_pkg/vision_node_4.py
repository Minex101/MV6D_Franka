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

        self.get_logger().info('Vision Node Ready')

    def info_cb(self, msg):
        self.intrinsic_matrix = np.array(msg.k).reshape(3, 3)

    def depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def command_callback(self, msg):
        if msg.data == 'SNAP':
            self.sweep_snap_requested = False
            self.snap_requested       = True
            self.get_logger().info('SNAP requested')

        elif msg.data == 'SWEEP_SNAP':
            self.snap_requested       = False
            self.sweep_snap_requested = True
            self.get_logger().info('SWEEP SNAP requested')

        elif msg.data == 'CANCEL':
            self.snap_requested       = False
            self.sweep_snap_requested = False
            self.get_logger().warn('Snap cancelled')

    def get_uncertainty(self, pos):
        if self.latest_depth is None or self.intrinsic_matrix is None:
            self.get_logger().warn('No depth/intrinsics yet')
            return 0.5

        K = self.intrinsic_matrix

        # Project 3D point to pixel coordinates
        point = np.array([pos.x, pos.y, pos.z])
        pixel = K @ point
        if pixel[2] == 0:
            return 0.5

        # Convert to pixel coordinates by removing the depth scaling
        u = int(pixel[0] / pixel[2])
        v = int(pixel[1] / pixel[2])
        h, w = self.latest_depth.shape

        # Check if pixel is within image bounds
        if not (0 <= u < w and 0 <= v < h):
            return 0.5

        # Extract a patch around the pixel and compute median depth
        hp = self.patch_half
        u_min, u_max = max(0, u - hp), min(w, u + hp + 1)
        v_min, v_max = max(0, v - hp), min(h, v + hp + 1)
        patch = self.latest_depth[v_min:v_max, u_min:u_max]

        # Filter out invalid depth values (zero or NaN)
        valid = patch[np.isfinite(patch) & (patch > 0)]

        # If too few valid pixels, return high uncertainty
        if valid.size < 5:
            return 1.0

        z_depth     = float(np.median(valid))
        self.get_logger().info(f"Depth at pixel ({u}, {v}): {z_depth:.4f} m | Point Z: {pos.z:.4f} m")
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
            self.process_and_publish(res, header, is_sweep=True, uncertainty=uncertainty)

    def process_and_publish(self, result, header, is_sweep, uncertainty=0.05):
        try:
            pose_stamped        = PoseStamped()
            pose_stamped.header = header
            pose_stamped.pose   = result.pose.pose

            transformed_pose = self.tf_buffer.transform(
                pose_stamped,
                self.target_frame,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            fusion_msg           = PoseWithCovarianceStamped()
            fusion_msg.header    = transformed_pose.header
            fusion_msg.pose.pose = transformed_pose.pose

            cov = [0.0] * 36
            var = uncertainty ** 2

            cov[0]  = var
            cov[7]  = var
            cov[14] = var
            cov[21] = var
            cov[28] = var
            cov[35] = var

            fusion_msg.pose.covariance = cov

            p = transformed_pose.pose.position
            o = transformed_pose.pose.orientation

            if is_sweep:
                self.pose_pub_sweep.publish(fusion_msg)
                self.get_logger().info(
                    f'Pose Published (Sweep) | '
                    f'Pos: [{p.x:.4f}, {p.y:.4f}, {p.z:.4f}] | '
                    f'Ori (XYZW): [{o.x:.4f}, {o.y:.4f}, {o.z:.4f}, {o.w:.4f}]'
                )
            else:
                self.pose_pub.publish(fusion_msg)
                self.status_pub.publish(String(data='SUCCESS'))
                self.view_count += 1

                self.get_logger().info(
                    f'Pose Published | View {self.view_count} | '
                    f'Uncertainty: {uncertainty:.4f} | '
                    f'Pos: [{p.x:.4f}, {p.y:.4f}, {p.z:.4f}] | '
                    f'Ori (XYZW): [{o.x:.4f}, {o.y:.4f}, {o.z:.4f}, {o.w:.4f}]'
                )

        except Exception as e:
            self.get_logger().error(f'TF Error: {e}')
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