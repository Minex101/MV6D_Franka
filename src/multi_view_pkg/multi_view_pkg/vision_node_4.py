"""
ROS 2 Node for Object Detection and Coordinate Transformation.
This node utilizes the Deep Object Pose Estimation (DOPE) framework.

Reference:
Tremblay, J., To, T., Sundaralingam, B., Xiang, Y., Fox, D., & Birchfield, S. (2018).
Deep Object Pose Estimation for Semantic Robotic Grasping of Household Objects.
In Conference on Robot Learning (CoRL). arXiv:1809.10790.
"""

# ---

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

            # -- Publishers --
        self.status_pub = self.create_publisher(String,
                                                '/fusion/status',
                                                10
        )
        
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                              '/object/pose_raw',
                                              10
        )
        
        self.pose_pub_sweep = self.create_publisher(PoseWithCovarianceStamped,
                                                    '/object/pose_raw_sweep', 
                                                    10
        )

        self.score_pub = self.create_publisher(Float32,
                                              '/robot/depth_residual',  
                                              10
        )

        # -- Subscribers --
        self.command_sub = self.create_subscription(String,           
                                                   '/fusion/command',
                                                   self.command_callback,
                                                   10
        )

        self.info_sub = self.create_subscription(CameraInfo,
                                                 '/camera_info_rect', 
                                                 self.info_cb,
                                                 10
        )
        
        self.depth_sub = self.create_subscription(Image, 
                                                  '/depth', 
                                                  self.depth_cb,
                                                  10
        )

        # -- TF Setup --
        self.bridge      = CvBridge()
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -- State Variables --
        self.target_frame         = 'World'
        self.snap_requested       = False
        self.sweep_snap_requested = False
        self.view_count           = 0
        self.latest_depth         = None
        self.intrinsic_matrix     = None
        self.patch_half_w = 35
        self.patch_half_h = 70
        self.max_residual = 1.0

        self.get_logger().info('Vision Node Ready')

    def info_cb(self, msg):

        """
        Store the camera intrinsic matrix from the CameraInfo topic.
        """

        self.intrinsic_matrix = np.array(msg.k).reshape(3, 3)

    def depth_cb(self, msg):

        """
        Store the latest depth image from the /depth topic.
        """

        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def command_callback(self, msg):

        """
        Listen for commands from the fusion node to trigger snap or sweep-snap actions.
        """

        if msg.data == 'SNAP':
            self.sweep_snap_requested = False
            self.snap_requested       = True
        elif msg.data == 'SWEEP_SNAP':
            self.snap_requested       = False
            self.sweep_snap_requested = True
        elif msg.data == 'CANCEL':
            self.snap_requested       = False
            self.sweep_snap_requested = False

    def get_uncertainty(self, pos):

        """
        Calculate the uncertainty based on the depth residual at the given position.
        """

        if self.latest_depth is None or self.intrinsic_matrix is None:
            return 0.5

        K = self.intrinsic_matrix
        point = np.array([pos.x, pos.y, pos.z])
        pixel = K @ point
        if pixel[2] == 0:
            return 0.5

        u = int(pixel[0] / pixel[2])
        v = int(pixel[1] / pixel[2])

        h, w = self.latest_depth.shape

        if not (0 <= u < w and 0 <= v < h):
            return 0.5

        half_w = self.patch_half_w
        half_h = self.patch_half_h

        u_min, u_max = max(0, u - half_w), min(w, u + half_w + 1)
        v_min, v_max = max(0, v - half_h), min(h, v + half_h + 1)

        patch = self.latest_depth[v_min:v_max, u_min:u_max]

        valid = patch[np.isfinite(patch) & (patch > 0)]

        if valid.size < 5:
            return 1.0

        z_depth = float(np.median(valid))
        residual = abs(pos.z - z_depth)
        uncertainty = float(np.clip(residual / self.max_residual, 0.0, 1.0))

        return uncertainty

    def detections_callback(self, msg):

        """
        Process incoming detections from the vision system. Depending on the current command state, either trigger a snap or sweep-snap action, calculate uncertainty, 
        and publish the results.
        """

        if not (self.snap_requested or self.sweep_snap_requested) or not msg.detections:
            return

        det = msg.detections[0]
        res = det.results[0]
        header = msg.header
        pos = res.pose.pose.position

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

        """
        Take the raw pose result from the detection, transform it to the target frame, and publish it along with the associated uncertainty.
        """

        try:
            pose_stamped = PoseStamped()
            pose_stamped.header = header
            pose_stamped.pose = result.pose.pose

            transformed_pose = self.tf_buffer.transform(
                pose_stamped,
                self.target_frame,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            fusion_msg = PoseWithCovarianceStamped()
            fusion_msg.header = transformed_pose.header
            fusion_msg.pose.pose = transformed_pose.pose

            cov = [0.0] * 36
            var = uncertainty ** 2

            cov[0] = var
            cov[7] = var
            cov[14] = var
            cov[21] = var
            cov[28] = var
            cov[35] = var

            fusion_msg.pose.covariance = cov

            if is_sweep:
                self.pose_pub_sweep.publish(fusion_msg)
            else:
                self.pose_pub.publish(fusion_msg)
                self.status_pub.publish(String(data='SUCCESS'))
                self.view_count += 1

        except Exception:
            self.status_pub.publish(String(data='INSTABILITY_DETECTED'))

# --- Main Execution ---

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