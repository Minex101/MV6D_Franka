from omni.isaac.core.utils.stage import get_current_stage
from pxr import UsdGeom
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GTPublisher(Node):
    def __init__(self):
        super().__init__('gt_publisher')
        self.pub = self.create_publisher(PoseStamped, '/object/pose_gt', 10)
        self.create_timer(0.1, self.publish_gt)

    def publish_gt(self):
        stage = get_current_stage()
        prim = stage.GetPrimAtPath('/World/your_object')  # change to your object path
        
        xform = UsdGeom.Xformable(prim)
        transform = xform.ComputeLocalToWorldTransform(0)
        
        # Extract translation
        t = transform.ExtractTranslation()
        
        # Extract rotation as quaternion
        r = transform.ExtractRotationQuat()
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = t[0]
        msg.pose.position.y = t[1]
        msg.pose.position.z = t[2]
        msg.pose.orientation.x = r.imaginary[0]
        msg.pose.orientation.y = r.imaginary[1]
        msg.pose.orientation.z = r.imaginary[2]
        msg.pose.orientation.w = r.real
        
        self.pub.publish(msg)