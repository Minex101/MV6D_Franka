import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class MultiViewCoordinator(Node):
    def __init__(self):
        super().__init__("movement_node")

        # 1. Publisher to move the robot
        self.publisher_ = self.create_publisher(JointState, '/joint_command', 10)

        self.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]

        
        # These joints are adjusted to ensure the camera 'looks back' at the center
        self.viewpoints = [
            
            [0.266, 0.124, -0.235, -2.228, 0.266, 3.100, 0.611],
            # 1. Left Side: Base moves left, Joint 5 twists right to look back at center
            [0.000, 0.000, 0.000, -1.571, 0.078, 1.939, 0.736],
            [-0.642, 0.105, 0.486, -2.098, 0.579, 2.672, -0.329],
            
            # 3. Right Side: Base moves right, Joint 5 twists left to look back at center
            [1.237, 0.867, -0.893, -2.098, 1.206, 3.406, 0.830],
            
            # 4. High-Offset: Looking down from a slight angle
        ]

        self.current_index = 0

        # We move every 5 seconds to give the Vision + Kalman Filter 
        # enough time to "settle" and merge the data.
        self.timer = self.create_timer(5.0, self.timer_callback)
        
        self.get_logger().info("Multi-View Sequence Initialized...")

    def timer_callback(self):
        if self.current_index >= len(self.viewpoints):
            self.get_logger().info("Multi-View Scan Complete. Filter should be stable now.")
            self.timer.cancel()
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.viewpoints[self.current_index]

        self.publisher_.publish(msg)
        self.get_logger().info(f"Moving to Viewpoint {self.current_index + 1}/4")
        
        self.current_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = MultiViewCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()