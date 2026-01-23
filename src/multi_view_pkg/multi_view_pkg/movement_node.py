import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String  # 1. Added for the command

class MultiViewCoordinator(Node):
    def __init__(self):
        super().__init__("movement_node")

        self.publisher_ = self.create_publisher(JointState, '/joint_command', 10)
        
        # 2. Added: Publisher to tell the Vision Node to take a picture
        self.cmd_pub = self.create_publisher(String, '/fusion/command', 10)

        self.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]

        self.viewpoints = [
            [0.000, -0.048, 0.000, -1.571, -0.003, 2.122, 0.767], 
            [0.266, 0.124, -0.235, -2.228, 0.266, 3.100, 0.611],
            [2.647, -0.943, -2.897, -1.141, 0.767, 2.244, -0.830],
            [0.486, 1.058, -0.016, -0.686, -0.830, 1.817, 2.333],
            [0.000, 0.000, 0.000, -1.571, 0.078, 1.939, 0.736],
            [-0.642, 0.105, 0.486, -2.098, 0.579, 2.672, -0.329],
            [0.486, 0.295, -0.141, -1.660, -0.548, 2.061, 1.989],
        ]

        self.current_index = 0
        self.timer = self.create_timer(10.0, self.timer_callback)
        self.get_logger().info("Multi-View Sequence Initialized...")

    def timer_callback(self):
        if self.current_index >= len(self.viewpoints):
            self.get_logger().info("Multi-View Scan Complete.")
            # 3. Added: Final command to tell the Fusion node to solve the math
            self.cmd_pub.publish(String(data="SOLVE"))
            self.timer.cancel()
            return

        # --- A simple trick for your old node ---
        # Before we move to the NEXT position, we SNAP the CURRENT position
        # where the robot has been sitting for 5 seconds.
        if self.current_index > 0:
             self.get_logger().info("Triggering SNAP for previous viewpoint...")
             self.cmd_pub.publish(String(data="SNAP"))

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