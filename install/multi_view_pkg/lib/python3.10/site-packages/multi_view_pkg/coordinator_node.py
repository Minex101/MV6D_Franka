import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class Coordinator(Node):
    def __init__(self):
        super().__init__("coordinator_node")

        # Matches your screenshot's "ROS2 Subscribe Joint State" node
        self.publisher_ = self.create_publisher(
            JointState, 
            '/joint_command', 
            10
        )

        # Joint names must match the robot in your Sim
        self.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]

        self.waypoints = [
            # View 1 – Front
            [-0.36, -0.568, 0.467, -2.533, 0.063, 2.676, 0.741],

            # View 2 – Front-Left
            [0.755, -0.508, -1.069, -2.574, -0.534, 3.019, 0.686],

            # View 3 – Left
            [1.369, -0.066, -1.377, -2.373, -0.358, 2.99, 0.661],

            # View 4 – Front-Right
            [1.515, -0.068, -1.488, -1.869, -0.016, 2.196, 0.726],
        ]

        self.current_index = 0

        # Move to a new viewpoint every 5 seconds
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info("Node started using JointState mode.")

    def timer_callback(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info("All viewpoints finished!")
            self.timer.cancel()
            return

        # Create the message that matches your graph node
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.waypoints[self.current_index]

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published Viewpoint {self.current_index + 1} to /joint_command")
        
        self.current_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = Coordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()