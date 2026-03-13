import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import matplotlib.pyplot as plt

class PrecisionAccuracyTester(Node):
    def __init__(self):
        super().__init__('precision_accuracy_tester')

        self.gt_t = np.array([0.36919, 0.30273, 0.03045])

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/object/pose_raw',
            self.pose_callback,
            10
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/object/pose_fused',
            self.fused_pose_callback,
            10
        )

        # Raw storage
        self.raw_X, self.raw_Y, self.raw_Z = [], [], []
        self.raw_trans_error = []

        # Single fused pose storage
        self.fused_pose = None
        self.fused_trans_error = None

        plt.ion()
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(10, 14))
        self.timer = self.create_timer(0.1, self.plot_results)
        self.fig.subplots_adjust(
            hspace=0.5   # vertical space between subplots
        )

    def pose_callback(self, msg):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z

        self.raw_X.append(px)
        self.raw_Y.append(py)
        self.raw_Z.append(pz)

        pred_pos = np.array([px, py, pz])
        trans_error = np.linalg.norm(pred_pos - self.gt_t)
        self.raw_trans_error.append(trans_error)

    def fused_pose_callback(self, msg):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z

        self.fused_pose = np.array([px, py, pz])
        self.fused_trans_error = np.linalg.norm(self.fused_pose - self.gt_t)

        self.get_logger().info(f"Fused translation error: {self.fused_trans_error:.6f} m")

    def plot_results(self):
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()

        # --- X-Y scatter ---
        if len(self.raw_X) > 0:
            self.ax1.scatter(self.raw_X, self.raw_Y,
                 c="#D55E00", alpha=0.7,
                 edgecolors="black", linewidths=0.8,
                 marker='o', label="Raw (X-Y)")

        if self.fused_pose is not None:
            self.ax1.scatter(self.fused_pose[0], self.fused_pose[1],
                 c="#0072B2", alpha=0.9,
                 edgecolors="black", linewidths=1.0,
                 marker='X', s=100, label="Fused (X-Y)")

        self.ax1.scatter(self.gt_t[0], self.gt_t[1],
                 c="#009E73", alpha=1.0,
                 edgecolors="black", linewidths=1.0,
                 marker='X', s=120, label="Ground Truth (X-Y)")

        self.ax1.set_title("X-Y Position")
        self.ax1.set_xlabel("X (meters)")
        self.ax1.set_ylabel("Y (meters)")
        self.ax1.legend()
        self.ax1.grid(True)

        # --- Z plot ---
        if len(self.raw_Z) > 0:
            self.ax2.plot(self.raw_Z,
              color="#D55E00", linestyle='--',
              alpha=0.7, linewidth=1.5, label="Raw Z")

        if self.fused_pose is not None:
            self.ax2.axhline(self.fused_pose[2],
                 color="#0072B2", linestyle='-',
                 linewidth=2, label="Fused Z")

        
        self.ax2.axhline(self.gt_t[2],
                        color="#009E73", linestyle='-',
                        linewidth=2, label="GT Z")

        self.ax2.set_title("Depth (Z) Accuracy Plot")
        self.ax2.set_xlabel("")
        self.ax2.set_ylabel("Z (meters)")
        self.ax2.legend()
        self.ax2.grid(True)

        # --- Translation Error ---
        if len(self.raw_trans_error) > 0:
            self.ax3.plot(
            self.raw_trans_error,
            color="#D55E00",        # muted crimson for raw
            linestyle='--',
            alpha=0.7,
            linewidth=1.5,
            marker='o',
            markersize=5,
            markeredgecolor='black',  # black edge for markers
            markerfacecolor="#D55E00",
            label="Raw Euclidean Error"
        )

        if self.fused_trans_error is not None:
            if self.fused_trans_error is not None:
                self.ax3.axhline(
                    self.fused_trans_error,
                    color="#0072B2",       # dark navy for fused
                    linestyle='-',
                    linewidth=2,
                    label="Fused Euclidean Error"
                )
        
        self.ax3.axhline(
            0,
            color="#009E73",           # dark green
            linestyle='-.',
            linewidth=1.5,
            label="Zero Error Reference"
        )

        self.ax3.set_title("Euclidean Translation Error Over Frames")
        self.ax3.set_xlabel("Frame Index")
        self.ax3.set_ylabel("Error (meters)")
        self.ax3.legend()
        self.ax3.grid(True)

        plt.draw()
        plt.pause(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionAccuracyTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()