import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({
    "font.family": "serif",
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.grid": True,
    "grid.linestyle": "--",
    "grid.alpha": 0.3,
    "axes.labelsize": 11,
    "axes.titlesize": 12,
    "legend.fontsize": 10,
    "figure.dpi": 150,
})

GT = {"x": -0.2396, "y": 0.79796, "z": 0.368762, "w": 0.41385}

class SimpleStaticTester(Node):
    def __init__(self):
        super().__init__('static_tester')

        self.raw   = {"x": [], "y": [], "z": [], "w": []}
        self.fused = {"x": [], "y": [], "z": [], "w": []}

        self.create_subscription(PoseWithCovarianceStamped, '/object/pose_raw',   self.raw_cb,   10)
        self.create_subscription(PoseWithCovarianceStamped, '/object/pose_fused', self.fused_cb, 10)
        self.get_logger().info("Collecting... Ctrl+C to plot.")

    def _unpack(self, msg):
        o = msg.pose.pose.orientation
        return {"x": o.x, "y": o.y, "z": o.z, "w": o.w}

    def raw_cb(self, msg):
        q = self._unpack(msg)
        for k in self.raw:
            self.raw[k].append(q[k])

    def fused_cb(self, msg):
        q = self._unpack(msg)
        for k in self.fused:
            self.fused[k].append(q[k])

    def generate_report(self):
        if not self.raw["x"]:
            print("No data collected.")
            return

        components = ["x", "y", "z", "w"]
        colours = {"raw": "#E05252", "fused": "#3A86FF", "gt": "#2EC4B6"}

        fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=False)
        fig.suptitle("Quaternion Components: Raw vs. Fused vs. Ground Truth", fontsize=14, fontweight="bold")
        axes = axes.flatten()

        for i, comp in enumerate(components):
            ax = axes[i]
            raw_vals = np.array(self.raw[comp])

            # Raw scatter
            ax.scatter(np.arange(len(raw_vals)), raw_vals,
                       color=colours["raw"], alpha=0.5, s=18, zorder=3, label="Raw")

            # Fused line + markers
            if self.fused[comp]:
                fused_vals = np.array(self.fused[comp])
                ax.plot(np.arange(len(fused_vals)), fused_vals,
                        color=colours["fused"], linewidth=2, zorder=4, label="Fused")
                ax.scatter(np.arange(len(fused_vals)), fused_vals,
                           color=colours["fused"], s=40, zorder=5, marker='D')

            # Ground truth flat line
            ax.axhline(GT[comp], color=colours["gt"], linewidth=1.8,
                       linestyle="--", zorder=2, label=f"GT = {GT[comp]:.4f}")

            ax.set_title(f"Component: {comp}")
            ax.set_xlabel("Measurement index")
            ax.set_ylabel("Value")
            ax.legend(loc="upper right", framealpha=0.9)

        plt.tight_layout()
        plt.savefig("quaternion_components.pdf", bbox_inches="tight")
        print("Saved → quaternion_components.pdf")
        plt.show()


def main():
    rclpy.init()
    node = SimpleStaticTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.generate_report()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()