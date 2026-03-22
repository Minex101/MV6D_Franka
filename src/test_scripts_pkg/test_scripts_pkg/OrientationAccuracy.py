import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib.pyplot as plt
import numpy as np
import sys

plt.rcParams.update({
    "axes.labelsize": 9,
    "axes.titlesize": 10,
    "legend.fontsize": 7,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
})

GT = {"x": 0.70710678, "y": 0.0, "z": 0.70710678, "w": 0.0} # Set the known ground truth orientation here (x, y, z, w) format

COLOURS = {
    "raw": "#D55E00",     
    "fused": "#0072B2",   
    "gt": "#009E73"       
}

class OrientationTester(Node):

    def __init__(self):
        super().__init__('orientation_tester')

        self.raw = {"x": [], "y": [], "z": [], "w": []}
        self.fused = None
        self.q_ref = None

        self.create_subscription(PoseWithCovarianceStamped, '/object/pose_raw', self.raw_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/object/pose_fused', self.fused_cb, 10)

        self.get_logger().info("Collecting data, press Ctrl+C to stop and generate report...")

    def _unpack(self, msg):
        o = msg.pose.pose.orientation
        return {"x": o.x, "y": o.y, "z": o.z, "w": o.w}

    def raw_cb(self, msg):
        q = self._unpack(msg)
        if self.q_ref is None:
            # first raw quaternion as reference
            q_arr = np.array([q['w'], q['x'], q['y'], q['z']])
            self.q_ref = q_arr / np.linalg.norm(q_arr)
        for k in self.raw:
            self.raw[k].append(q[k])

    def fused_cb(self, msg):
        q = self._unpack(msg)
        q_arr = np.array([q['w'], q['x'], q['y'], q['z']])
        q_arr /= np.linalg.norm(q_arr)
        if self.q_ref is not None:
            if np.dot(q_arr, self.q_ref) < 0:
                q_arr = -q_arr
        self.fused = {"w": q_arr[0], "x": q_arr[1], "y": q_arr[2], "z": q_arr[3]}

    def quat_error(self, q):
        q_gt = np.array([GT['w'], GT['x'], GT['y'], GT['z']])
        q_meas = np.array([q['w'], q['x'], q['y'], q['z']])
        q_gt /= np.linalg.norm(q_gt)
        q_meas /= np.linalg.norm(q_meas)
        dot = np.clip(np.abs(np.dot(q_gt, q_meas)), -1.0, 1.0)
        return 2 * np.arccos(dot) * 180 / np.pi

    def generate_report(self):
        if not self.raw["x"]:
            self.get_logger().warn("No data collected. Plotting cancelled.")
            return

        components = ["x", "y", "z", "w"]
        titles = {"x": r"$q_x$", "y": r"$q_y$", "z": r"$q_z$", "w": r"$q_w$"}

        fig = plt.figure(figsize=(12, 8))
        gs = fig.add_gridspec(2, 3, height_ratios=[1, 1], hspace=0.4, wspace=0.3)

        axes = [fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1]), 
                fig.add_subplot(gs[0, 2]), fig.add_subplot(gs[1, 0])]
        ax_err = fig.add_subplot(gs[1, 1:])

        for i, comp in enumerate(components):
            ax = axes[i]
            y_vals = np.array(self.raw[comp])
            x_vals = np.arange(len(y_vals))
            r_mean, r_std = np.mean(y_vals), np.std(y_vals)
            
            ax.scatter(x_vals, y_vals, color=COLOURS["raw"], alpha=0.7, s=25, 
                       edgecolors="black", linewidths=0.6, marker='o', 
                       label=f"Raw ($\mu$: {r_mean:.3f})")

            if self.fused:
                ax.axhline(self.fused[comp], color=COLOURS["fused"], linestyle='-', 
                           linewidth=2, label=f"Fused: {self.fused[comp]:.3f}")

            ax.axhline(GT[comp], color=COLOURS["gt"], linestyle='--', 
                       linewidth=1.5, label=f"GT: {GT[comp]:.3f}")

            ax.set_title(f"Orientation Component {titles[comp]}")
            ax.set_xlabel("Observation Index")
            ax.set_ylabel(f"{titles[comp]} Value")
            ax.grid(True, linestyle='--', alpha=0.3)
            ax.legend(loc="best")

        raw_errs = np.array([self.quat_error({k: self.raw[k][j] for k in components}) 
                             for j in range(len(self.raw["x"]))])
        x_err = np.arange(len(raw_errs))
        e_mean = np.mean(raw_errs)

        ax_err.plot(x_err, raw_errs, color=COLOURS["raw"], marker='o', markersize=6, 
                    linestyle='-', linewidth=1.5, markeredgecolor='black',
                    label=f"Raw Error ($\mu$: {e_mean:.2f}°)")

        if self.fused:
            f_err = self.quat_error(self.fused)
            ax_err.axhline(f_err, color=COLOURS["fused"], linestyle='-', linewidth=2, 
                           label=f"Fused Error: {f_err:.2f}°")

        ax_err.axhline(0, color=COLOURS["gt"], linestyle='-.', linewidth=1.5, label="Ideal (0°)")
        ax_err.set_title("Total Angular Error (Geodesic)")
        ax_err.set_ylabel("Error [Degrees]")
        ax_err.set_xlabel("Observation Index")
        ax_err.legend(loc='upper right')
        ax_err.grid(True, linestyle='--', alpha=0.3)

        plt.tight_layout()
        plt.savefig("orientation_report.pdf", bbox_inches="tight")
        print("\n[✔] Report saved to orientation_report.pdf")
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = OrientationTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[!] Shutdown signal received. Generating plots...")
        node.generate_report()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()