import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import matplotlib.pyplot as plt

# -----------------------------
# Unified IEEE / Dissertation Style
# -----------------------------
plt.rcParams.update({
    "axes.labelsize": 9,
    "axes.titlesize": 10,
    "legend.fontsize": 7,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "axes.spines.top": False,
    "axes.spines.right": False,
})

COLOURS = {
    "raw": "#D55E00",     
    "fused": "#0072B2",   
    "gt": "#009E73"       
}

class PrecisionAccuracyTester(Node):
    def __init__(self):
        super().__init__('position_tester')

        # Ground Truth from Isaac Sim
        self.gt_t = np.array([0.56047, -0.01202, 0.07074]) 

        self.create_subscription(PoseWithCovarianceStamped, '/object/pose_raw', self.pose_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/object/pose_fused', self.fused_pose_callback, 10)

        self.raw_X, self.raw_Y, self.raw_Z = [], [], []
        self.raw_trans_error = []
        self.fused_pose = None
        self.fused_trans_error = None

        plt.ion()
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(8, 10))
        self.fig.canvas.manager.set_window_title('Translation Accuracy Report')
        self.fig.subplots_adjust(hspace=0.6)

    def pose_callback(self, msg):
        px, py, pz = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        self.raw_X.append(px)
        self.raw_Y.append(py)
        self.raw_Z.append(pz)
        err = np.linalg.norm(np.array([px, py, pz]) - self.gt_t)
        self.raw_trans_error.append(err)

    def fused_pose_callback(self, msg):
        px, py, pz = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        self.fused_pose = np.array([px, py, pz])
        self.fused_trans_error = np.linalg.norm(self.fused_pose - self.gt_t)

    def update_plot(self):
        if not plt.fignum_exists(self.fig.number):
            return

        for ax in [self.ax1, self.ax2, self.ax3]:
            ax.clear()
            ax.grid(True, linestyle='--', alpha=0.3)

        m_x = np.mean(self.raw_X) if self.raw_X else 0
        m_y = np.mean(self.raw_Y) if self.raw_Y else 0
        m_z = np.mean(self.raw_Z) if self.raw_Z else 0
        m_err = np.mean(self.raw_trans_error) if self.raw_trans_error else 0

        if self.raw_X:
            self.ax1.scatter(self.raw_X, self.raw_Y, color=COLOURS["raw"], alpha=0.7, s=25, 
                            edgecolors="black", linewidths=0.6, label=f"Raw ($\mu_x$: {m_x:.3f}, $\mu_y$: {m_y:.3f})")
        
        if self.fused_pose is not None:
            self.ax1.scatter(self.fused_pose[0], self.fused_pose[1], color=COLOURS["fused"], 
                            marker='X', s=60, edgecolors="black", label=f"Fused: ({self.fused_pose[0]:.3f}m, {self.fused_pose[1]:.3f}m)")
        
        self.ax1.scatter(self.gt_t[0], self.gt_t[1], color=COLOURS["gt"], 
                        marker='*', s=60, edgecolors="black", label=f"GT: ({self.gt_t[0]:.3f}m, {self.gt_t[1]:.3f}m)")
        
        self.ax1.set_title("Lateral ($X$-$Y$ Plane) Convergence")
        self.ax1.set_xlabel("X (m)")
        self.ax1.set_ylabel("Y (m)")
        self.ax1.legend(loc='lower left')

        if self.raw_Z:
            self.ax2.plot(self.raw_Z, color=COLOURS["raw"], marker='o', markersize=4, alpha=0.4, linestyle='-', markeredgecolor='black', label=f"Raw Z ($\mu$: {m_z:.3f})")
        
        if self.fused_pose is not None:
            self.ax2.axhline(self.fused_pose[2], color=COLOURS["fused"], linewidth=2, label=f"Fused Z: {self.fused_pose[2]:.3f}m")
        
        self.ax2.axhline(self.gt_t[2], color=COLOURS["gt"], linestyle='--', linewidth=1.5, label=f"GT Z: {self.gt_t[2]:.3f}m")
        
        self.ax2.set_title("Longitudinal ($Z$ Axis / Depth) Stability")
        self.ax2.set_ylabel("Z (m)")
        self.ax2.set_xlabel("Observation Index")
        self.ax2.legend(loc='upper right')

        if self.raw_trans_error:
            self.ax3.plot(self.raw_trans_error, color=COLOURS["raw"], marker='o', markersize=4, 
                         linestyle='-', linewidth=1, markeredgecolor='black', alpha=0.8,
                         label=f"Raw Error ($\mu$: {m_err*1000:.1f})")
        
        if self.fused_trans_error is not None:
            self.ax3.axhline(self.fused_trans_error, color=COLOURS["fused"], 
                            linewidth=2, label=f"Fused Error: {self.fused_trans_error*1000:.1f}mm")
        
        self.ax3.axhline(0, color=COLOURS["gt"], linestyle='-.', linewidth=1.5, label="Ideal (0mm)")
        
        self.ax3.set_title("Total Translation Error (Euclidean Distance)")
        self.ax3.set_xlabel("Observation Index")
        self.ax3.set_ylabel("Error (m)")
        self.ax3.legend(loc='upper right')

        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    tester = PrecisionAccuracyTester()
    try:
        while rclpy.ok():
            rclpy.spin_once(tester, timeout_sec=0.01)
            tester.update_plot()
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()