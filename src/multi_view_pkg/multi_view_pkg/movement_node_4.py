import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseWithCovarianceStamped

import asyncio
import PyKDL
import numpy as np
from tracikpy import TracIKSolver

class MovementCoordinator(Node):
    def __init__(self):
        super().__init__('movement_coordinator')
        
        self.joint_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.cmd_pub = self.create_publisher(String, '/fusion/command', 10)

        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 10)
        self.status_sub = self.create_subscription(String, '/fusion/status', self.status_cb, 10)
        self.pose_sub_sweep = self.create_subscription(PoseWithCovarianceStamped, '/object/pose_raw_sweep', self.sweep_cb, 10)
        self.uncertainty_sub = self.create_subscription(Float32, '/robot/depth_residual', self.uncertainty_cb, 10)

        self.panda_joints = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]   

        self.urdf_path = "/home/affan/Documents/FY_Project/isaac_sim_assets/franka.urdf"
        self.base_link = "World" 
        self.ee_link = "franka_panda_hand" 
        self.InitializeTRACIK()

        self.object_found = False
        self.current_joints = None
        self.found_pose = None
        self.current_uncertainty = 1.0  
        self.views_completed = 0
        self.threshold_met = False
        self.waiting_for_vision = False

    def uncertainty_cb(self, msg):
        self.current_uncertainty = msg.data

    def joint_states_cb(self, msg):
        try:
            positions = []
            for name in self.panda_joints:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
            self.current_joints = np.array(positions)
        except (ValueError, IndexError):
            pass

    def status_cb(self, msg):
        if msg.data == "SUCCESS":
            self.get_logger().info("✅ Vision Success")
            self.object_found = True
            self.waiting_for_vision = False
        elif msg.data == "INSTABILITY_DETECTED":
            self.get_logger().warn("⚠️ Vision reported instability/clutter")
            self.object_found = False
            self.waiting_for_vision = False

    def sweep_cb(self, msg):
        self.found_pose = msg.pose.pose
        self.object_found = True

    def InitializeTRACIK(self):
        self.ik_solver = TracIKSolver(self.urdf_path, self.base_link, self.ee_link, timeout=0.20, epsilon=1e-5, solve_type="Distance")
        self.get_logger().info(f"TRAC-IK initialized")

    def InverseKinematics(self, offset):
        if self.found_pose is None:
            return None
        seed_state = self.current_joints if self.current_joints is not None else np.zeros(7)
        pos = self.found_pose.position
        final_rot = PyKDL.Rotation.EulerZYX(offset[0], offset[1], offset[2])
        if len(offset) > 6 and offset[6] != 0:
            final_rot = final_rot * PyKDL.Rotation.RotX(offset[6])
        ee_pose = np.eye(4)
        for r in range(3):
            for c in range(3):
                ee_pose[r, c] = final_rot[r, c]
        ee_pose[0, 3], ee_pose[1, 3], ee_pose[2, 3] = float(pos.x) + offset[3], float(pos.y) + offset[4], float(pos.z) + offset[5]
        try:
            qout = self.ik_solver.ik(ee_pose, qinit=seed_state)
            return qout.tolist() if qout is not None else None
        except:
            return None

    async def run_scan_pattern(self):
        found = await self.search_for_object()

        view_offset = {
            "back-high":    [np.pi,   -np.pi/4,  0, -0.25, 0.0,   0.35, 0.0],
            "left-high":    [np.pi/2, -np.pi/4,  0, 0.0,   0.25,  0.35, 0.0],
            "left"  : [1.5708, 0, 0, 0.0, 0.45, 0.10, 0.65],
            "front-high":   [0.0,     -np.pi/4,  0, 0.25,  0.0,   0.45, 0.0],
            "top"   : [0, -1.5708, 0, 0.0, 0.0, 0.45, 0],
            "right-high":   [-np.pi/2,-np.pi/4,  0, 0.0,  -0.25,  0.35, 0.0],
            "right" : [-1.5708, 0, 0, 0.0, -0.45, 0.10, 0.65],
        }

        safe_overhead = [0.0, -0.696, 0.0, -1.5, 0.0, 1.5, 0.7]

        if found:
            for view, offset in view_offset.items():
                self.get_logger().info(f"📐 Moving to: {view}")
                pose = self.InverseKinematics(offset)

                if pose:
                    await self.move_to_viewpoint(pose)
                    
                    self.waiting_for_vision = True
                    await asyncio.sleep(1.0)
                    self.cmd_pub.publish(String(data="SNAP"))
                    
                    # Wait for Vision Node to return SUCCESS or INSTABILITY
                    start_wait = self.get_clock().now()
                    while self.waiting_for_vision:
                        await asyncio.sleep(0.1)
                        # Safety timeout for 5 seconds
                        if (self.get_clock().now() - start_wait).nanoseconds > 5e9:
                            self.get_logger().error("⌛ Vision timeout!")
                            break

                    self.views_completed += 1

                    if self.views_completed >= 1 and self.current_uncertainty <= 0.02:
                        self.get_logger().info(f"🎯 Threshold reached: {self.current_uncertainty:.4f}")
                        self.threshold_met = True
                        break
                    
                    await self.move_to_viewpoint(safe_overhead)

            if not self.threshold_met:
                self.get_logger().info("🏁 Multi-view finished. Executing with available data.")
            
            self.cmd_pub.publish(String(data="SOLVE"))

    async def search_for_object(self):
        self.get_logger().info("🔍 Starting Sweep")
        search_viewpoints = [
            [1.551, -0.696, -0.203, -1.076, 0.016, 0.802, 0.579],
            [-0.016, -0.696, -0.203, -1.076, 0.016, 0.802, 0.579],
            [-1.519, -0.696, -0.203, -1.076, 0.016, 0.802, 0.579],
        ]
        for pose in search_viewpoints:
            await self.move_to_viewpoint(pose)
            self.cmd_pub.publish(String(data="SWEEP_SNAP"))
            await asyncio.sleep(2.0)
            if self.object_found:
                self.get_logger().info(f"📍 Target Found")
                break
        return self.object_found

    async def move_to_viewpoint(self, pose):
        target = np.array(pose)
        msg = JointState(name=self.panda_joints, position=pose)
        arrival_threshold = 0.015 
        arrived = False
        while not arrived and rclpy.ok():
            self.joint_pub.publish(msg)
            if self.current_joints is not None:
                if np.max(np.abs(self.current_joints - target)) < arrival_threshold:
                    arrived = True
                else:
                    await asyncio.sleep(0.1)
            else:
                await asyncio.sleep(0.5)
        await asyncio.sleep(0.5)

async def run_node(node):
    movement_task = asyncio.create_task(node.run_scan_pattern())
    while rclpy.ok() and not movement_task.done():
        rclpy.spin_once(node, timeout_sec=0)
        await asyncio.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = MovementCoordinator()
    try:
        asyncio.run(run_node(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()