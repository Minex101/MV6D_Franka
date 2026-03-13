import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseWithCovarianceStamped
import asyncio
import PyKDL
import numpy as np
import time
from tracikpy import TracIKSolver

class MovementCoordinator(Node):
    def __init__(self):
        super().__init__('movement_coordinator')
        
        # Publishers & Subscribers
        self.joint_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.cmd_pub = self.create_publisher(String, '/fusion/command', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 10)
        self.status_sub = self.create_subscription(String, '/fusion/status', self.status_cb, 10)
        self.pose_sub_sweep = self.create_subscription(PoseWithCovarianceStamped, '/object/pose_raw_sweep', self.sweep_cb, 10)
        self.uncertainty_sub = self.create_subscription(Float32, '/robot/depth_residual', self.uncertainty_cb, 10)

        # Robot Config
        self.panda_joints = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]   
        self.urdf_path = "/home/affan/Documents/FY_Project/isaac_sim_assets/franka.urdf"
        self.base_link = "World" 
        self.ee_link = "franka_panda_hand" 
        
        self.ik_solver = TracIKSolver(self.urdf_path, self.base_link, self.ee_link, timeout=0.20, epsilon=1e-5, solve_type="Distance")
        
        # State Variables
        self.object_found = False
        self.current_joints = None
        self.found_pose = None
        self.current_uncertainty = 1.0  
        self.waiting_for_vision = False

    def uncertainty_cb(self, msg):
        self.current_uncertainty = msg.data

    def joint_states_cb(self, msg):
        try:
            positions = [msg.position[msg.name.index(name)] for name in self.panda_joints]
            self.current_joints = np.array(positions)
        except (ValueError, IndexError):
            pass

    def status_cb(self, msg):
        if "SUCCESS" in msg.data:
            self.waiting_for_vision = False

    def sweep_cb(self, msg):
        self.found_pose = msg.pose.pose
        self.object_found = True

    def calculate_ik_quaternion(self, dx, dy, dz, pitch_rad, divisor=0):
        if not self.found_pose: return None
        
        tx, ty, tz = self.found_pose.position.x, self.found_pose.position.y, self.found_pose.position.z
        cx, cy, cz = tx + dx, ty + dy, tz + dz
        

        base_rot = PyKDL.Rotation.RotY(np.pi/divisor) 
        tilt = PyKDL.Rotation.RotZ(pitch_rad)
        
        final_rot = base_rot * tilt * PyKDL.Rotation.RotZ(np.pi) 

        ee_pose = np.eye(4)
        for r in range(3):
            for c in range(3):
                ee_pose[r, c] = final_rot[r, c]
        ee_pose[:3, 3] = [cx, cy, cz]
        
        seed = self.current_joints if self.current_joints is not None else np.zeros(7)
        qout = self.ik_solver.ik(ee_pose, qinit=seed)
        return qout.tolist() if qout is not None else None

    async def run_scan_pattern(self):
        self.get_logger().info("🔍 Starting Search...")
        found = await self.search_for_object()
        
        if found:
            # We only define X, Y, Z offsets now. 
            # The rotation is handled globally in calculate_ik_quaternion
            view_offsets = {
                "FRONT":  [-0.30, 0.0, 0.40, 0, 3],
                "TOP":    [0.0,  0.0, 0.45, 0, 2],
                "LEFT":   [0.0,  0.35, 0.35, -0.735, 2],
                "RIGHT":  [0.0, -0.35, 0.35, 0.735, 2]
            }

            for name, offset in view_offsets.items():
                self.get_logger().info(f"📸 Moving to: {name}")
                joint_pose = self.calculate_ik_quaternion(offset[0], offset[1], offset[2], offset[3], offset[4])
                
                if joint_pose:
                    await self.move_to_viewpoint(joint_pose)
                    self.waiting_for_vision = True
                    self.cmd_pub.publish(String(data="SNAP"))
                    
                    v_start = time.time()
                    while self.waiting_for_vision and (time.time() - v_start) < 5.0:
                        await asyncio.sleep(0.1)
                else:
                    self.get_logger().warn(f"⚠️ IK failed for {name}")

            self.cmd_pub.publish(String(data="SOLVE"))
            self.get_logger().info("🏁 Scan Complete.")
        else:
            self.get_logger().error("❌ Object not found.")

    async def search_for_object(self):
        search_poses = [
            [1.55, -0.7, -0.2, -1.0, 0.0, 0.8, 0.5],
            [0.0, -0.7, -0.2, -1.0, 0.0, 0.8, 0.5],
            [-1.55, -0.7, -0.2, -1.0, 0.0, 0.8, 0.5]
        ]
        for pose in search_poses:
            await self.move_to_viewpoint(pose)
            self.cmd_pub.publish(String(data="SWEEP_SNAP"))
            await asyncio.sleep(2.0) 
            if self.object_found: return True
        return False

    async def move_to_viewpoint(self, pose):
        msg = JointState(name=self.panda_joints, position=pose)
        target = np.array(pose)
        arrived = False
        while not arrived and rclpy.ok():
            self.joint_pub.publish(msg)
            if self.current_joints is not None:
                if np.max(np.abs(self.current_joints - target)) < 0.02:
                    arrived = True
            await asyncio.sleep(0.1)
        await asyncio.sleep(0.5)

async def run_node(node):
    task = asyncio.create_task(node.run_scan_pattern())
    while rclpy.ok() and not task.done():
        rclpy.spin_once(node, timeout_sec=0)
        await asyncio.sleep(0.01)

def main():
    rclpy.init()
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