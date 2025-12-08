# isaac_teleop.py
# Real-time VR Teleoperation for Franka Panda in Isaac Sim
# Dynamic calibration based on initial hand position
# Run using: ./python.sh path/to/isaac_teleop.py

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

try:
    import rclpy
except ImportError:
    print("ERROR: rclpy not found. Don't source system ROS 2 before running Isaac Sim.")
    raise

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
import numpy as np
from scipy.spatial.transform import Rotation as R
from omni.isaac.core import World
from omni.isaac.franka import Franka
from omni.isaac.core.utils.types import ArticulationAction


# ============================================================================
# CONFIGURATION
# ============================================================================
CONFIG = {
    # Position scaling (1.0 = 1:1 mapping)
    "pos_scale": 1.0,
    
    # Robot home position (where hand center maps to)
    "robot_home": [0.5, 0.0, 0.4],
    
    # Workspace limits (robot frame)
    "workspace": {
        "x_min": 0.15, "x_max": 0.85,
        "y_min": -0.6, "y_max": 0.6,
        "z_min": 0.02, "z_max": 0.9,
    },
    
    # Smoothing (0.0 = instant, higher = smoother but laggy)
    "smoothing": 0.0,
    
    # Gripper threshold
    "gripper_threshold": 0.3,
    
    # Calibration samples (average first N poses to get stable reference)
    "calibration_samples": 30,
}


class QuestTeleop(Node):
    def __init__(self, config):
        super().__init__('isaac_quest_teleop')
        self.config = config
        
        self.pose_count = 0
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/quest/right_hand/pose', self.pose_callback, 10)
        self.input_sub = self.create_subscription(
            Joy, '/quest/right_hand/inputs', self.input_callback, 10)
        
        # Target state
        self.target_pos = np.array(config["robot_home"])
        self.target_rot = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
        self.gripper_closed = False
        
        # Dynamic calibration
        self.calibrated = False
        self.calibration_poses = []
        self.reference_pos = None  # XR position that maps to robot_home
        
        # Coordinate transform matrix (XR -> Robot)
        self.T = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
        
        self.get_logger().info("="*50)
        self.get_logger().info("QuestTeleop - Dynamic Calibration Mode")
        self.get_logger().info("Hold your hand steady for calibration...")
        self.get_logger().info("="*50)

    def pose_callback(self, msg):
        self.pose_count += 1
        
        # Raw XR position
        xr_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
        # =====================================================================
        # DYNAMIC CALIBRATION
        # =====================================================================
        if not self.calibrated:
            self.calibration_poses.append(xr_pos.copy())
            
            if len(self.calibration_poses) >= self.config["calibration_samples"]:
                # Average the calibration poses for stability
                self.reference_pos = np.mean(self.calibration_poses, axis=0)
                self.calibrated = True
                
                self.get_logger().info("="*50)
                self.get_logger().info("✓ CALIBRATION COMPLETE!")
                self.get_logger().info(f"  Reference XR pos: ({self.reference_pos[0]:.2f}, {self.reference_pos[1]:.2f}, {self.reference_pos[2]:.2f})")
                self.get_logger().info(f"  Maps to robot home: {self.config['robot_home']}")
                self.get_logger().info("  Move your hand to control the robot!")
                self.get_logger().info("="*50)
            else:
                remaining = self.config["calibration_samples"] - len(self.calibration_poses)
                if remaining % 10 == 0:
                    self.get_logger().info(f"Calibrating... {remaining} samples remaining")
            return
        
        # =====================================================================
        # RELATIVE POSITION (from calibration reference)
        # =====================================================================
        # Calculate offset from reference position
        xr_offset = xr_pos - self.reference_pos
        
        # Transform to robot frame
        robot_offset = self.T @ xr_offset
        
        # Apply scaling and add to robot home position
        robot_pos = robot_offset * self.config["pos_scale"] + np.array(self.config["robot_home"])
        
        # Clamp to workspace
        ws = self.config["workspace"]
        robot_pos[0] = np.clip(robot_pos[0], ws["x_min"], ws["x_max"])
        robot_pos[1] = np.clip(robot_pos[1], ws["y_min"], ws["y_max"])
        robot_pos[2] = np.clip(robot_pos[2], ws["z_min"], ws["z_max"])
        
        # =====================================================================
        # ORIENTATION (full tracking)
        # =====================================================================
        xr_quat = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        
        r_xr = R.from_quat(xr_quat)
        mat_xr = r_xr.as_matrix()
        mat_robot = self.T @ mat_xr @ self.T.T
        
        flip = R.from_euler('x', 180, degrees=True).as_matrix()
        mat_robot = mat_robot @ flip
        
        quat_robot = R.from_matrix(mat_robot).as_quat()
        robot_rot = np.array([quat_robot[3], quat_robot[0], quat_robot[1], quat_robot[2]])
        
        # Apply smoothing
        alpha = self.config["smoothing"]
        if alpha > 0:
            self.target_pos = alpha * self.target_pos + (1 - alpha) * robot_pos
            self.target_rot = alpha * self.target_rot + (1 - alpha) * robot_rot
            self.target_rot = self.target_rot / np.linalg.norm(self.target_rot)
        else:
            self.target_pos = robot_pos
            self.target_rot = robot_rot
        
        if self.pose_count % 500 == 0:
            self.get_logger().info(f"Pos: ({self.target_pos[0]:.2f}, {self.target_pos[1]:.2f}, {self.target_pos[2]:.2f})")

    def input_callback(self, msg):
        trigger = msg.axes[0] if len(msg.axes) > 0 else 0.0
        squeeze = msg.axes[1] if len(msg.axes) > 1 else 0.0
        self.gripper_closed = trigger > self.config["gripper_threshold"] or squeeze > self.config["gripper_threshold"]
        
    def recalibrate(self):
        """Call this to recalibrate (e.g., when user changes position)"""
        self.calibrated = False
        self.calibration_poses = []
        self.reference_pos = None
        self.get_logger().info("Recalibrating... hold hand steady")


def main():
    rclpy.init()
    teleop_node = QuestTeleop(CONFIG)
    
    # Setup World
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    
    # Add Franka
    franka = Franka(prim_path="/World/Franka", name="franka")
    world.scene.add(franka)
    world.reset()
    
    # IK Solver
    from omni.isaac.motion_generation import LulaKinematicsSolver, interface_config_loader
    
    mg_config = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
    ik_solver = LulaKinematicsSolver(
        robot_description_path=mg_config["robot_description_path"],
        urdf_path=mg_config["urdf_path"]
    )
    
    print("="*60)
    print("Isaac Sim VR Teleoperation")
    print("Dynamic calibration - works with any hand position!")
    print("="*60)
    
    ik_success = 0
    ik_fail = 0
    frame_count = 0
    last_good_arm_positions = None
    
    while simulation_app.is_running():
        rclpy.spin_once(teleop_node, timeout_sec=0.0)
        frame_count += 1
        
        # Only do IK after calibration
        if not teleop_node.calibrated:
            world.step(render=True)
            continue
        
        # Direct IK solve
        actions, success = ik_solver.compute_inverse_kinematics(
            target_position=teleop_node.target_pos,
            target_orientation=teleop_node.target_rot,
            frame_name="panda_hand"
        )
        
        gripper_pos = 0.0 if teleop_node.gripper_closed else 0.04
        
        if success:
            ik_success += 1
            arm_positions = np.array(actions).flatten()[:7]
            last_good_arm_positions = arm_positions.copy()
            full_positions = np.concatenate([arm_positions, [gripper_pos, gripper_pos]])
            franka.apply_action(ArticulationAction(joint_positions=full_positions))
        else:
            ik_fail += 1
            if last_good_arm_positions is not None:
                full_positions = np.concatenate([last_good_arm_positions, [gripper_pos, gripper_pos]])
                franka.apply_action(ArticulationAction(joint_positions=full_positions))
        
        # Debug output
        if frame_count % 600 == 0:
            total = ik_success + ik_fail
            rate = (ik_success / total * 100) if total > 0 else 0
            print(f"[Frame {frame_count}] IK: {rate:.1f}% | Gripper: {'CLOSED' if teleop_node.gripper_closed else 'OPEN'}")
        
        world.step(render=True)
    
    teleop_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
