# isaac_teleop_usd.py
# Real-time VR Teleoperation loading 'environment.usd'
# Allows for WYSIWYG editing of camera/environment in Isaac Sim GUI.

from omni.isaac.kit import SimulationApp

# Headless = False to see the simulation
simulation_app = SimulationApp({
    "headless": False, 
    "width": 1920, 
    "height": 1080, 
    "window_width": 1920, 
    "window_height": 1080,
})

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
from omni.isaac.core.utils.stage import open_stage
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf

# CONFIGURE PATH TO USD
USD_PATH = "/home/saurabh/Development/quest3_streamer/environment.usd"

# CONFIGURATION (Same as original)
CONFIG = {
    "pos_scale": 1.0,
    "robot_home": [0.5, 0.0, 0.4],
    "workspace": {
        "x_min": 0.15, "x_max": 0.85,
        "y_min": -0.6, "y_max": 0.6,
        "z_min": 0.02, "z_max": 0.9,
    },
    "smoothing": 0.0,
    "gripper_threshold": 0.3,
    "calibration_samples": 30,
}

class QuestTeleop(Node):
    def __init__(self, config):
        super().__init__('isaac_quest_teleop_usd')
        self.config = config
        self.pose_count = 0
        self.pose_sub = self.create_subscription(PoseStamped, '/quest/right_hand/pose', self.pose_callback, 10)
        self.input_sub = self.create_subscription(Joy, '/quest/right_hand/inputs', self.input_callback, 10)
        self.target_pos = np.array(config["robot_home"])
        self.target_rot = np.array([1.0, 0.0, 0.0, 0.0])
        self.gripper_closed = False
        self.button_a_pressed = False
        self.calibrated = False
        self.calibration_poses = []
        self.reference_pos = None
        self.T = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
        self.get_logger().info("QuestTeleop Initialized (USD Mode)")

    def pose_callback(self, msg):
        self.pose_count += 1
        xr_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
        if not self.calibrated:
            self.calibration_poses.append(xr_pos.copy())
            if len(self.calibration_poses) >= self.config["calibration_samples"]:
                self.reference_pos = np.mean(self.calibration_poses, axis=0)
                self.calibrated = True
                self.get_logger().info("CALIBRATION COMPLETE")
            return
            
        xr_offset = xr_pos - self.reference_pos
        robot_offset = self.T @ xr_offset
        robot_pos = robot_offset * self.config["pos_scale"] + np.array(self.config["robot_home"])
        
        ws = self.config["workspace"]
        robot_pos[0] = np.clip(robot_pos[0], ws["x_min"], ws["x_max"])
        robot_pos[1] = np.clip(robot_pos[1], ws["y_min"], ws["y_max"])
        robot_pos[2] = np.clip(robot_pos[2], ws["z_min"], ws["z_max"])
        
        xr_quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        r_xr = R.from_quat(xr_quat)
        mat_xr = r_xr.as_matrix()
        mat_robot = self.T @ mat_xr @ self.T.T
        flip = R.from_euler('x', 180, degrees=True).as_matrix()
        mat_robot = mat_robot @ flip
        quat_robot = R.from_matrix(mat_robot).as_quat()
        robot_rot = np.array([quat_robot[3], quat_robot[0], quat_robot[1], quat_robot[2]])
        
        self.target_pos = robot_pos
        self.target_rot = robot_rot

    def input_callback(self, msg):
        trigger = msg.axes[0] if len(msg.axes) > 0 else 0.0
        squeeze = msg.axes[1] if len(msg.axes) > 1 else 0.0
        self.gripper_closed = trigger > self.config["gripper_threshold"] or squeeze > self.config["gripper_threshold"]
        self.button_a_pressed = (len(msg.buttons) > 0 and msg.buttons[0] == 1)

def main():
    # =========================================================================
    # STEP 1: Warm up Isaac Sim FIRST (before loading anything)
    # =========================================================================
    print("[Init] Warming up Isaac Sim...")
    for _ in range(30):
        simulation_app.update()
    
    # =========================================================================
    # STEP 2: Load Stage
    # =========================================================================
    print(f"[Init] Loading stage from {USD_PATH}...")
    open_stage(USD_PATH)
    
    # More warmup after stage load
    print("[Init] Stabilizing stage...")
    for _ in range(50):
        simulation_app.update()
    
    # =========================================================================
    # STEP 3: Setup World and Robot
    # =========================================================================
    print("[Init] Creating World...")
    world = World(stage_units_in_meters=1.0)
    
    # More warmup
    for _ in range(20):
        simulation_app.update()
    
    print("[Init] Adding Franka...")
    franka = Franka(prim_path="/World/Franka", name="franka")
    world.scene.add(franka)
    
    # IK Solver
    print("[Init] Loading IK Solver...")
    from omni.isaac.motion_generation import LulaKinematicsSolver, interface_config_loader
    mg_config = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
    ik_solver = LulaKinematicsSolver(
        robot_description_path=mg_config["robot_description_path"],
        urdf_path=mg_config["urdf_path"]
    )
    
    # =========================================================================
    # UI SETUP: Zoom Mode (Minimize Panels)
    # =========================================================================
    import omni.ui
    # Attempt to hide standard panels to maximize viewport
    windows_to_hide = [
        "Stage", "Layer", "Render Settings", "Content", "Content Library", 
        "Console", "Property", "Properties", "Semantics", "Visual Scripting"
    ]
    
    print("[UI] Minimizing panels for Zoom Mode...")
    for name in windows_to_hide:
        try:
            w = omni.ui.Workspace.get_window(name)
            if w:
                w.visible = False
        except:
            pass

    print("[Init] Resetting World...")
    world.reset()
    
    # Final warmup
    for _ in range(20):
        simulation_app.update()
    
    # =========================================================================
    # STEP 4: Initialize ROS AFTER everything else is stable
    # =========================================================================
    print("[Init] Initializing ROS2...")
    rclpy.init()
    teleop_node = QuestTeleop(CONFIG)
    
    print("="*60)
    print("Isaac Sim Teleop (USD Mode)")
    print(f"Loaded: {USD_PATH}")
    print("="*60)
    
    ik_success = 0
    ik_fail = 0
    last_good_arm_positions = None
    
    # Camera Switch
    import omni.kit.viewport.utility
    # Camera path is now under /World/Franka/panda_hand
    cameras = ["/OmniverseKit_Persp", "/World/Franka/panda_hand/gripper_camera"]
    current_cam_index = 0
    last_button_a = False
    
    while simulation_app.is_running():
        rclpy.spin_once(teleop_node, timeout_sec=0.0)
        
        # Camera Switch
        if teleop_node.button_a_pressed and not last_button_a:
            current_cam_index = (current_cam_index + 1) % len(cameras)
            new_cam = cameras[current_cam_index]
            viewport = omni.kit.viewport.utility.get_active_viewport()
            if viewport:
                try:
                    viewport.camera_path = new_cam
                    print(f"[Camera] Switched to {new_cam}")
                except:
                    print(f"[Camera] Could not switch to {new_cam}")
        last_button_a = teleop_node.button_a_pressed
        
        # IK Logic
        if not teleop_node.calibrated:
            world.step(render=True)
            continue
            
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
        
        world.step(render=True)
        
    teleop_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()

if __name__ == "__main__":
    main()
