# isaac_openarm_teleop.py
# Real-time VR Bimanual Teleoperation for OpenArm robot
# Uses both Meta Quest 3 controllers for left/right arm control

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
from sensor_msgs.msg import Joy, JointState, Image
import numpy as np
from scipy.spatial.transform import Rotation as R
from omni.isaac.core import World
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.robots import Robot
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf
import os
import yaml

# LOAD CONFIG
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
CONFIG_PATH = os.path.join(PROJECT_ROOT, "config", "config.yaml")

with open(CONFIG_PATH, 'r') as f:
    PATH_CONFIG = yaml.safe_load(f)

# CONFIGURE PATHS (from config.yaml)
USD_PATH = os.path.join(PROJECT_ROOT, PATH_CONFIG['paths']['openarm']['usd'])
URDF_PATH = os.path.join(PROJECT_ROOT, PATH_CONFIG['paths']['openarm']['urdf'])
LEFT_ARM_CONFIG_DIR = os.path.join(PROJECT_ROOT, PATH_CONFIG['paths']['openarm']['left_arm_config'])
RIGHT_ARM_CONFIG_DIR = os.path.join(PROJECT_ROOT, PATH_CONFIG['paths']['openarm']['right_arm_config'])

# CONFIGURATION
# Home positions are computed dynamically at calibration time
# based on where the user's hands are in VR space
CONFIG = {
    "pos_scale": 1.0,  # 1:1 VR to robot movement
    # Robot workspace center (where hands map to when at VR origin)
    "robot_workspace_center": [0.3, 0.0, 0.3],  # Forward, centered, mid-height
    # Left/Right offset from center (applied at calibration)
    "left_arm_offset": [0.0, 0.15, 0.0],   # Slightly left
    "right_arm_offset": [0.0, -0.15, 0.0],  # Slightly right
    # No workspace limits - let arms move freely like human arms
    # IK will naturally fail for unreachable positions
    "smoothing": 0.9,  # Smoothing factor (0=no smoothing, 0.9=very smooth)
    "gripper_threshold": 0.5,
    "gripper_open_pos": 0.132,  # Gripper fully open position (from USD joint upperLimit)
    "gripper_closed_pos": -1,  # Gripper fully closed position
    "gripper_speed": 0.05,  # Gripper movement per frame (faster = less delay)
    "calibration_samples": 30,
    "debug_ik": False,  # Enable IK debug
}

# OpenArm joint names
LEFT_ARM_JOINTS = [
    "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
    "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
    "openarm_left_joint7"
]
RIGHT_ARM_JOINTS = [
    "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
    "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
    "openarm_right_joint7"
]
LEFT_GRIPPER_JOINTS = ["openarm_left_finger_joint1", "openarm_left_finger_joint2"]
RIGHT_GRIPPER_JOINTS = ["openarm_right_finger_joint1", "openarm_right_finger_joint2"]

# Preferred IK seed configurations for natural elbow orientation
# Joint order: joint1, joint2, joint3, joint4, joint5, joint6, joint7
# Joint2 controls elbow - needs opposite signs for left/right arms
LEFT_ARM_PREFERRED_CONFIG = np.array([0.0, -1.0, 0.0, 1.2, 0.0, 0.0, 0.0])
RIGHT_ARM_PREFERRED_CONFIG = np.array([0.0, 1.0, 0.0, 1.2, 0.0, 0.0, 0.0])


class ArmState:
    """Tracks state for a single arm controller."""
    def __init__(self, name, arm_offset, transform_matrix):
        self.name = name
        self.arm_offset = np.array(arm_offset)  # Offset from workspace center
        self.transform_matrix = transform_matrix
        # Home position computed dynamically at calibration
        self.home_pos = None
        self.target_pos = np.array([0.3, 0.0, 0.3])  # Default until calibrated
        self.target_rot = np.array([1.0, 0.0, 0.0, 0.0])  # wxyz
        # Smoothed targets (what we actually send to IK)
        self.smoothed_pos = np.array([0.3, 0.0, 0.3])
        self.smoothed_rot = np.array([1.0, 0.0, 0.0, 0.0])
        self.gripper_closed = False
        self.smoothed_gripper_pos = 0.132  # Start open (matches gripper_open_pos)
        self.calibrated = False
        self.calibration_poses = []
        self.reference_pos = None
        self.pose_count = 0


class BimanualQuestTeleop(Node):
    """ROS2 node for bimanual Quest 3 teleoperation."""
    
    def __init__(self, config):
        super().__init__('isaac_openarm_teleop')
        self.config = config
        
        # Transformation matrix: VR (right-hand Z-up) to Robot frame
        # Adjust as needed for your coordinate system
        self.T = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
        
        # Robot workspace center
        self.workspace_center = np.array(config["robot_workspace_center"])
        
        # Initialize arm states with offsets (home computed at calibration)
        self.left_arm = ArmState("left", config["left_arm_offset"], self.T)
        self.right_arm = ArmState("right", config["right_arm_offset"], self.T)
        
        # Button states
        self.button_a_pressed = False  # Right controller A button
        self.button_x_pressed = False  # Left controller X button
        
        # Subscribe to left controller
        self.left_pose_sub = self.create_subscription(
            PoseStamped, '/quest/left_hand/pose', 
            lambda msg: self.pose_callback(msg, self.left_arm), 10)
        self.left_input_sub = self.create_subscription(
            Joy, '/quest/left_hand/inputs', 
            lambda msg: self.input_callback(msg, self.left_arm, is_left=True), 10)
        
        # Subscribe to right controller
        self.right_pose_sub = self.create_subscription(
            PoseStamped, '/quest/right_hand/pose', 
            lambda msg: self.pose_callback(msg, self.right_arm), 10)
        self.right_input_sub = self.create_subscription(
            Joy, '/quest/right_hand/inputs', 
            lambda msg: self.input_callback(msg, self.right_arm, is_left=False), 10)
        
        # Publisher for joint states (for LeRobot recording)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Publishers for camera images (for LeRobot recording)
        self.camera_pubs = {
            'head': self.create_publisher(Image, '/camera/head/image_raw', 10),
            'wrist_left': self.create_publisher(Image, '/camera/wrist_left/image_raw', 10),
            'wrist_right': self.create_publisher(Image, '/camera/wrist_right/image_raw', 10),
        }
        
        self.get_logger().info("BimanualQuestTeleop Initialized")
        self.get_logger().info("Publishing joint states to /joint_states for recording")
        self.get_logger().info("Publishing camera images to /camera/*/image_raw for recording")
        self.get_logger().info("Waiting for Quest controller data...")
    
    def pose_callback(self, msg, arm_state):
        """Process pose data for an arm."""
        arm_state.pose_count += 1
        xr_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
        # Calibration phase
        if not arm_state.calibrated:
            arm_state.calibration_poses.append(xr_pos.copy())
            if len(arm_state.calibration_poses) >= self.config["calibration_samples"]:
                arm_state.reference_pos = np.mean(arm_state.calibration_poses, axis=0)
                # Compute dynamic home position: workspace center + arm offset
                arm_state.home_pos = self.workspace_center + arm_state.arm_offset
                arm_state.target_pos = arm_state.home_pos.copy()
                arm_state.smoothed_pos = arm_state.home_pos.copy()
                arm_state.calibrated = True
                self.get_logger().info(f"{arm_state.name.upper()} ARM CALIBRATION COMPLETE")
                self.get_logger().info(f"  Home position set to: {arm_state.home_pos}")
            return
        
        # Transform VR position to robot frame - no clipping, free movement
        xr_offset = xr_pos - arm_state.reference_pos
        robot_offset = arm_state.transform_matrix @ xr_offset
        robot_pos = robot_offset * self.config["pos_scale"] + arm_state.home_pos
        
        # Transform orientation
        xr_quat = np.array([
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w
        ])
        r_xr = R.from_quat(xr_quat)
        mat_xr = r_xr.as_matrix()
        mat_robot = arm_state.transform_matrix @ mat_xr @ arm_state.transform_matrix.T
        flip = R.from_euler('x', 180, degrees=True).as_matrix()
        mat_robot = mat_robot @ flip
        quat_robot = R.from_matrix(mat_robot).as_quat()
        # Convert to wxyz format
        robot_rot = np.array([quat_robot[3], quat_robot[0], quat_robot[1], quat_robot[2]])
        
        arm_state.target_pos = robot_pos
        arm_state.target_rot = robot_rot
    
    def input_callback(self, msg, arm_state, is_left):
        """Process controller input for gripper and buttons."""
        trigger = msg.axes[0] if len(msg.axes) > 0 else 0.0
        squeeze = msg.axes[1] if len(msg.axes) > 1 else 0.0
        arm_state.gripper_closed = (
            trigger > self.config["gripper_threshold"] or 
            squeeze > self.config["gripper_threshold"]
        )
        
        # Button handling
        if is_left:
            self.button_x_pressed = (len(msg.buttons) > 0 and msg.buttons[0] == 1)
        else:
            self.button_a_pressed = (len(msg.buttons) > 0 and msg.buttons[0] == 1)
    
    @property
    def both_calibrated(self):
        return self.left_arm.calibrated and self.right_arm.calibrated


def main():
    # =========================================================================
    # STEP 1: Warm up Isaac Sim
    # =========================================================================
    print("[Init] Warming up Isaac Sim...")
    for _ in range(30):
        simulation_app.update()
    
    # =========================================================================
    # STEP 2: Load Stage
    # =========================================================================
    print(f"[Init] Loading stage from {USD_PATH}...")
    open_stage(USD_PATH)
    
    print("[Init] Stabilizing stage...")
    for _ in range(50):
        simulation_app.update()
    
    # =========================================================================
    # STEP 3: Setup World and Robot
    # =========================================================================
    print("[Init] Creating World...")
    world = World(stage_units_in_meters=1.0)
    
    for _ in range(20):
        simulation_app.update()
    
    # Add the OpenArm robot as an Articulation
    print("[Init] Loading OpenArm robot...")
    robot_prim_path = None
    
    # Check for robot prim in various possible locations
    from pxr import Usd
    stage = world.stage
    possible_paths = [
        "/World/Robot",
        "/World/openarm",
        "/openarm",
        "/Robot",
        "/Environment/openarm",
        "/World/openarm_bimanual",
    ]
    for path in possible_paths:
        robot_prim = stage.GetPrimAtPath(path)
        if robot_prim.IsValid():
            robot_prim_path = path
            break
    
    if robot_prim_path is None:
        print("[ERROR] Could not find OpenArm robot in the USD stage!")
        print("Available prims at root:")
        for p in stage.GetPseudoRoot().GetChildren():
            print(f"  - {p.GetPath()}")
        simulation_app.close()
        return
    
    print(f"[Init] Found robot at: {robot_prim_path}")
    
    # Create the articulation
    openarm = world.scene.add(
        Articulation(
            prim_path=robot_prim_path,
            name="openarm"
        )
    )
    
    # Initialize IK Solvers for each arm
    print("[Init] Loading IK Solvers...")
    from omni.isaac.motion_generation import LulaKinematicsSolver
    
    left_robot_desc_path = os.path.join(LEFT_ARM_CONFIG_DIR, "robot_descriptor.yaml")
    right_robot_desc_path = os.path.join(RIGHT_ARM_CONFIG_DIR, "robot_descriptor.yaml")
    
    try:
        left_ik_solver = LulaKinematicsSolver(
            robot_description_path=left_robot_desc_path,
            urdf_path=URDF_PATH
        )
        right_ik_solver = LulaKinematicsSolver(
            robot_description_path=right_robot_desc_path,
            urdf_path=URDF_PATH
        )
        ik_enabled = True
        print("[Init] IK Solvers loaded successfully!")
    except Exception as e:
        print(f"[WARNING] Could not load IK Solvers: {e}")
        print("[INFO] Falling back to joint position control mode")
        ik_enabled = False
    
    # =========================================================================
    # UI SETUP: Minimize Panels
    # =========================================================================
    import omni.ui
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
    
    for _ in range(20):
        simulation_app.update()
    
    # Get joint indices after world reset
    print("[Init] Getting joint information...")
    dof_names = openarm.dof_names
    print(f"[Info] Available DOFs: {dof_names}")
    
    # Map joint names to indices
    left_arm_indices = []
    right_arm_indices = []
    left_gripper_indices = []
    right_gripper_indices = []
    
    for i, name in enumerate(dof_names):
        if name in LEFT_ARM_JOINTS:
            left_arm_indices.append(i)
        elif name in RIGHT_ARM_JOINTS:
            right_arm_indices.append(i)
        elif name in LEFT_GRIPPER_JOINTS:
            left_gripper_indices.append(i)
        elif name in RIGHT_GRIPPER_JOINTS:
            right_gripper_indices.append(i)
    
    print(f"[Info] Left arm indices: {left_arm_indices}")
    print(f"[Info] Right arm indices: {right_arm_indices}")
    print(f"[Info] Left gripper indices: {left_gripper_indices}")
    print(f"[Info] Right gripper indices: {right_gripper_indices}")
    
    # NOTE: Joint drive settings (stiffness, damping, max_force) should be configured
    # directly in the USD file using analyze_usd_physics.py script

    # =========================================================================
    # STEP 4: Initialize ROS
    # =========================================================================
    print("[Init] Initializing ROS2...")
    rclpy.init()
    teleop_node = BimanualQuestTeleop(CONFIG)
    
    print("="*60)
    print("OpenArm Bimanual Teleop Ready")
    print(f"Loaded: {USD_PATH}")
    print("="*60)
    print("Controls:")
    print("  - Left Quest Controller  -> Left Arm")
    print("  - Right Quest Controller -> Right Arm")
    print("  - Trigger/Grip -> Close Gripper")
    print("  - A/X Button -> Camera Switch")
    print("="*60)
    
    # Tracking variables
    left_ik_success = 0
    left_ik_fail = 0
    right_ik_success = 0
    right_ik_fail = 0
    last_left_arm_positions = None
    last_right_arm_positions = None
    
    # Camera handling - dynamically find cameras in the scene
    import omni.kit.viewport.utility
    cameras = ["/OmniverseKit_Persp"]  # Always include default perspective
    camera_names = ["Perspective"]
    
    # Find additional cameras in the USD scene
    from pxr import UsdGeom
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Camera) and str(prim.GetPath()) != "/OmniverseKit_Persp" and str(prim.GetPath()) != '/OmniverseKit_Front' and str(prim.GetPath()) != '/OmniverseKit_Right':
            cameras.append(str(prim.GetPath()))
            # Create a friendly name from the path
            cam_name = str(prim.GetPath()).split("/")[-1].replace("_", " ").title()
            camera_names.append(cam_name)
    
    print(f"[Camera] Found {len(cameras)} cameras: {camera_names}")
    current_cam_index = 0
    last_button_a = False
    
    # =========================================================================
    # Camera capture setup for LeRobot recording (Async/Threaded)
    # =========================================================================
    import omni.replicator.core as rep
    import threading
    import queue
    
    # Camera paths for recording
    RECORDING_CAMERAS = {
        'head': '/openarm/openarm_body_link/head_camera',
        'wrist_left': '/openarm/openarm_left_link7/left_wrist_camera',
        'wrist_right': '/openarm/openarm_right_link7/right_wrist_camera',
    }
    
    # Higher resolution now that we have async publishing
    CAMERA_RESOLUTION = (480, 360)  # Good balance of quality and performance
    
    # Create render products for each camera
    camera_render_products = {}
    camera_annotators = {}
    for cam_name, cam_path in RECORDING_CAMERAS.items():
        cam_prim = stage.GetPrimAtPath(cam_path)
        if cam_prim.IsValid():
            rp = rep.create.render_product(cam_path, CAMERA_RESOLUTION)
            camera_render_products[cam_name] = rp
            # Create RGB annotator
            rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
            rgb_annot.attach([rp])
            camera_annotators[cam_name] = rgb_annot
            print(f"[Camera] Setup recording for: {cam_name} at {CAMERA_RESOLUTION}")
        else:
            print(f"[Camera] Warning: Camera not found: {cam_path}")
    
    # Async camera publishing setup
    camera_queue = queue.Queue(maxsize=3)  # Small queue to avoid memory buildup
    camera_thread_running = True
    
    def camera_publish_thread():
        """Background thread for ROS2 image publishing."""
        while camera_thread_running:
            try:
                # Get image data from queue (blocking with timeout)
                cam_name, img_rgb, timestamp = camera_queue.get(timeout=0.1)
                
                # Create and publish ROS2 Image message
                img_msg = Image()
                img_msg.header.stamp = timestamp
                img_msg.header.frame_id = cam_name
                img_msg.height = img_rgb.shape[0]
                img_msg.width = img_rgb.shape[1]
                img_msg.encoding = 'rgb8'
                img_msg.is_bigendian = False
                img_msg.step = img_rgb.shape[1] * 3
                img_msg.data = img_rgb.tobytes()
                
                teleop_node.camera_pubs[cam_name].publish(img_msg)
                camera_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                pass  # Silently ignore errors
    
    # Start camera publish thread
    cam_thread = threading.Thread(target=camera_publish_thread, daemon=True)
    cam_thread.start()
    print("[Camera] Async camera publisher started")
    
    # Frame counter for camera capture
    frame_counter = 0
    CAMERA_CAPTURE_INTERVAL = 2  # Capture every 2nd frame (~15 Hz at 30 FPS sim)
    
    # Main loop
    while simulation_app.is_running():
        rclpy.spin_once(teleop_node, timeout_sec=0.0)
        
        # Camera switch with A button (cycle through: Perspective -> Left Wrist -> Right Wrist -> Top View)
        if teleop_node.button_a_pressed and not last_button_a:
            current_cam_index = (current_cam_index + 1) % len(cameras)
            new_cam = cameras[current_cam_index]
            cam_name = camera_names[current_cam_index]
            viewport = omni.kit.viewport.utility.get_active_viewport()
            if viewport:
                try:
                    viewport.camera_path = new_cam
                    print(f"[Camera] Switched to: {cam_name} ({new_cam})")
                except:
                    print(f"[Camera] Could not switch to {cam_name}")
        last_button_a = teleop_node.button_a_pressed
        
        # Wait for calibration
        if not teleop_node.both_calibrated:
            if teleop_node.left_arm.pose_count > 0 or teleop_node.right_arm.pose_count > 0:
                left_status = "CALIBRATED" if teleop_node.left_arm.calibrated else f"Calibrating ({len(teleop_node.left_arm.calibration_poses)}/{CONFIG['calibration_samples']})"
                right_status = "CALIBRATED" if teleop_node.right_arm.calibrated else f"Calibrating ({len(teleop_node.right_arm.calibration_poses)}/{CONFIG['calibration_samples']})"
                # Only print occasionally to avoid spam
                if (teleop_node.left_arm.pose_count + teleop_node.right_arm.pose_count) % 30 == 1:
                    print(f"[Calibration] Left: {left_status} | Right: {right_status}")
            else:
                # No poses received yet - print diagnostic message
                import time
                if not hasattr(teleop_node, '_last_waiting_print'):
                    teleop_node._last_waiting_print = 0
                current_time = time.time()
                if current_time - teleop_node._last_waiting_print > 2.0:  # Print every 2 seconds
                    print("[Waiting] No Quest controller data received yet. Is the Quest ROS2 bridge running?")
                    print("          Check: ros2 topic list | grep quest")
                    print("          Expected topics: /quest/left_hand/pose, /quest/right_hand/pose")
                    teleop_node._last_waiting_print = current_time
            world.step(render=True)
            continue
        
        # Get current joint positions
        current_positions = openarm.get_joint_positions()
        if current_positions is None:
            world.step(render=True)
            continue
        
        # Prepare target positions
        target_positions = current_positions.copy()
        
        # Apply smoothing to reduce jittery motion
        alpha = CONFIG["smoothing"]
        if alpha > 0:
            # Exponential moving average for position
            teleop_node.left_arm.smoothed_pos = (
                alpha * teleop_node.left_arm.smoothed_pos + 
                (1 - alpha) * teleop_node.left_arm.target_pos
            )
            teleop_node.right_arm.smoothed_pos = (
                alpha * teleop_node.right_arm.smoothed_pos + 
                (1 - alpha) * teleop_node.right_arm.target_pos
            )
            # For rotation, use SLERP-like interpolation
            teleop_node.left_arm.smoothed_rot = (
                alpha * teleop_node.left_arm.smoothed_rot + 
                (1 - alpha) * teleop_node.left_arm.target_rot
            )
            teleop_node.left_arm.smoothed_rot /= np.linalg.norm(teleop_node.left_arm.smoothed_rot)
            teleop_node.right_arm.smoothed_rot = (
                alpha * teleop_node.right_arm.smoothed_rot + 
                (1 - alpha) * teleop_node.right_arm.target_rot
            )
            teleop_node.right_arm.smoothed_rot /= np.linalg.norm(teleop_node.right_arm.smoothed_rot)
        else:
            teleop_node.left_arm.smoothed_pos = teleop_node.left_arm.target_pos
            teleop_node.left_arm.smoothed_rot = teleop_node.left_arm.target_rot
            teleop_node.right_arm.smoothed_pos = teleop_node.right_arm.target_pos
            teleop_node.right_arm.smoothed_rot = teleop_node.right_arm.target_rot
        
        # Debug print targets (every 100 frames)
        frame_count = left_ik_success + left_ik_fail + right_ik_success + right_ik_fail
        if CONFIG.get("debug_ik") and frame_count % 100 == 0:
            print(f"[IK Debug] Left target: pos={teleop_node.left_arm.smoothed_pos}, rot={teleop_node.left_arm.smoothed_rot}")
            print(f"[IK Debug] Right target: pos={teleop_node.right_arm.smoothed_pos}, rot={teleop_node.right_arm.smoothed_rot}")
        
        # ----- LEFT ARM IK -----
        if ik_enabled:
            # Use smoothed targets for IK with warm_start for preferred elbow config
            warm_start = last_left_arm_positions if last_left_arm_positions is not None else LEFT_ARM_PREFERRED_CONFIG
            left_actions, left_success = left_ik_solver.compute_inverse_kinematics(
                target_position=teleop_node.left_arm.smoothed_pos,
                target_orientation=teleop_node.left_arm.smoothed_rot,
                frame_name="openarm_left_hand",
                warm_start=warm_start
            )
            
            if left_success:
                left_ik_success += 1
                left_arm_positions = np.array(left_actions).flatten()[:7]
                last_left_arm_positions = left_arm_positions.copy()
                for i, idx in enumerate(left_arm_indices):
                    if i < len(left_arm_positions):
                        target_positions[idx] = left_arm_positions[i]
            else:
                left_ik_fail += 1
                # Print failure reason on first few failures
                if left_ik_fail <= 3:
                    print(f"[IK] Left arm failed for target: {teleop_node.left_arm.smoothed_pos}")
                if last_left_arm_positions is not None:
                    for i, idx in enumerate(left_arm_indices):
                        if i < len(last_left_arm_positions):
                            target_positions[idx] = last_left_arm_positions[i]
        
        # ----- RIGHT ARM IK -----
        if ik_enabled:
            # Use warm_start with preferred elbow config for right arm
            warm_start = last_right_arm_positions if last_right_arm_positions is not None else RIGHT_ARM_PREFERRED_CONFIG
            right_actions, right_success = right_ik_solver.compute_inverse_kinematics(
                target_position=teleop_node.right_arm.smoothed_pos,
                target_orientation=teleop_node.right_arm.smoothed_rot,
                frame_name="openarm_right_hand",
                warm_start=warm_start
            )
            
            if right_success:
                right_ik_success += 1
                right_arm_positions = np.array(right_actions).flatten()[:7]
                last_right_arm_positions = right_arm_positions.copy()
                for i, idx in enumerate(right_arm_indices):
                    if i < len(right_arm_positions):
                        target_positions[idx] = right_arm_positions[i]
            else:
                right_ik_fail += 1
                # Print failure reason on first few failures
                if right_ik_fail <= 3:
                    print(f"[IK] Right arm failed for target: {teleop_node.right_arm.smoothed_pos}")
                if last_right_arm_positions is not None:
                    for i, idx in enumerate(right_arm_indices):
                        if i < len(last_right_arm_positions):
                            target_positions[idx] = last_right_arm_positions[i]
        
        # ----- GRIPPER CONTROL (with smoothing) -----
        gripper_speed = CONFIG["gripper_speed"]
        gripper_open = CONFIG["gripper_open_pos"]
        gripper_closed = CONFIG["gripper_closed_pos"]
        
        # Left gripper - smooth interpolation
        left_target = gripper_closed if teleop_node.left_arm.gripper_closed else gripper_open
        if teleop_node.left_arm.smoothed_gripper_pos < left_target:
            teleop_node.left_arm.smoothed_gripper_pos = min(
                teleop_node.left_arm.smoothed_gripper_pos + gripper_speed, left_target)
        elif teleop_node.left_arm.smoothed_gripper_pos > left_target:
            teleop_node.left_arm.smoothed_gripper_pos = max(
                teleop_node.left_arm.smoothed_gripper_pos - gripper_speed, left_target)
        for idx in left_gripper_indices:
            target_positions[idx] = teleop_node.left_arm.smoothed_gripper_pos
        
        # Right gripper - smooth interpolation
        right_target = gripper_closed if teleop_node.right_arm.gripper_closed else gripper_open
        if teleop_node.right_arm.smoothed_gripper_pos < right_target:
            teleop_node.right_arm.smoothed_gripper_pos = min(
                teleop_node.right_arm.smoothed_gripper_pos + gripper_speed, right_target)
        elif teleop_node.right_arm.smoothed_gripper_pos > right_target:
            teleop_node.right_arm.smoothed_gripper_pos = max(
                teleop_node.right_arm.smoothed_gripper_pos - gripper_speed, right_target)
        for idx in right_gripper_indices:
            target_positions[idx] = teleop_node.right_arm.smoothed_gripper_pos
        
        # Apply the action (position control only - joint drive settings in USD)
        openarm.apply_action(ArticulationAction(joint_positions=target_positions))
        
        # Publish joint states for LeRobot recording
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = teleop_node.get_clock().now().to_msg()
        joint_state_msg.name = dof_names
        joint_state_msg.position = target_positions.tolist()
        teleop_node.joint_state_pub.publish(joint_state_msg)
        
        # Capture and publish camera images for LeRobot recording (async)
        frame_counter += 1
        if frame_counter % CAMERA_CAPTURE_INTERVAL == 0:
            timestamp = teleop_node.get_clock().now().to_msg()
            for cam_name, annotator in camera_annotators.items():
                try:
                    # Get image data from annotator (fast)
                    data = annotator.get_data()
                    if data is not None and len(data) > 0:
                        # Convert to uint8 RGB (remove alpha if present)
                        if len(data.shape) == 3 and data.shape[2] == 4:
                            img_rgb = data[:, :, :3].astype(np.uint8)
                        else:
                            img_rgb = data.astype(np.uint8)
                        
                        # Queue for async publishing (non-blocking)
                        try:
                            camera_queue.put_nowait((cam_name, img_rgb.copy(), timestamp))
                        except queue.Full:
                            pass  # Drop frame if queue full (avoid backlog)
                except Exception:
                    pass  # Silently ignore camera capture errors
        
        world.step(render=True)
    
    # Cleanup
    print("\n" + "="*60)
    print("Session Statistics:")
    print(f"  Left Arm  - IK Success: {left_ik_success}, IK Fail: {left_ik_fail}")
    print(f"  Right Arm - IK Success: {right_ik_success}, IK Fail: {right_ik_fail}")
    print("="*60)
    
    teleop_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
