# isaac_openarm_teleop_openpi.py
# Real-time VR Bimanual Teleoperation for OpenArm robot - OpenPI Compatible Version
# Uses both Meta Quest 3 controllers for left/right arm control
# 
# OpenPI Compatibility:
# - UNIFIED 16D format throughout: [left_7_joints, left_gripper, right_7_joints, right_gripper]
# - Gripper normalized to [0, 1] where 0=open, 1=closed (internally and in published data)
# - Joint angles in radians
# - Compatible with LeRobot v3.0 dataset format
#
# Home Pose: Computed from USD's initial joint positions using forward kinematics

from omni.isaac.kit import SimulationApp

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
from pxr import UsdGeom
import os
import yaml

# LOAD CONFIG
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
CONFIG_PATH = os.path.join(PROJECT_ROOT, "config", "config.yaml")

with open(CONFIG_PATH, 'r') as f:
    PATH_CONFIG = yaml.safe_load(f)

# CONFIGURE PATHS
USD_PATH = os.path.join(PROJECT_ROOT, PATH_CONFIG['paths']['openarm']['usd'])
URDF_PATH = os.path.join(PROJECT_ROOT, PATH_CONFIG['paths']['openarm']['urdf'])
LEFT_ARM_CONFIG_DIR = os.path.join(PROJECT_ROOT, PATH_CONFIG['paths']['openarm']['left_arm_config'])
RIGHT_ARM_CONFIG_DIR = os.path.join(PROJECT_ROOT, PATH_CONFIG['paths']['openarm']['right_arm_config'])

# ============================================================================
# CONFIGURATION - OpenPI Compatible
# ============================================================================
CONFIG = {
    "pos_scale": 1.0,
    "smoothing": 0.9,
    "gripper_threshold": 0.5,
    
    # OpenPI uses normalized gripper [0, 1] where 0=open, 1=closed
    "gripper_raw_open": 0.11,
    "gripper_raw_closed": 0,
    
    # Gripper movement speed in NORMALIZED units per frame
    "gripper_speed": 0.1,
    
    "calibration_samples": 30,
    "debug_ik": False,
}

# OpenPI 16D joint names (this is what the model sees)
OPENPI_JOINT_NAMES = [
    "left_joint1", "left_joint2", "left_joint3", "left_joint4", 
    "left_joint5", "left_joint6", "left_joint7", "left_gripper",
    "right_joint1", "right_joint2", "right_joint3", "right_joint4",
    "right_joint5", "right_joint6", "right_joint7", "right_gripper"
]

# Robot's actual joint names
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

# IK preferred configurations (fallback if no warm start)
LEFT_ARM_PREFERRED_CONFIG = np.array([0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0])
RIGHT_ARM_PREFERRED_CONFIG = np.array([0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0])


# ============================================================================
# SCENE RANDOMIZATION FOR DATASET VARIATION
# ============================================================================

class SceneRandomizer:
    """Randomizes object positions on the table with small perturbations."""
    
    # Object paths in USD (at root level based on stage structure)
    # Format: { name: (path, height_offset_meters) }
    OBJECTS_TO_RANDOMIZE = {
        'box': ('/box', 0.2),
        'electric_screw_driver': ('/electric_screw_driver', 0.02),
    }
    TABLE_PATH = '/packing_table_01'
    
    # Small perturbation settings (most resets)
    POSITION_PERTURBATION = 0.15  # ±5cm random offset from current position
    ROTATION_PERTURBATION = 60.0  # ±30 degrees random rotation offset
    
    # Big perturbation settings (occasional - relocates objects across table)
    BIG_PERTURBATION_MIN_INTERVAL = 2   # Minimum resets before big perturbation
    BIG_PERTURBATION_MAX_INTERVAL = 4   # Maximum resets before big perturbation
    
    # Minimum distance between objects (meters)
    MIN_OBJECT_DISTANCE = 0.5
    
    # Margin from table edges (meters)
    EDGE_MARGIN = 0.1
    
    # Lighting randomization settings
    LIGHT_INTENSITY_MIN = 800.0    # Minimum light intensity (darker)
    LIGHT_INTENSITY_MAX = 1500.0   # Maximum light intensity (brighter)
    LIGHT_COLOR_TEMP_MIN = 4500    # Minimum color temperature (warmer/yellow)
    LIGHT_COLOR_TEMP_MAX = 7000    # Maximum color temperature (cooler/blue)
    LIGHT_ROTATION_RANGE = 30.0    # ±degrees rotation around Z-axis
    
    def __init__(self, stage):
        self.stage = stage
        self.table_bounds = None
        self.table_height = None
        self.reset_count = 0
        self.next_big_perturbation = np.random.randint(
            self.BIG_PERTURBATION_MIN_INTERVAL, 
            self.BIG_PERTURBATION_MAX_INTERVAL + 1
        )
        # Store last randomized poses for continuity after world.reset()
        self.last_poses = {}  # { obj_name: (position, rotation) }
        # Store original light settings for reference
        self.original_light_intensity = None
        self._query_table_bounds()
        self._find_lights()
    
    def _query_table_bounds(self):
        """Query table dimensions from USD."""
        from pxr import UsdGeom, Gf
        
        table_prim = self.stage.GetPrimAtPath(self.TABLE_PATH)
        if not table_prim.IsValid():
            print(f"[Randomizer] Warning: Table not found at {self.TABLE_PATH}")
            # Fallback bounds (approximate from visual)
            self.table_bounds = {
                'x_min': -0.4, 'x_max': 0.4,
                'y_min': -0.3, 'y_max': 0.3
            }
            self.table_height = 0.75
            return
        
        # Get the bounding box of the table
        bbox_cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
        bbox = bbox_cache.ComputeWorldBound(table_prim)
        bbox_range = bbox.GetBox()
        
        min_pt = bbox_range.GetMin()
        max_pt = bbox_range.GetMax()
        
        # Apply edge margins
        self.table_bounds = {
            'x_min': min_pt[0] + self.EDGE_MARGIN,
            'x_max': max_pt[0] - self.EDGE_MARGIN,
            'y_min': min_pt[1] + self.EDGE_MARGIN,
            'y_max': max_pt[1] - self.EDGE_MARGIN
        }
        # Table surface is at max Z of bounding box
        self.table_height = max_pt[2]
        
        print(f"[Randomizer] Table bounds: X[{self.table_bounds['x_min']:.2f}, {self.table_bounds['x_max']:.2f}] "
              f"Y[{self.table_bounds['y_min']:.2f}, {self.table_bounds['y_max']:.2f}] H={self.table_height:.2f}")
    
    def _find_lights(self):
        """Find all lights in the scene for randomization."""
        from pxr import UsdLux
        
        self.lights = []
        for prim in self.stage.Traverse():
            if prim.IsA(UsdLux.DomeLight) or prim.IsA(UsdLux.DistantLight) or prim.IsA(UsdLux.RectLight) or prim.IsA(UsdLux.SphereLight):
                self.lights.append(prim.GetPath())
                print(f"[Randomizer] Found light: {prim.GetPath()}")
        
        if not self.lights:
            print("[Randomizer] No lights found - will create dome light for randomization")
    
    def _color_temp_to_rgb(self, temp_kelvin):
        """Convert color temperature (Kelvin) to RGB values (0-1 range)."""
        # Approximation algorithm for color temperature
        temp = temp_kelvin / 100.0
        
        # Red
        if temp <= 66:
            r = 1.0
        else:
            r = 1.292936 * ((temp - 60) ** -0.1332047592)
            r = np.clip(r, 0, 1)
        
        # Green
        if temp <= 66:
            g = 0.390082 * np.log(temp) - 0.631841
        else:
            g = 1.129891 * ((temp - 60) ** -0.0755148492)
        g = np.clip(g, 0, 1)
        
        # Blue
        if temp >= 66:
            b = 1.0
        elif temp <= 19:
            b = 0.0
        else:
            b = 0.543207 * np.log(temp - 10) - 1.196254
            b = np.clip(b, 0, 1)
        
        return (r, g, b)
    
    def randomize_lighting(self):
        """Randomize lighting conditions in the scene."""
        from pxr import UsdLux, Gf
        from omni.isaac.core.prims import XFormPrim
        from scipy.spatial.transform import Rotation as R
        
        if not self.lights:
            return
        
        # Random intensity
        new_intensity = np.random.uniform(self.LIGHT_INTENSITY_MIN, self.LIGHT_INTENSITY_MAX)
        
        # Random color temperature
        color_temp = np.random.uniform(self.LIGHT_COLOR_TEMP_MIN, self.LIGHT_COLOR_TEMP_MAX)
        new_color = self._color_temp_to_rgb(color_temp)
        
        # Random direction (rotation around X and Y axes for lighting angle)
        rot_x = np.random.uniform(-self.LIGHT_ROTATION_RANGE, self.LIGHT_ROTATION_RANGE)
        rot_y = np.random.uniform(-self.LIGHT_ROTATION_RANGE, self.LIGHT_ROTATION_RANGE)
        rot_z = np.random.uniform(-180, 180)  # Full rotation around Z
        
        for light_path in self.lights:
            prim = self.stage.GetPrimAtPath(light_path)
            if not prim.IsValid():
                continue
            
            try:
                # Set intensity
                if prim.HasAttribute("inputs:intensity"):
                    prim.GetAttribute("inputs:intensity").Set(new_intensity)
                elif prim.HasAttribute("intensity"):
                    prim.GetAttribute("intensity").Set(new_intensity)
                
                # Set color
                if prim.HasAttribute("inputs:color"):
                    prim.GetAttribute("inputs:color").Set(Gf.Vec3f(*new_color))
                elif prim.HasAttribute("color"):
                    prim.GetAttribute("color").Set(Gf.Vec3f(*new_color))
                
                # Set rotation (direction) for the light
                try:
                    xform = XFormPrim(prim_path=str(light_path))
                    # Create quaternion from euler angles (xyz order)
                    rotation = R.from_euler('xyz', [rot_x, rot_y, rot_z], degrees=True)
                    quat = rotation.as_quat()  # Returns [x, y, z, w]
                    # Convert to wxyz format for Isaac Sim
                    quat_wxyz = np.array([quat[3], quat[0], quat[1], quat[2]])
                    
                    # Get current position, only change rotation
                    current_pos, _ = xform.get_world_pose()
                    xform.set_world_pose(position=current_pos, orientation=quat_wxyz)
                except Exception as rot_err:
                    pass  # Some lights may not support rotation
                
            except Exception as e:
                print(f"[Randomizer] Light error: {e}")
        
        print(f"[Lighting] intensity={new_intensity:.0f}, temp={color_temp:.0f}K, dir=({rot_x:.0f}°,{rot_y:.0f}°,{rot_z:.0f}°)")
    
    def _get_current_pose(self, obj_path):
        """Get current position and orientation of an object."""
        from omni.isaac.core.prims import XFormPrim
        try:
            xform = XFormPrim(prim_path=obj_path)
            pos, rot = xform.get_world_pose()
            return np.array(pos), np.array(rot)
        except:
            return None, None
    
    def _perturb_position(self, current_pos, height_offset):
        """Apply small random perturbation to position, clamped to table bounds."""
        # Random XY offset
        dx = np.random.uniform(-self.POSITION_PERTURBATION, self.POSITION_PERTURBATION)
        dy = np.random.uniform(-self.POSITION_PERTURBATION, self.POSITION_PERTURBATION)
        
        new_x = current_pos[0] + dx
        new_y = current_pos[1] + dy
        
        # Clamp to table bounds
        new_x = np.clip(new_x, self.table_bounds['x_min'], self.table_bounds['x_max'])
        new_y = np.clip(new_y, self.table_bounds['y_min'], self.table_bounds['y_max'])
        new_z = self.table_height + height_offset
        
        return np.array([new_x, new_y, new_z])
    
    def _perturb_rotation(self, current_rot):
        """Apply small random Z-rotation perturbation to orientation."""
        # Convert current quaternion to Z angle
        # For quaternion [w, x, y, z], Z rotation angle = 2 * atan2(z, w)
        current_angle = 2 * np.arctan2(current_rot[3], current_rot[0])
        
        # Add small random perturbation
        delta_angle = np.random.uniform(
            -np.radians(self.ROTATION_PERTURBATION), 
            np.radians(self.ROTATION_PERTURBATION)
        )
        new_angle = current_angle + delta_angle
        
        # Convert back to quaternion (wxyz format)
        w = np.cos(new_angle / 2)
        z = np.sin(new_angle / 2)
        return np.array([w, 0.0, 0.0, z])
    
    def _check_collision(self, pos1, pos2):
        """Check if two positions are too close."""
        dist = np.linalg.norm(pos1[:2] - pos2[:2])  # XY distance only
        return dist < self.MIN_OBJECT_DISTANCE
    
    def _big_perturbation_position(self, height_offset, use_left_side=None):
        """Generate random position on left or right side of table for big perturbation."""
        # Randomly choose side if not specified
        if use_left_side is None:
            use_left_side = np.random.choice([True, False])
        
        # Split table into left (positive Y) and right (negative Y) halves
        y_mid = (self.table_bounds['y_min'] + self.table_bounds['y_max']) / 2
        
        x = np.random.uniform(self.table_bounds['x_min'], self.table_bounds['x_max'])
        if use_left_side:
            y = np.random.uniform(y_mid, self.table_bounds['y_max'])
        else:
            y = np.random.uniform(self.table_bounds['y_min'], y_mid)
        z = self.table_height + height_offset
        
        return np.array([x, y, z]), use_left_side
    
    def _big_perturbation_rotation(self):
        """Generate fully random Z-rotation for big perturbation."""
        angle = np.random.uniform(0, 2 * np.pi)
        w = np.cos(angle / 2)
        z = np.sin(angle / 2)
        return np.array([w, 0.0, 0.0, z])
    
    def randomize_objects(self):
        """Randomize object positions - small perturbations most times, big occasionally."""
        from omni.isaac.core.prims import XFormPrim
        
        if self.table_bounds is None:
            print("[Randomizer] Error: Table bounds not available")
            return False
        
        # Increment reset counter and check if big perturbation is due
        self.reset_count += 1
        is_big_perturbation = (self.reset_count >= self.next_big_perturbation)
        
        if is_big_perturbation:
            print(f"[Randomizer] ★ BIG PERTURBATION (after {self.reset_count} resets) ★")
            # Reset counter and schedule next big perturbation
            self.reset_count = 0
            self.next_big_perturbation = np.random.randint(
                self.BIG_PERTURBATION_MIN_INTERVAL, 
                self.BIG_PERTURBATION_MAX_INTERVAL + 1
            )
        
        placed_positions = []
        
        for obj_name, (obj_path, height_offset) in self.OBJECTS_TO_RANDOMIZE.items():
            prim = self.stage.GetPrimAtPath(obj_path)
            if not prim.IsValid():
                print(f"[Randomizer] Warning: {obj_name} not found at {obj_path}")
                continue
            
            # For small perturbation, use stored position if available (to continue from last randomized position)
            # For big perturbation, we ignore stored position and place randomly
            if obj_name in self.last_poses and not is_big_perturbation:
                # Use last randomized position as base for small perturbation
                base_pos, base_rot = self.last_poses[obj_name]
            else:
                # Get current pose from USD (initial position or first time)
                base_pos, base_rot = self._get_current_pose(obj_path)
                if base_pos is None:
                    print(f"[Randomizer] Warning: Could not get pose for {obj_name}")
                    continue
            
            # Choose perturbation type
            if is_big_perturbation:
                # Big perturbation strategy:
                # - Box stays in center/back region
                # - Electric screw driver alternates left/right sides for arm variety
                max_attempts = 15
                
                if obj_name == 'box':
                    # Box: place in center-back region of table
                    for attempt in range(max_attempts):
                        # Center X, back half of table (positive X typically)
                        x_center = (self.table_bounds['x_min'] + self.table_bounds['x_max']) / 2
                        new_x = np.random.uniform(x_center - 0.1, self.table_bounds['x_max'])
                        new_y = np.random.uniform(self.table_bounds['y_min'] + 0.05, self.table_bounds['y_max'] - 0.05)
                        new_pos = np.array([new_x, new_y, self.table_height + height_offset])
                        
                        # Check collision
                        collision = False
                        for placed_pos in placed_positions:
                            if self._check_collision(new_pos, placed_pos):
                                collision = True
                                break
                        
                        if not collision:
                            break
                    
                    new_rot = self._big_perturbation_rotation()
                    print(f"[Randomizer] {obj_name}: CENTER, pos=({new_pos[0]:.2f}, {new_pos[1]:.2f})")
                    
                else:  # electric_screw_driver
                    # Screw driver: random LEFT or RIGHT side, away from box
                    use_left_side = np.random.choice([True, False])
                    
                    for attempt in range(max_attempts):
                        new_pos, side = self._big_perturbation_position(height_offset, use_left_side)
                        
                        # Check collision with MIN_OBJECT_DISTANCE
                        collision = False
                        for placed_pos in placed_positions:
                            if self._check_collision(new_pos, placed_pos):
                                collision = True
                                break
                        
                        if not collision:
                            break
                        
                        # If collision, try opposite side
                        if attempt == max_attempts // 2:
                            use_left_side = not use_left_side
                    
                    new_rot = self._big_perturbation_rotation()
                    side_str = "LEFT" if side else "RIGHT"
                    print(f"[Randomizer] {obj_name}: → {side_str} side, pos=({new_pos[0]:.2f}, {new_pos[1]:.2f})")
            else:
                # Small perturbation: offset from base position (stored or initial)
                max_attempts = 10
                new_pos = base_pos
                for attempt in range(max_attempts):
                    new_pos = self._perturb_position(base_pos, height_offset)
                    
                    # Check collision
                    collision = False
                    for placed_pos in placed_positions:
                        if self._check_collision(new_pos, placed_pos):
                            collision = True
                            break
                    
                    if not collision:
                        break
                
                new_rot = self._perturb_rotation(base_rot)
                delta_x = new_pos[0] - base_pos[0]
                delta_y = new_pos[1] - base_pos[1]
                print(f"[Randomizer] {obj_name}: Δpos=({delta_x:+.2f}, {delta_y:+.2f})")
            
            # Apply pose and store for next iteration
            try:
                xform = XFormPrim(prim_path=obj_path)
                xform.set_world_pose(position=new_pos, orientation=new_rot)
                placed_positions.append(new_pos)
                # Store this pose for next small perturbation
                self.last_poses[obj_name] = (new_pos.copy(), new_rot.copy())
            except Exception as e:
                print(f"[Randomizer] Error moving {obj_name}: {e}")
        
        return True


# ============================================================================
# GRIPPER CONVERSION FUNCTIONS
# ============================================================================

def gripper_raw_to_normalized(raw_value: float) -> float:
    """Convert robot's raw gripper to OpenPI [0, 1]. 0=open, 1=closed."""
    raw_open = CONFIG["gripper_raw_open"]
    raw_closed = CONFIG["gripper_raw_closed"]
    if abs(raw_open - raw_closed) < 1e-6:
        return 0.5
    normalized = (raw_open - raw_value) / (raw_open - raw_closed)
    return float(np.clip(normalized, 0.0, 1.0))


def gripper_normalized_to_raw(normalized_value: float) -> float:
    """Convert OpenPI [0, 1] to robot's raw gripper."""
    raw_open = CONFIG["gripper_raw_open"]
    raw_closed = CONFIG["gripper_raw_closed"]
    raw_value = raw_open - normalized_value * (raw_open - raw_closed)
    return float(raw_value)


class ArmState:
    """Tracks state for a single arm controller."""
    def __init__(self, name, transform_matrix):
        self.name = name
        self.transform_matrix = transform_matrix
        # Home position computed from USD initial pose via FK
        self.home_pos = None
        self.home_rot = None
        self.target_pos = np.array([0.2, 0.0, 0.2])
        self.target_rot = np.array([1.0, 0.0, 0.0, 0.0])
        self.smoothed_pos = np.array([0.2, 0.0, 0.2])
        self.smoothed_rot = np.array([1.0, 0.0, 0.0, 0.0])
        # NORMALIZED gripper: 0 = open, 1 = closed
        self.gripper_target = 0.0
        self.gripper_current = 0.0
        self.calibrated = False
        self.calibration_poses = []
        self.reference_pos = None
        self.pose_count = 0


class BimanualQuestTeleop(Node):
    """ROS2 node for bimanual Quest 3 teleoperation."""
    
    def __init__(self, config):
        super().__init__('isaac_openarm_teleop_openpi')
        self.config = config
        
        self.T = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
        
        # Arm states - home positions set later from USD
        self.left_arm = ArmState("left", self.T)
        self.right_arm = ArmState("right", self.T)
        
        self.button_a_pressed = False
        self.button_b_pressed = False
        self.button_x_pressed = False
        self.button_y_pressed = False
        
        # Quest controller subscriptions
        self.left_pose_sub = self.create_subscription(
            PoseStamped, '/quest/left_hand/pose', 
            lambda msg: self.pose_callback(msg, self.left_arm), 10)
        self.left_input_sub = self.create_subscription(
            Joy, '/quest/left_hand/inputs', 
            lambda msg: self.input_callback(msg, self.left_arm, is_left=True), 10)
        self.right_pose_sub = self.create_subscription(
            PoseStamped, '/quest/right_hand/pose', 
            lambda msg: self.pose_callback(msg, self.right_arm), 10)
        self.right_input_sub = self.create_subscription(
            Joy, '/quest/right_hand/inputs', 
            lambda msg: self.input_callback(msg, self.right_arm, is_left=False), 10)
        
        # OpenPI data publishers (16D format)
        self.state_pub = self.create_publisher(JointState, '/openpi/state', 10)
        self.action_pub = self.create_publisher(JointState, '/openpi/action', 10)
        
        # Camera publishers
        self.camera_pubs = {
            'head': self.create_publisher(Image, '/camera/head/image_raw', 10),
            'wrist_left': self.create_publisher(Image, '/camera/wrist_left/image_raw', 10),
            'wrist_right': self.create_publisher(Image, '/camera/wrist_right/image_raw', 10),
        }
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("OpenPI Teleop - USD-Based Home Pose")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Home pose computed from USD initial joint positions")
        self.get_logger().info("Data Format: 16D [7 joints + 1 gripper] x 2 arms")
        self.get_logger().info("Gripper: 0.0 = open, 1.0 = closed")
        self.get_logger().info("=" * 60)
    
    def set_home_from_fk(self, left_pos, left_rot, right_pos, right_rot):
        """Set home positions from forward kinematics results."""
        self.left_arm.home_pos = np.array(left_pos)
        self.left_arm.home_rot = np.array(left_rot)
        self.right_arm.home_pos = np.array(right_pos)
        self.right_arm.home_rot = np.array(right_rot)
        
        # Initialize targets to home
        self.left_arm.target_pos = self.left_arm.home_pos.copy()
        self.left_arm.smoothed_pos = self.left_arm.home_pos.copy()
        self.left_arm.target_rot = self.left_arm.home_rot.copy()
        self.left_arm.smoothed_rot = self.left_arm.home_rot.copy()
        
        self.right_arm.target_pos = self.right_arm.home_pos.copy()
        self.right_arm.smoothed_pos = self.right_arm.home_pos.copy()
        self.right_arm.target_rot = self.right_arm.home_rot.copy()
        self.right_arm.smoothed_rot = self.right_arm.home_rot.copy()
        
        self.get_logger().info(f"Left arm home: pos={left_pos}, rot={left_rot}")
        self.get_logger().info(f"Right arm home: pos={right_pos}, rot={right_rot}")
    
    def pose_callback(self, msg, arm_state):
        arm_state.pose_count += 1
        xr_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
        if not arm_state.calibrated:
            arm_state.calibration_poses.append(xr_pos.copy())
            if len(arm_state.calibration_poses) >= self.config["calibration_samples"]:
                # Reference is average VR position during calibration
                arm_state.reference_pos = np.mean(arm_state.calibration_poses, axis=0)
                arm_state.calibrated = True
                self.get_logger().info(f"{arm_state.name.upper()} ARM CALIBRATED")
                self.get_logger().info(f"  VR reference: {arm_state.reference_pos}")
                self.get_logger().info(f"  Robot home: {arm_state.home_pos}")
            return
        
        # VR offset from calibration reference
        xr_offset = xr_pos - arm_state.reference_pos
        # Transform to robot frame
        robot_offset = arm_state.transform_matrix @ xr_offset
        # Apply to home position
        robot_pos = robot_offset * self.config["pos_scale"] + arm_state.home_pos
        
        # Orientation
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
        robot_rot = np.array([quat_robot[3], quat_robot[0], quat_robot[1], quat_robot[2]])
        
        arm_state.target_pos = robot_pos
        arm_state.target_rot = robot_rot
    
    def input_callback(self, msg, arm_state, is_left):
        trigger = msg.axes[0] if len(msg.axes) > 0 else 0.0
        squeeze = msg.axes[1] if len(msg.axes) > 1 else 0.0
        
        if trigger > self.config["gripper_threshold"] or squeeze > self.config["gripper_threshold"]:
            arm_state.gripper_target = 1.0
        else:
            arm_state.gripper_target = 0.0
        
        if is_left:
            self.button_x_pressed = (len(msg.buttons) > 0 and msg.buttons[0] == 1)
            self.button_y_pressed = (len(msg.buttons) > 1 and msg.buttons[1] == 1)
        else:
            self.button_a_pressed = (len(msg.buttons) > 0 and msg.buttons[0] == 1)
            self.button_b_pressed = (len(msg.buttons) > 1 and msg.buttons[1] == 1)
    
    def get_openpi_state(self, robot_positions, left_arm_indices, right_arm_indices, 
                         left_gripper_indices, right_gripper_indices):
        """Convert robot's 18D raw positions to OpenPI 16D format."""
        state = []
        for idx in left_arm_indices:
            state.append(float(robot_positions[idx]))
        if left_gripper_indices:
            state.append(gripper_raw_to_normalized(robot_positions[left_gripper_indices[0]]))
        else:
            state.append(0.0)
        for idx in right_arm_indices:
            state.append(float(robot_positions[idx]))
        if right_gripper_indices:
            state.append(gripper_raw_to_normalized(robot_positions[right_gripper_indices[0]]))
        else:
            state.append(0.0)
        return state
    
    def get_openpi_action(self, left_arm_positions, right_arm_positions,
                          left_gripper_normalized, right_gripper_normalized):
        """Build OpenPI 16D action."""
        action = []
        for pos in left_arm_positions:
            action.append(float(pos))
        action.append(float(left_gripper_normalized))
        for pos in right_arm_positions:
            action.append(float(pos))
        action.append(float(right_gripper_normalized))
        return action
    
    def publish_openpi_data(self, openpi_state, openpi_action):
        """Publish state and action in OpenPI 16D format."""
        timestamp = self.get_clock().now().to_msg()
        
        state_msg = JointState()
        state_msg.header.stamp = timestamp
        state_msg.header.frame_id = "openpi"
        state_msg.name = OPENPI_JOINT_NAMES
        state_msg.position = openpi_state
        self.state_pub.publish(state_msg)
        
        action_msg = JointState()
        action_msg.header.stamp = timestamp
        action_msg.header.frame_id = "openpi"
        action_msg.name = OPENPI_JOINT_NAMES
        action_msg.position = openpi_action
        self.action_pub.publish(action_msg)
    
    @property
    def both_calibrated(self):
        return self.left_arm.calibrated and self.right_arm.calibrated


def compute_fk_from_ik_solver(ik_solver, joint_positions, frame_name):
    """Use IK solver to compute forward kinematics."""
    try:
        result = ik_solver.compute_forward_kinematics(frame_name, joint_positions)
        
        if isinstance(result, tuple) and len(result) == 2:
            pos, rot = result
            pos = np.array(pos).flatten()
            rot = np.array(rot).flatten()
            
            if len(pos) >= 3 and len(rot) >= 4:
                position = pos[:3]
                rotation = np.array([rot[3], rot[0], rot[1], rot[2]])
                print(f"[FK] {frame_name}: pos={position}")
                return position, rotation
        
        print(f"[FK] Unexpected result type: {type(result)}")
        return None, None
                
    except Exception as e:
        print(f"[FK] Error computing FK for {frame_name}: {e}")
        import traceback
        traceback.print_exc()
        return None, None




def main():
    print("[Init] Warming up Isaac Sim...")
    for _ in range(30):
        simulation_app.update()
    
    print(f"[Init] Loading stage from {USD_PATH}...")
    open_stage(USD_PATH)
    
    for _ in range(50):
        simulation_app.update()
    
    print("[Init] Creating World...")
    world = World(stage_units_in_meters=1.0)
    
    for _ in range(20):
        simulation_app.update()
    
    # Find robot
    stage = world.stage
    robot_prim_path = None
    for path in ["/World/Robot", "/World/openarm", "/openarm", "/Robot", "/World/openarm_bimanual"]:
        if stage.GetPrimAtPath(path).IsValid():
            robot_prim_path = path
            break
    
    if robot_prim_path is None:
        print("[ERROR] Could not find OpenArm robot!")
        simulation_app.close()
        return
    
    print(f"[Init] Found robot at: {robot_prim_path}")
    
    openarm = world.scene.add(Articulation(prim_path=robot_prim_path, name="openarm"))
    
    # IK Solvers
    print("[Init] Loading IK Solvers...")
    from omni.isaac.motion_generation import LulaKinematicsSolver
    
    left_ik_solver = None
    right_ik_solver = None
    ik_enabled = False
    
    try:
        left_ik_solver = LulaKinematicsSolver(
            robot_description_path=os.path.join(LEFT_ARM_CONFIG_DIR, "robot_descriptor.yaml"),
            urdf_path=URDF_PATH
        )
        right_ik_solver = LulaKinematicsSolver(
            robot_description_path=os.path.join(RIGHT_ARM_CONFIG_DIR, "robot_descriptor.yaml"),
            urdf_path=URDF_PATH
        )
        ik_enabled = True
        print("[Init] IK Solvers loaded!")
    except Exception as e:
        print(f"[WARNING] IK Solvers failed: {e}")
    
    # Minimize UI
    import omni.ui
    for name in ["Stage", "Layer", "Render Settings", "Content", "Console", "Property"]:
        try:
            w = omni.ui.Workspace.get_window(name)
            if w:
                w.visible = False
        except:
            pass
    
    world.reset()
    for _ in range(20):
        simulation_app.update()
    
    # Get joint indices
    dof_names = openarm.dof_names
    print(f"[Info] DOFs: {dof_names}")
    
    left_arm_indices = [i for i, n in enumerate(dof_names) if n in LEFT_ARM_JOINTS]
    right_arm_indices = [i for i, n in enumerate(dof_names) if n in RIGHT_ARM_JOINTS]
    left_gripper_indices = [i for i, n in enumerate(dof_names) if n in LEFT_GRIPPER_JOINTS]
    right_gripper_indices = [i for i, n in enumerate(dof_names) if n in RIGHT_GRIPPER_JOINTS]
    
    print(f"[Info] Left arm: {left_arm_indices}, Right arm: {right_arm_indices}")
    
    # =========================================================================
    # COMPUTE HOME POSITIONS FROM USD INITIAL POSE USING FORWARD KINEMATICS
    # =========================================================================
    print("[Init] Computing home positions from USD initial pose...")
    
    initial_positions = openarm.get_joint_positions()
    print(f"[Init] USD initial joint positions: {initial_positions}")
    
    # Extract arm joint positions
    left_arm_initial = np.array([initial_positions[i] for i in left_arm_indices])
    right_arm_initial = np.array([initial_positions[i] for i in right_arm_indices])
    
    print(f"[Init] Left arm initial joints: {left_arm_initial}")
    print(f"[Init] Right arm initial joints: {right_arm_initial}")
    
    # Compute forward kinematics to get end-effector positions
    left_home_pos = np.array([0.25, 0.15, 0.25])  # Fallback
    left_home_rot = np.array([1.0, 0.0, 0.0, 0.0])
    right_home_pos = np.array([0.25, -0.15, 0.25])
    right_home_rot = np.array([1.0, 0.0, 0.0, 0.0])
    
    if ik_enabled and left_ik_solver and right_ik_solver:
        # Compute FK for left arm
        pos, rot = compute_fk_from_ik_solver(left_ik_solver, left_arm_initial, "openarm_left_hand")
        if pos is not None:
            left_home_pos = pos
            left_home_rot = rot
            print(f"[FK] Left arm home: pos={left_home_pos}")
        
        # Compute FK for right arm
        pos, rot = compute_fk_from_ik_solver(right_ik_solver, right_arm_initial, "openarm_right_hand")
        if pos is not None:
            right_home_pos = pos
            right_home_rot = rot
            print(f"[FK] Right arm home: pos={right_home_pos}")
    else:
        print("[Init] IK solvers not available, using fallback home positions")
    
    # Initialize ROS
    rclpy.init()
    teleop_node = BimanualQuestTeleop(CONFIG)
    
    # Set home positions from FK results
    teleop_node.set_home_from_fk(left_home_pos, left_home_rot, right_home_pos, right_home_rot)
    
    print("=" * 60)
    print("OpenArm Teleop Ready - USD-Based Home Pose")
    print("=" * 60)
    print(f"Left home:  {left_home_pos}")
    print(f"Right home: {right_home_pos}")
    print("=" * 60)
    
    # Tracking
    left_ik_success = right_ik_success = 0
    left_ik_fail = right_ik_fail = 0
    last_left_positions = left_arm_initial.copy()
    last_right_positions = right_arm_initial.copy()
    
    # Scene randomizer for dataset variation
    print("[Init] Setting up scene randomizer...")
    scene_randomizer = SceneRandomizer(stage)
    
    # Camera setup
    import omni.kit.viewport.utility
    cameras = ["/OmniverseKit_Persp"]
    camera_names = ["Perspective"]
    
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Camera):
            path = str(prim.GetPath())
            if path not in ["/OmniverseKit_Persp", "/OmniverseKit_Front", "/OmniverseKit_Right"]:
                cameras.append(path)
                camera_names.append(path.split("/")[-1].replace("_", " ").title())
    
    current_cam = 0
    last_button_a = False
    last_button_y = False
    
    # Camera recording
    import omni.replicator.core as rep
    import threading
    import queue
    
    RECORDING_CAMERAS = {
        'head': '/openarm/openarm_body_link/head_camera',
        'wrist_left': '/openarm/openarm_left_link7/left_wrist_camera',
        'wrist_right': '/openarm/openarm_right_link7/right_wrist_camera',
    }
    
    camera_annotators = {}
    camera_names_list = []
    
    for cam_name, cam_path in RECORDING_CAMERAS.items():
        if stage.GetPrimAtPath(cam_path).IsValid():
            rp = rep.create.render_product(cam_path, (480, 360))
            annot = rep.AnnotatorRegistry.get_annotator("rgb")
            annot.attach([rp])
            camera_annotators[cam_name] = annot
            camera_names_list.append(cam_name)
            print(f"[Camera] Setup: {cam_name}")
    
    camera_queue = queue.Queue(maxsize=6)
    
    def camera_thread():
        while True:
            try:
                cam_name, img, ts = camera_queue.get(timeout=0.1)
                msg = Image()
                msg.header.stamp = ts
                msg.header.frame_id = cam_name
                msg.height, msg.width = img.shape[:2]
                msg.encoding = 'rgb8'
                msg.step = img.shape[1] * 3
                msg.data = img.tobytes()
                teleop_node.camera_pubs[cam_name].publish(msg)
                camera_queue.task_done()
            except queue.Empty:
                continue
            except:
                pass
    
    threading.Thread(target=camera_thread, daemon=True).start()
    camera_idx = 0
    
    # Main loop
    while simulation_app.is_running():
        rclpy.spin_once(teleop_node, timeout_sec=0.0)
        
        # Camera switch
        if teleop_node.button_a_pressed and not last_button_a:
            current_cam = (current_cam + 1) % len(cameras)
            viewport = omni.kit.viewport.utility.get_active_viewport()
            if viewport:
                viewport.camera_path = cameras[current_cam]
                print(f"[Camera] {camera_names[current_cam]}")
        last_button_a = teleop_node.button_a_pressed
        
        # Scene reset - resets to USD initial pose and randomizes objects
        if teleop_node.button_y_pressed and not last_button_y:
            print("[Reset] Resetting scene and randomizing objects...")
            world.reset()
            
            # Randomize object positions on table
            scene_randomizer.randomize_objects()
            
            # Randomize lighting conditions
            scene_randomizer.randomize_lighting()
            
            # Let physics settle after randomization
            for _ in range(10):
                simulation_app.update()
            
            # Reset arm states to home
            teleop_node.left_arm.smoothed_pos = teleop_node.left_arm.home_pos.copy()
            teleop_node.left_arm.target_pos = teleop_node.left_arm.home_pos.copy()
            teleop_node.left_arm.smoothed_rot = teleop_node.left_arm.home_rot.copy()
            teleop_node.left_arm.target_rot = teleop_node.left_arm.home_rot.copy()
            
            teleop_node.right_arm.smoothed_pos = teleop_node.right_arm.home_pos.copy()
            teleop_node.right_arm.target_pos = teleop_node.right_arm.home_pos.copy()
            teleop_node.right_arm.smoothed_rot = teleop_node.right_arm.home_rot.copy()
            teleop_node.right_arm.target_rot = teleop_node.right_arm.home_rot.copy()
            
            teleop_node.left_arm.gripper_current = 0.0
            teleop_node.right_arm.gripper_current = 0.0
            
            last_left_positions = left_arm_initial.copy()
            last_right_positions = right_arm_initial.copy()
            
            print("[Reset] Done!")
            while teleop_node.button_y_pressed:
                rclpy.spin_once(teleop_node, timeout_sec=0.01)
        last_button_y = teleop_node.button_y_pressed
        
        # Wait for calibration
        if not teleop_node.both_calibrated:
            if (teleop_node.left_arm.pose_count + teleop_node.right_arm.pose_count) % 30 == 1:
                left_cal = len(teleop_node.left_arm.calibration_poses)
                right_cal = len(teleop_node.right_arm.calibration_poses)
                print(f"[Cal] L:{left_cal}/30 R:{right_cal}/30")
            world.step(render=True)
            continue
        
        # Get current robot state
        current_positions = openarm.get_joint_positions()
        if current_positions is None:
            world.step(render=True)
            continue
        
        target_positions = current_positions.copy()
        
        # Apply smoothing
        alpha = CONFIG["smoothing"]
        teleop_node.left_arm.smoothed_pos = alpha * teleop_node.left_arm.smoothed_pos + (1-alpha) * teleop_node.left_arm.target_pos
        teleop_node.right_arm.smoothed_pos = alpha * teleop_node.right_arm.smoothed_pos + (1-alpha) * teleop_node.right_arm.target_pos
        
        teleop_node.left_arm.smoothed_rot = alpha * teleop_node.left_arm.smoothed_rot + (1-alpha) * teleop_node.left_arm.target_rot
        teleop_node.left_arm.smoothed_rot /= np.linalg.norm(teleop_node.left_arm.smoothed_rot)
        teleop_node.right_arm.smoothed_rot = alpha * teleop_node.right_arm.smoothed_rot + (1-alpha) * teleop_node.right_arm.target_rot
        teleop_node.right_arm.smoothed_rot /= np.linalg.norm(teleop_node.right_arm.smoothed_rot)
        
        # Left arm IK
        left_arm_positions = last_left_positions.copy()
        if ik_enabled:
            actions, success = left_ik_solver.compute_inverse_kinematics(
                target_position=teleop_node.left_arm.smoothed_pos,
                target_orientation=teleop_node.left_arm.smoothed_rot,
                frame_name="openarm_left_hand",
                warm_start=last_left_positions
            )
            if success:
                left_ik_success += 1
                left_arm_positions = np.array(actions).flatten()[:7]
                last_left_positions = left_arm_positions.copy()
            else:
                left_ik_fail += 1
        
        # Right arm IK
        right_arm_positions = last_right_positions.copy()
        if ik_enabled:
            actions, success = right_ik_solver.compute_inverse_kinematics(
                target_position=teleop_node.right_arm.smoothed_pos,
                target_orientation=teleop_node.right_arm.smoothed_rot,
                frame_name="openarm_right_hand",
                warm_start=last_right_positions
            )
            if success:
                right_ik_success += 1
                right_arm_positions = np.array(actions).flatten()[:7]
                last_right_positions = right_arm_positions.copy()
            else:
                right_ik_fail += 1
        
        # Update arm joints
        for i, idx in enumerate(left_arm_indices):
            if i < 7:
                target_positions[idx] = left_arm_positions[i]
        for i, idx in enumerate(right_arm_indices):
            if i < 7:
                target_positions[idx] = right_arm_positions[i]
        
        # Gripper control
        gripper_speed = CONFIG["gripper_speed"]
        
        if teleop_node.left_arm.gripper_current < teleop_node.left_arm.gripper_target:
            teleop_node.left_arm.gripper_current = min(
                teleop_node.left_arm.gripper_current + gripper_speed,
                teleop_node.left_arm.gripper_target)
        else:
            teleop_node.left_arm.gripper_current = max(
                teleop_node.left_arm.gripper_current - gripper_speed,
                teleop_node.left_arm.gripper_target)
        
        if teleop_node.right_arm.gripper_current < teleop_node.right_arm.gripper_target:
            teleop_node.right_arm.gripper_current = min(
                teleop_node.right_arm.gripper_current + gripper_speed,
                teleop_node.right_arm.gripper_target)
        else:
            teleop_node.right_arm.gripper_current = max(
                teleop_node.right_arm.gripper_current - gripper_speed,
                teleop_node.right_arm.gripper_target)
        
        # Convert normalized gripper to raw for robot
        left_gripper_raw = gripper_normalized_to_raw(teleop_node.left_arm.gripper_current)
        right_gripper_raw = gripper_normalized_to_raw(teleop_node.right_arm.gripper_current)
        
        for idx in left_gripper_indices:
            target_positions[idx] = left_gripper_raw
        for idx in right_gripper_indices:
            target_positions[idx] = right_gripper_raw
        
        # Apply action
        openarm.apply_action(ArticulationAction(joint_positions=target_positions))
        
        # Publish OpenPI data
        openpi_state = teleop_node.get_openpi_state(
            current_positions, left_arm_indices, right_arm_indices,
            left_gripper_indices, right_gripper_indices
        )
        openpi_action = teleop_node.get_openpi_action(
            left_arm_positions, right_arm_positions,
            teleop_node.left_arm.gripper_current,
            teleop_node.right_arm.gripper_current
        )
        teleop_node.publish_openpi_data(openpi_state, openpi_action)
        
        # Camera capture
        if camera_names_list:
            cam_name = camera_names_list[camera_idx]
            camera_idx = (camera_idx + 1) % len(camera_names_list)
            annot = camera_annotators.get(cam_name)
            if annot:
                try:
                    data = annot.get_data()
                    if data is not None and len(data) > 0:
                        img = np.ascontiguousarray(data[:, :, :3] if data.shape[2] == 4 else data, dtype=np.uint8)
                        camera_queue.put_nowait((cam_name, img, teleop_node.get_clock().now().to_msg()))
                except:
                    pass
        
        world.step(render=True)
    
    print(f"\n[Stats] Left IK: {left_ik_success} ok, {left_ik_fail} fail")
    print(f"[Stats] Right IK: {right_ik_success} ok, {right_ik_fail} fail")
    
    teleop_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
