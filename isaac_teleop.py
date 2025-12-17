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
from omni.isaac.core.objects import DynamicSphere
from omni.isaac.core.prims import XFormPrim
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf


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
    
    # Camera Offset (Hand Frame -> Camera Frame)
    # Adjust these values to position the camera on the hand
    "camera_offset_pos": [0.1, 0.0, -0.15], # X, Y, Z
    "camera_offset_rot": [0.0, 180.0, -90.0], # Euler XYZ in degrees (Approximate from previous)
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
        self.button_a_pressed = False  # Track Button A state
        
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
        
        # Track Button A (index 0) for camera, toggle
        self.button_a_pressed = (len(msg.buttons) > 0 and msg.buttons[0] == 1)
        
    def recalibrate(self):
        """Call this to recalibrate (e.g., when user changes position)"""
        self.calibrated = False
        self.calibration_poses = []
        self.reference_pos = None
        self.get_logger().info("Recalibrating... hold hand steady")


def create_bowl(stage, prim_path: str, position: tuple, radius: float = 0.08, height: float = 0.05):
    """
    Create a physics-enabled bowl using a hollow cylinder approximation.
    The bowl consists of:
    - A bottom disc (thin cylinder)
    - A cylindrical wall (torus-like ring made of small spheres, or we use a cone)
    
    For simplicity, we'll create a cone with an open top to act as a bowl.
    """
    from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf, Usd
    
    # Create Xform for the bowl
    bowl_xform = UsdGeom.Xform.Define(stage, prim_path)
    bowl_xform.AddTranslateOp().Set(Gf.Vec3f(*position))
    
    # Create the bowl base (bottom disc)
    base_path = f"{prim_path}/base"
    base = UsdGeom.Cylinder.Define(stage, base_path)
    base.GetRadiusAttr().Set(radius)
    base.GetHeightAttr().Set(0.005)  # Thin base
    base.CreateDisplayColorAttr([(0.6, 0.3, 0.1)])  # Brown color
    
    # Position base at bottom
    base_xformable = UsdGeom.Xformable(base.GetPrim())
    base_xformable.AddTranslateOp().Set(Gf.Vec3f(0, 0, 0.0025))
    
    # Add physics to base
    UsdPhysics.RigidBodyAPI.Apply(bowl_xform.GetPrim())
    UsdPhysics.CollisionAPI.Apply(base.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(base.GetPrim())
    
    # Create bowl walls using a torus approximation (cone frustum)
    # We'll use multiple small cylinders arranged in a ring, or just use a single hollow cone
    # For simplicity, let's create a cone and subtract inner cone (or just use outer ring)
    
    # Create outer wall segments (ring of small boxes/cylinders)
    num_segments = 16
    wall_thickness = 0.008
    for i in range(num_segments):
        import math
        angle = (2 * math.pi * i) / num_segments
        seg_x = (radius - wall_thickness/2) * math.cos(angle)
        seg_y = (radius - wall_thickness/2) * math.sin(angle)
        
        seg_path = f"{prim_path}/wall_{i}"
        seg = UsdGeom.Cube.Define(stage, seg_path)
        seg.GetSizeAttr().Set(1.0)
        
        seg_xformable = UsdGeom.Xformable(seg.GetPrim())
        seg_xformable.AddTranslateOp().Set(Gf.Vec3f(seg_x, seg_y, height/2 + 0.005))
        seg_xformable.AddScaleOp().Set(Gf.Vec3f(wall_thickness, 0.035, height))
        seg_xformable.AddRotateZOp().Set(math.degrees(angle))
        
        seg.CreateDisplayColorAttr([(0.6, 0.3, 0.1)])  # Brown color
        UsdPhysics.CollisionAPI.Apply(seg.GetPrim())
    
    # Mass properties
    mass_api = UsdPhysics.MassAPI.Apply(bowl_xform.GetPrim())
    mass_api.GetMassAttr().Set(0.5)  # 500g bowl - heavy enough to stay in place
    
    return bowl_xform


def main():
    rclpy.init()
    teleop_node = QuestTeleop(CONFIG)
    
    # Setup World
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    
    # Add Franka
    franka = Franka(prim_path="/World/Franka", name="franka")
    world.scene.add(franka)
    # =========================================================================
    # ADD GRIPPER CAMERA (Fixed Joint Attachment)
    # =========================================================================
    from pxr import UsdGeom, Gf, UsdPhysics, PhysxSchema
    from omni.isaac.core.utils.stage import get_current_stage
    stage = get_current_stage()
    
    gripper_cam_path = "/World/gripper_camera"
    gripper_cam = UsdGeom.Camera.Define(stage, gripper_cam_path)
    
    # Set camera properties
    gripper_cam.GetFocalLengthAttr().Set(18.0)
    gripper_cam.GetFocusDistanceAttr().Set(0.1)
    gripper_cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))
    
    # -------------------------------------------------------------------------
    # Create Fixed Joint to Panda Hand
    # -------------------------------------------------------------------------
    # 1. Make Camera a Rigid Body (Required for Joints, mass roughly 0)
    UsdPhysics.RigidBodyAPI.Apply(gripper_cam.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(gripper_cam.GetPrim())
    mass_api.GetMassAttr().Set(0.001) # Negligible mass
    
    # 2. Find Panda Hand
    hand_prim_path = "/World/Franka/panda_hand"
    
    # 3. Create Fixed Joint
    joint_path = f"{gripper_cam_path}/FixedJoint"
    fixed_joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    
    # 4. Configure Joint Bodies
    fixed_joint.GetBody0Rel().SetTargets([hand_prim_path])
    fixed_joint.GetBody1Rel().SetTargets([gripper_cam_path])
    
    # 5. Set Local Pose (Offset)
    # We apply the offset to Body1 (Camera) relative to Body0 (Hand)
    # Note: Physics joints often use "Local Position 0" and "Local Position 1" 
    # which are the attachment points in each body's frame.
    # If we want Body1 (Cam) to be at Offset X relative to Body0 (Hand):
    # - LocalPos0 = Offset
    # - LocalPos1 = Zero
    # OR
    # - LocalPos0 = Zero
    # - LocalPos1 = -Offset (Inverse)
    
    # Let's use LocalPos0 (Hand Frame) to define where the joint is.
    # And LocalPos1 (Cam Frame) is origin of camera.
    
    # Get Config
    cam_pos = CONFIG.get("camera_offset_pos", [0.1, 0.0, -0.15])
    cam_rot_euler = CONFIG.get("camera_offset_rot", [0.0, 0.0, 0.0])
    
    pos_vec = Gf.Vec3f(*cam_pos)
    
    # Convert Euler (Deg) to Quat/Rotation
    # USD Physics joints use Position and Orientation attributes
    # Orientation is a Quatf (w, x, y, z)
    
    # Create Rotation Matrix from Euler (XYZ)
    # Note: Gf.Rotation constructs from axis/angle. We need to compose.
    rot_x = Gf.Rotation(Gf.Vec3d(1,0,0), cam_rot_euler[0])
    rot_y = Gf.Rotation(Gf.Vec3d(0,1,0), cam_rot_euler[1])
    rot_z = Gf.Rotation(Gf.Vec3d(0,0,1), cam_rot_euler[2])
    
    # Compose rotations
    final_rot = rot_x * rot_y * rot_z
    
    quat = final_rot.GetQuat() # Returns Gf.Quatd
    
    # Explicitly convert to Gf.Quatf
    # Quatf constructor takes (real, i, j, k) or (real, Vec3f)
    # We'll use (w, x, y, z) to be safe if supported, or ensure Vec3f
    # Gf.Quatf(real, imag) is the standard.
    imag = quat.GetImaginary() # Vec3d
    imag_f = Gf.Vec3f(float(imag[0]), float(imag[1]), float(imag[2]))
    quat_f = Gf.Quatf(float(quat.GetReal()), imag_f)
    
    fixed_joint.GetLocalPos0Attr().Set(pos_vec)
    fixed_joint.GetLocalRot0Attr().Set(quat_f)
    
    fixed_joint.GetLocalPos1Attr().Set(Gf.Vec3f(0,0,0))
    fixed_joint.GetLocalRot1Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0,0,0)))
    
    print(f"[Setup] Fixed Joint created: Camera attached to {hand_prim_path}")
    print(f"[Setup] Offset: Pos={cam_pos}, Rot={cam_rot_euler}")
    print("[TIP] Open Viewport 2 -> Select 'gripper_camera'")
    
    # =========================================================================
    # ADD PHYSICS OBJECTS: Bowl and Ball
    # =========================================================================
    # (stage already obtained above for camera setup)
    
    # Add Bowl (positioned on the table, to the right of the robot)
    bowl_position = (0.6, -0.35, 0.0)  # X forward, Y right, Z up (moved farther)
    create_bowl(stage, "/World/Bowl", bowl_position, radius=0.08, height=0.05)
    print(f"[Setup] Bowl placed at {bowl_position}")
    
    # Add Ball (small dynamic sphere, positioned in front of robot for easy pickup)
    ball = DynamicSphere(
        prim_path="/World/Ball",
        name="ball",
        position=np.array([0.55, 0.25, 0.03]),  # In front and to the left (moved farther)
        radius=0.025,  # 2.5cm radius ball (5cm diameter)
        color=np.array([1.0, 0.2, 0.2]),  # Red ball
        mass=0.1  # 100g ball
    )
    world.scene.add(ball)
    print(f"[Setup] Ball placed at (0.55, 0.25, 0.03)")
    
    print("="*60)
    print("TASK: Grab the RED BALL and place it in the BROWN BOWL")
    print("Ball: Front-left of robot | Bowl: Front-right of robot")
    print("="*60)
    
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
    
    # Helper to get hand pose (not strictly needed for camera anymore)
    from pxr import Usd
    hand_prim = stage.GetPrimAtPath("/World/Franka/panda_link8")
    if not hand_prim:
        hand_prim = stage.GetPrimAtPath("/World/Franka/panda_hand")
    
    # Camera Switching Logic
    import omni.kit.viewport.utility
    current_cam_index = 0
    cameras = ["/OmniverseKit_Persp", gripper_cam_path]
    last_button_a = False
    
    print("[TIP] Press 'A' or 'X' button on controller to SWITCH CAMERAS")
    
    while simulation_app.is_running():
        rclpy.spin_once(teleop_node, timeout_sec=0.0)
        frame_count += 1
        
        # Check Camera Switch
        if teleop_node.button_a_pressed and not last_button_a:
            current_cam_index = (current_cam_index + 1) % len(cameras)
            new_cam = cameras[current_cam_index]
            viewport = omni.kit.viewport.utility.get_active_viewport()
            if viewport:
                viewport.camera_path = new_cam
                print(f"[Camera] Switched to {new_cam}")
        last_button_a = teleop_node.button_a_pressed
        
        # Camera attached via FixedJoint - no manual update needed
            
        
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
