# isaac_teleop.py
# Run this script using the Isaac Sim python.sh wrapper:
# ./python.sh path/to/isaac_teleop.py

from omni.isaac.kit import SimulationApp

# Start SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

import omni.graph.core as og
try:
    import rclpy
except ImportError:
    print("ERROR: Could not import rclpy. Please ensure you are NOT sourcing system ROS 2 (like /opt/ros/humble/setup.bash) before running this script. Isaac Sim uses its own bundled ROS 2 libraries.")
    raise

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
import numpy as np
import carb
from omni.isaac.core import World
from omni.isaac.franka import Franka
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.motion_generation import LulaKinematicsSolver
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from scipy.spatial.transform import Rotation as R

class QuestTeleop(Node):
    def __init__(self):
        super().__init__('isaac_quest_teleop')
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/quest/right_hand/pose',
            self.pose_callback,
            10
        )
        
        self.input_sub = self.create_subscription(
            Joy,
            '/quest/right_hand/inputs',
            self.input_callback,
            10
        )
        
        self.target_pos = np.array([0.5, 0.0, 0.5])
        self.target_rot = np.array([1.0, 0.0, 0.0, 0.0]) # w, x, y, z (Isaac uses w, x, y, z)
        self.gripper_closed = False
        
        # Coordinate Mapping (Verified in MuJoCo)
        # Robot X (Forward) = -XR Z (Forward)
        # Robot Y (Left)    = -XR X (Left)
        # Robot Z (Up)      =  XR Y (Up)
        self.pos_offset = np.array([0.3, 0.0, 0.7])

    def pose_callback(self, msg):
        # Raw OpenXR Data
        xr_x = msg.pose.position.x
        xr_y = msg.pose.position.y
        xr_z = msg.pose.position.z
        
        # Apply Mapping
        mapped_x = -xr_z
        mapped_y = -xr_x
        mapped_z =  xr_y
        
        self.target_pos = np.array([mapped_x, mapped_y, mapped_z]) + self.pos_offset
        
        # Orientation Mapping
        # Basis change matrix T (same as position mapping)
        T = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
        
        # XR Quaternion (x, y, z, w)
        r_xr = R.from_quat([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        
        mat_xr = r_xr.as_matrix()
        mat_mj = T @ mat_xr @ T.T
        
        # Apply 180-degree rotation around X-axis to flip gripper orientation
        # This corrects for the natural controller pose vs natural gripper pose
        flip_x = R.from_euler('x', 180, degrees=True).as_matrix()
        mat_mj = mat_mj @ flip_x
        
        r_mj = R.from_matrix(mat_mj)
        quat_mj = r_mj.as_quat() # x, y, z, w
        
        # Isaac expects (w, x, y, z)
        self.target_rot = np.array([quat_mj[3], quat_mj[0], quat_mj[1], quat_mj[2]])

    def input_callback(self, msg):
        # Trigger is usually axis 0. If > 0.5, close gripper.
        if len(msg.axes) > 0:
            self.gripper_closed = msg.axes[0] > 0.5

def main():
    # Initialize ROS
    rclpy.init()
    teleop_node = QuestTeleop()
    
    # Setup World
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    
    # Add Franka
    franka = Franka(prim_path="/World/Franka", name="franka")
    world.scene.add(franka)
    
    # Setup IK Solver (Lula)
    from omni.isaac.motion_generation import interface_config_loader
    config = interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
    
    kinematics_solver = LulaKinematicsSolver(
        robot_description_path=config["robot_description_path"],
        urdf_path=config["urdf_path"]
    )
    
    # Wait for world to start
    world.reset()
    
    print("Starting Teleop Loop...")
    
    while simulation_app.is_running():
        # Spin ROS
        rclpy.spin_once(teleop_node, timeout_sec=0.0)
        
        # Calculate IK
        # end_effector_frame_name might need to be "panda_hand" or "right_gripper"
        # "right_gripper" is common for the high-level Franka class
        actions, success = kinematics_solver.compute_inverse_kinematics(
            target_position=teleop_node.target_pos,
            target_orientation=teleop_node.target_rot,
            frame_name="panda_hand"
        )
        
        if success:
            # IK solver returns 7 joint positions (arm only)
            # Franka expects 9 (7 arm + 2 gripper)
            gripper_positions = np.array([0.04, 0.04]) if not teleop_node.gripper_closed else np.array([0.0, 0.0])
            full_positions = np.concatenate([actions, gripper_positions])
            franka.apply_action(ArticulationAction(joint_positions=full_positions))
        else:
            # carb.log_warn("IK Failed")
            pass

            
        world.step(render=True)
    
    teleop_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()

if __name__ == "__main__":
    main()
