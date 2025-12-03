import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import mujoco
import mujoco.viewer
import numpy as np
import time
import threading
from scipy.spatial.transform import Rotation as R

# Dummy Robot Arm with IK via Equality Constraint
# We use a 'mocap' body as the target, and 'weld' the end-effector to it.
# The physics engine solves the IK automatically.
xml = """
<mujoco>
  <option gravity="0 0 -9.81" integrator="implicitfast"/>
  
  <default>
    <joint damping="1" stiffness="0"/>
    <geom rgba="0.8 0.8 0.8 1"/>
  </default>

  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="2 2 0.1" rgba=".9 .9 .9 1"/>

    <!-- Robot Base -->
    <body name="base" pos="0 0 0">
        <geom type="cylinder" size="0.1 0.05" rgba="0.2 0.2 0.2 1"/>
        
        <!-- Shoulder Pan -->
        <body name="link1" pos="0 0 0.1">
            <joint name="joint1" type="hinge" axis="0 0 1" range="-2.9 2.9"/>
            <geom type="capsule" fromto="0 0 0 0 0 0.3" size="0.05"/>
            
            <!-- Shoulder Lift -->
            <body name="link2" pos="0 0 0.3">
                <joint name="joint2" type="hinge" axis="0 1 0" range="-1.8 1.8"/>
                <geom type="capsule" fromto="0 0 0 0 0 0.3" size="0.05"/>
                
                <!-- Elbow -->
                <body name="link3" pos="0 0 0.3">
                    <joint name="joint3" type="hinge" axis="0 1 0" range="-2.9 2.9"/>
                    <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.04"/>
                    
                    <!-- Wrist 1 -->
                    <body name="link4" pos="0.3 0 0">
                        <joint name="joint4" type="hinge" axis="1 0 0" range="-2.9 2.9"/>
                        <geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.03"/>
                        
                        <!-- Wrist 2 -->
                        <body name="link5" pos="0.2 0 0">
                            <joint name="joint5" type="hinge" axis="0 1 0" range="-2.9 2.9"/>
                            <geom type="capsule" fromto="0 0 0 0.1 0 0" size="0.03"/>
                            
                            <!-- End Effector -->
                            <body name="end_effector" pos="0.1 0 0">
                                <joint name="joint6" type="hinge" axis="1 0 0" range="-3.0 3.0"/>
                                <geom type="box" size="0.03 0.05 0.02" rgba="1 0 0 1"/>
                                <site name="ee_site" pos="0 0 0"/>
                            </body>
                        </body>
                    </body>
                </body>
            </body>
        </body>
    </body>

    <!-- Target (Mocap Body) - Controlled by ROS -->
    <body name="target" pos="0.4 0 0.4" mocap="true">
        <geom type="box" size="0.02 0.05 0.01" rgba="0 1 0 0.5" contype="0" conaffinity="0"/>
        <site name="target_site" pos="0 0 0"/>
        <!-- Add axes to visualize orientation clearly -->
        <site name="axis_x" pos="0.1 0 0" size="0.005" rgba="1 0 0 1"/>
        <site name="axis_y" pos="0 0.1 0" size="0.005" rgba="0 1 0 1"/>
        <site name="axis_z" pos="0 0 0.1" size="0.005" rgba="0 0 1 1"/>
    </body>
  </worldbody>

  <equality>
    <!-- Soft Weld: Pulls the end-effector towards the target -->
    <!-- This acts as our Inverse Kinematics solver -->
    <weld body1="end_effector" body2="target" solref="0.02 1" solimp=".9 .95 0.001"/>
  </equality>
</mujoco>
"""

class MujocoSim(Node):
    def __init__(self):
        super().__init__('mujoco_sim')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/quest/right_hand/pose',
            self.listener_callback,
            10)
        
        # Initial target position (reachable by the arm)
        self.target_pos = np.array([0.4, 0.0, 0.4])
        self.target_quat = np.array([1.0, 0.0, 0.0, 0.0])
        
        # Load Model
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        
        self.target_id = self.model.body('target').mocapid[0]
        
        # Offset to map Quest space to Robot space
        # Quest origin might be at floor, Robot base is at floor (0,0,0)
        # We might need to shift the input so it's in the robot's workspace
        self.pos_offset = np.array([0.5, 0.0, 0.0]) 

    def listener_callback(self, msg):
        # Map ROS Pose (OpenXR Data) to MuJoCo Coordinate System
        # OpenXR VIEW Space: +X (Right), +Y (Up), -Z (Forward)
        # MuJoCo/Robot Frame: +X (Forward), +Y (Left), +Z (Up)
        
        # Mapping:
        # Robot X (Forward) = -XR Z (Forward)
        # Robot Y (Left)    = -XR X (Left)
        # Robot Z (Up)      =  XR Y (Up)
        
        # Raw data from ROS topic (which is just raw OpenXR data)
        xr_x = msg.pose.position.x
        xr_y = msg.pose.position.y
        xr_z = msg.pose.position.z
        
        # Apply Rotation/Mapping
        mapped_x = -xr_z
        mapped_y = -xr_x
        mapped_z =  xr_y
        
        # Apply Offset
        # We want the "neutral" hand position to be in the robot's workspace.
        # Neutral Hand (relative to headset): ~0.3m down, ~0.4m forward
        # XR Coords: X=0, Y=-0.3, Z=-0.4
        # Mapped: X=0.4, Y=0, Z=-0.3
        # Robot Base is at Z=0. We want hand at Z=0.4 (above table).
        # So we need Z offset of +0.7
        
        self.pos_offset = np.array([0.3, 0.0, 0.7]) # Adjust X to push it further forward if needed
        
        self.target_pos = np.array([mapped_x, mapped_y, mapped_z]) + self.pos_offset
        
        self.target_pos = np.array([mapped_x, mapped_y, mapped_z]) + self.pos_offset
        
        # Orientation Mapping
        # We need to transform the rotation from XR frame to MJ frame.
        # T is the basis change matrix we derived for position:
        # [[ 0,  0, -1],
        #  [-1,  0,  0],
        #  [ 0,  1,  0]]
        
        T = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
        
        # Create Rotation object from XR quaternion (x, y, z, w)
        r_xr = R.from_quat([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        
        # Convert to matrix
        mat_xr = r_xr.as_matrix()
        
        # Apply basis change: R_mj = T @ R_xr @ T.T
        mat_mj = T @ mat_xr @ T.T
        
        # Convert back to quaternion (x, y, z, w)
        r_mj = R.from_matrix(mat_mj)
        quat_mj = r_mj.as_quat()
        
        # MuJoCo expects (w, x, y, z)
        self.target_quat = np.array([quat_mj[3], quat_mj[0], quat_mj[1], quat_mj[2]])

    def run_sim(self):
        print("Starting MuJoCo Simulation...")
        print("Green Sphere = Your Hand Target")
        print("Robot Arm = Tries to reach the target")
        
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running() and rclpy.ok():
                step_start = time.time()

                # Update Mocap Target
                self.data.mocap_pos[self.target_id] = self.target_pos
                self.data.mocap_quat[self.target_id] = self.target_quat

                # Step physics
                mujoco.mj_step(self.model, self.data)

                # Sync viewer
                viewer.sync()

                # Maintain real-time
                time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

def main(args=None):
    rclpy.init(args=args)
    sim_node = MujocoSim()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(sim_node,), daemon=True)
    spin_thread.start()
    
    try:
        sim_node.run_sim()
    except KeyboardInterrupt:
        pass
    finally:
        sim_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
