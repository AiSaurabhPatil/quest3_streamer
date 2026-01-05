# setup_stage.py
# Generates the initial USD stage for Isaac Sim Teleoperation
# Usage: ./python.sh setup_stage.py
# This script creates 'environment.usd' which you can then edit in Isaac Sim GUI.

from omni.isaac.kit import SimulationApp

# Headless = True to generate file quickly without UI
simulation_app = SimulationApp({"headless": True})

from omni.isaac.core import World
from omni.isaac.franka import Franka
from omni.isaac.core.objects import DynamicSphere
from omni.isaac.core.utils.stage import get_current_stage, save_stage, add_reference_to_stage
from pxr import UsdGeom, Gf, UsdPhysics, PhysxSchema, Usd
import numpy as np

# Isaac Sim Assets URL
ISAAC_ASSETS_URL = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.2/Isaac"


def add_physics_object(stage, asset_url: str, prim_path: str, position: tuple,
                       orientation_euler: tuple = (0, 0, 0), scale: float = 1.0,
                       mass: float = 0.2, static_friction: float = 0.7,
                       dynamic_friction: float = 0.5):
    """
    Load a USD asset and apply physics properties for pick-and-place.
    
    Args:
        stage: USD stage
        asset_url: URL or path to USD asset
        prim_path: Prim path for the object
        position: (x, y, z) position tuple
        orientation_euler: (rx, ry, rz) rotation in degrees
        scale: Uniform scale factor
        mass: Mass in kg
        static_friction: Static friction coefficient
        dynamic_friction: Dynamic friction coefficient
    """
    # Add reference to the asset
    add_reference_to_stage(usd_path=asset_url, prim_path=prim_path)
    
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f"[Warning] Failed to load asset at {prim_path}")
        return None
    
    # Apply transform
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(*position))
    xformable.AddRotateXYZOp().Set(Gf.Vec3d(*orientation_euler))
    xformable.AddScaleOp().Set(Gf.Vec3d(scale, scale, scale))
    
    # Apply RigidBody API for dynamics
    UsdPhysics.RigidBodyAPI.Apply(prim)
    
    # Apply Mass
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.GetMassAttr().Set(mass)
    
    # Create Physics Material for friction
    material_path = f"{prim_path}/PhysicsMaterial"
    material_prim = stage.DefinePrim(material_path, "Material")
    
    # Apply USD Physics material and set friction
    phys_material = UsdPhysics.MaterialAPI.Apply(material_prim)
    phys_material.CreateStaticFrictionAttr().Set(static_friction)
    phys_material.CreateDynamicFrictionAttr().Set(dynamic_friction)
    phys_material.CreateRestitutionAttr().Set(0.1)  # Low bounce
    
    # Apply collision to all mesh children
    def apply_collision_recursive(p):
        if p.GetTypeName() == "Mesh":
            UsdPhysics.CollisionAPI.Apply(p)
            UsdPhysics.MeshCollisionAPI.Apply(p)
        for child in p.GetChildren():
            apply_collision_recursive(child)
    
    apply_collision_recursive(prim)
    
    print(f"[Setup] Added physics object: {prim_path} (mass={mass}kg, friction={static_friction}/{dynamic_friction})")
    return prim

def create_bowl(stage, prim_path: str, position: tuple, radius: float = 0.08, height: float = 0.05):
    """Create a physics-enabled bowl using USD primitives."""
    # Create Xform for the bowl
    bowl_xform = UsdGeom.Xform.Define(stage, prim_path)
    bowl_xform.AddTranslateOp().Set(Gf.Vec3f(*position))
    
    # Create the bowl base (bottom disc)
    base_path = f"{prim_path}/base"
    base = UsdGeom.Cylinder.Define(stage, base_path)
    base.GetRadiusAttr().Set(radius)
    base.GetHeightAttr().Set(0.005)  
    base.CreateDisplayColorAttr([(0.6, 0.3, 0.1)])  
    
    base_xformable = UsdGeom.Xformable(base.GetPrim())
    base_xformable.AddTranslateOp().Set(Gf.Vec3f(0, 0, 0.0025))
    
    # Add physics to base
    UsdPhysics.RigidBodyAPI.Apply(bowl_xform.GetPrim())
    UsdPhysics.CollisionAPI.Apply(base.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(base.GetPrim())
    
    # Create bowl walls (simple approximated visual)
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
        
        seg.CreateDisplayColorAttr([(0.6, 0.3, 0.1)])  
        UsdPhysics.CollisionAPI.Apply(seg.GetPrim())
    
    mass_api = UsdPhysics.MassAPI.Apply(bowl_xform.GetPrim())
    mass_api.GetMassAttr().Set(0.5)
    
    return bowl_xform

def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    
    # Add Franka
    franka = Franka(prim_path="/World/Franka", name="franka")
    world.scene.add(franka)
    
    # Add Objects
    stage = get_current_stage()
    
    # Bowl
    bowl_pos = (0.6, -0.35, 0.0)
    create_bowl(stage, "/World/Bowl", bowl_pos)
    
    # Ball
    ball = DynamicSphere(
        prim_path="/World/Ball",
        name="ball",
        position=np.array([0.55, 0.25, 0.03]),
        radius=0.025,
        color=np.array([1.0, 0.2, 0.2]),
        mass=0.1
    )
    world.scene.add(ball)
    
    # -------------------------------------------------------------------------
    # PHYSICS OBJECTS (Mugs, Container, Cube)
    # -------------------------------------------------------------------------
    print("[Setup] Adding physics objects...")
    
    # Mug A2 (green mug)
    add_physics_object(
        stage, 
        f"{ISAAC_ASSETS_URL}/Props/Mugs/SM_Mug_A2.usd",
        "/World/Mug_A2",
        position=(0.4, 0.25, 0.02),
        mass=0.15,
        static_friction=0.8,
        dynamic_friction=0.6
    )
    
    # Mug C1 (yellow mug)
    add_physics_object(
        stage,
        f"{ISAAC_ASSETS_URL}/Props/Mugs/SM_Mug_C1.usd",
        "/World/Mug_C1",
        position=(0.5, -0.25, 0.02),
        mass=0.15,
        static_friction=0.8,
        dynamic_friction=0.6
    )
    
    # Small KLT Container (blue bin)
    add_physics_object(
        stage,
        f"{ISAAC_ASSETS_URL}/Props/KLT_Bin/small_KLT.usd",
        "/World/small_KLT",
        position=(0.7, 0.0, 0.02),
        mass=0.3,
        static_friction=0.6,
        dynamic_friction=0.4
    )
    
    # Nvidia Cube
    add_physics_object(
        stage,
        f"{ISAAC_ASSETS_URL}/Props/Blocks/nvidia_cube.usd",
        "/World/nvidia_cube",
        position=(0.45, 0.0, 0.02),
        scale=0.5,
        mass=0.08,
        static_friction=0.7,
        dynamic_friction=0.5
    )
    
    # -------------------------------------------------------------------------
    # CAMERA (Parented under Hand)
    # -------------------------------------------------------------------------
    # To avoid "Closed Articulation" errors, we parent the camera directly to the hand link.
    # It will not be a physics body, just a visual object moving with the hand.
    
    hand_prim_path = "/World/Franka/panda_hand"
    gripper_cam_path = f"{hand_prim_path}/gripper_camera"
    
    gripper_cam = UsdGeom.Camera.Define(stage, gripper_cam_path)
    gripper_cam.GetFocalLengthAttr().Set(18.0)
    gripper_cam.GetFocusDistanceAttr().Set(0.1)
    
    # Apply Transform (Position & Orientation relative to Hand)
    # Pos: [0.1, 0.0, -0.15]
    # Rot: [0, 180, -90] (Euler)
    
    xform = UsdGeom.Xformable(gripper_cam)
    xform.ClearXformOpOrder() # Clear default ops
    
    # 1. Translate
    xform.AddTranslateOp().Set(Gf.Vec3d(0.1, 0.0, -0.15))
    
    # 2. Rotate
    # Create rotation op. standard is RotationOrder usually XYZ or similar, but let's be explicit.
    # We can use AddRotateXYZOp.
    # Rot: [0, 180, -90]
    xform.AddRotateXYZOp().Set(Gf.Vec3d(0.0, 180.0, -90.0))
    
    print(f"[Setup] Camera created at {gripper_cam_path}")
    print(f"[Setup] Parented to {hand_prim_path}")
    
    # Save Stage
    save_path = "/home/saurabh/Development/quest3_streamer/environment.usd"
    save_stage(save_path)
    print(f"SUCCESS: Stage saved to {save_path}")
    print("INSTRUCTIONS:")
    print("1. Open Isaac Sim")
    print(f"2. File -> Open -> {save_path}")
    print("3. Select '/World/gripper_camera' or the Joint and adjust Position as needed.")
    print("4. File -> Save")
    print("5. Run './python.sh isaac_teleop_usd.py'")
    
    simulation_app.close()

if __name__ == "__main__":
    main()
