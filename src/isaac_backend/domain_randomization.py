from __future__ import annotations

import colorsys
from dataclasses import dataclass, field
import math
import random
from typing import Any


def get_world_pose_usd(prim) -> tuple[list[float], list[float]]:
    """Read a prim's world position and orientation directly from USD."""
    from pxr import UsdGeom

    transform = UsdGeom.XformCache(0.0).GetLocalToWorldTransform(prim)
    translation = transform.ExtractTranslation()
    rotation = transform.ExtractRotationQuat()
    imaginary = rotation.GetImaginary()
    return (
        [float(translation[0]), float(translation[1]), float(translation[2])],
        [float(rotation.GetReal()), float(imaginary[0]), float(imaginary[1]), float(imaginary[2])],
    )


def set_world_pose_usd(prim, position, orientation_wxyz) -> None:
    """Set a prim's world pose directly via USD."""
    from pxr import Gf, UsdGeom

    api = UsdGeom.XformCommonAPI(prim)
    api.SetTranslate(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))
    roll, pitch, yaw = _quat_wxyz_to_euler_xyz_degrees(orientation_wxyz)
    api.SetRotate((roll, pitch, yaw), UsdGeom.XformCommonAPI.RotationOrderXYZ)


def _quat_wxyz_to_euler_xyz_degrees(quat_wxyz) -> tuple[float, float, float]:
    """Convert a [w, x, y, z] quaternion to XYZ euler angles in degrees."""
    qw, qx, qy, qz = [float(v) for v in quat_wxyz]
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm < 1e-12:
        return 0.0, 0.0, 0.0
    qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (
        math.degrees(roll),
        math.degrees(pitch),
        math.degrees(yaw),
    )

def _euler_xyz_degrees_to_quat_wxyz(rotation_degrees) -> tuple[float, float, float, float]:
    roll, pitch, yaw = [math.radians(float(value)) for value in rotation_degrees]
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )

def _quat_multiply_wxyz(left, right) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = [float(value) for value in left]
    rw, rx, ry, rz = [float(value) for value in right]
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


@dataclass
class DomainRandomizationSample:
    light_intensity: float | None = None
    floor_color: tuple[float, float, float] | None = None
    missing_prims: tuple[str, ...] = ()
    # Maps USD prim path -> {"position": [...], "orientation": [...]} for all randomized objects
    tray_poses: dict = field(default_factory=dict)
    # Generic lights states map: prim_path -> {"intensity": float, "exposure": float, "color_temperature": float}
    light_states: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        """Serialize this sample to a plain JSON-compatible dict."""
        return {
            "type": "DomainRandomizationSample",
            "light_intensity": self.light_intensity,
            "floor_color": list(self.floor_color) if self.floor_color is not None else None,
            "missing_prims": list(self.missing_prims),
            "tray_poses": {
                path: {"position": list(pose["position"]), "orientation": list(pose["orientation"])}
                for path, pose in self.tray_poses.items()
            },
            "light_states": self.light_states,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "DomainRandomizationSample":
        """Reconstruct a sample from a plain dict (deserialized from JSON)."""
        floor_color = d.get("floor_color")
        tray_poses_raw = d.get("tray_poses", {})
        return cls(
            light_intensity=(
                float(d["light_intensity"]) if d.get("light_intensity") is not None else None
            ),
            floor_color=(
                tuple(float(v) for v in floor_color[:3]) if floor_color is not None else None
            ),
            missing_prims=tuple(d.get("missing_prims", [])),
            tray_poses={
                path: {
                    "position": [float(v) for v in pose["position"]],
                    "orientation": [float(v) for v in pose["orientation"]],
                }
                for path, pose in tray_poses_raw.items()
            },
            light_states=d.get("light_states", {}),
        )


class DomainRandomizer:
    def __init__(self, stage, config: dict[str, Any] | None):
        self.stage = stage
        self.config = config or {}
        self.enabled = bool(self.config.get("enabled", False))
        self.rng = random.Random(self.config.get("seed"))
        
        self._base_object_poses: dict[str, tuple[list[float], list[float]]] = {}
        self._base_lights: dict[str, dict[str, Any]] = {}
        self._base_scales: dict[str, tuple[float, float, float]] = {}
        
        self.randomized_objects = []
        self.randomized_lights = []
        self._corner_bounds = None
        self._last_randomized_poses = {}

    def initialize(self) -> None:
        if not self.enabled:
            return
            
        print("[DR] Traversing stage to search for corner and island prims...")
        for prim in self.stage.Traverse():
            name = prim.GetName()
            if "corner" in name.lower() or "island" in name.lower():
                print(f"[DR] Diagnostic: found prim path='{prim.GetPath()}', type='{prim.GetTypeName()}'")

        self.randomized_objects = self.config.get("objects", {}).get("items", [])
        if self.randomized_objects:
            for item in self.randomized_objects:
                path = item.get("path")
                prim = self.stage.GetPrimAtPath(path)
                if prim.IsValid():
                    self._base_object_poses[path] = self._get_world_pose(prim)
                    self._base_scales[path] = self._get_prim_scale_usd(prim)

        # Fetch coordinates of corners in runtime to construct the bounding box
        corners = []
        for i in range(1, 10):
            corner_path = f"/World/corner_{i:02d}"
            prim = self.stage.GetPrimAtPath(corner_path)
            if prim.IsValid():
                pos, _ = self._get_world_pose(prim)
                corners.append(pos)
        for i in range(1, 10):
            corner_path = f"/World/corner_{i}"
            prim = self.stage.GetPrimAtPath(corner_path)
            if prim.IsValid():
                pos, _ = self._get_world_pose(prim)
                corners.append(pos)

        if corners:
            x_coords = [c[0] for c in corners]
            y_coords = [c[1] for c in corners]
            x_min = min(x_coords)
            x_max = max(x_coords)
            y_min = min(y_coords)
            y_max = max(y_coords)
            self._corner_bounds = (x_min, x_max, y_min, y_max)
            print(f"[DR] Detected {len(corners)} corner prims. Dynamic bounds: x=[{x_min}, {x_max}], y=[{y_min}, {y_max}]")
        else:
            self._corner_bounds = None

        self.randomized_lights = self.config.get("lighting", {}).get("lights", [])
        if self.randomized_lights:
            self._capture_generic_light_baselines()
            
        self._ensure_floor_material()

    def randomize(self, settle_step) -> DomainRandomizationSample:
        sample = self.apply_randomization()
        self.settle(settle_step)
        return sample

    def settle(self, settle_step) -> None:
        if not self.enabled:
            return
            
        # Re-apply the randomized object poses after reset_world() to ensure they are at the correct coordinates
        # even if PhysX has cached states.
        last_poses = getattr(self, "_last_randomized_poses", None)
        if last_poses:
            print(f"[DR] settle: Re-applying {len(last_poses)} randomized object poses after reset_world()")
            for path, pose_data in last_poses.items():
                prim = self.stage.GetPrimAtPath(path)
                if prim.IsValid():
                    self._set_world_pose(prim, pose_data["position"], pose_data["orientation"])
                    
        for _ in range(max(0, int(self.config.get("settle_steps", 30)))):
            settle_step(render=True)

    def apply_randomization(self) -> DomainRandomizationSample:
        """Mutate the USD stage for a new random sample without stepping physics."""
        sample = DomainRandomizationSample()
        if not self.enabled:
            return sample

        intensity, light_states = self._randomize_generic_lighting()
        sample.light_intensity = intensity
        sample.light_states = light_states
        
        sample.floor_color = self._randomize_floor()
        
        tray_poses, missing = self._randomize_generic_objects()
        sample.tray_poses = tray_poses
        sample.missing_prims = tuple(missing)
        
        self._last_randomized_poses = tray_poses
        return sample

    def apply_randomization_from_dict(self, sample_dict: dict) -> None:
        """Deterministically reconstruct a previously saved domain-randomization state."""
        if not self.enabled:
            return
            
        self._last_randomized_poses = sample_dict.get("tray_poses", {})
        
        self._apply_saved_lighting(sample_dict)
        self._apply_saved_floor(sample_dict)
        
        tray_poses = sample_dict.get("tray_poses", {})
        for path, pose_data in tray_poses.items():
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue
            position = pose_data["position"]
            orientation = pose_data["orientation"]
            self._set_world_pose(prim, position, orientation)

    def _apply_saved_lighting(self, sample_dict: dict) -> None:
        """Restore the saved light intensity to all configured generic lights."""
        from pxr import UsdLux
        
        light_states = sample_dict.get("light_states", {})
        for path, state in light_states.items():
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue
            light = UsdLux.LightAPI(prim)
            if "intensity" in state:
                light.CreateIntensityAttr(float(state["intensity"]))
            if "exposure" in state:
                light.CreateExposureAttr(float(state["exposure"]))
            if "color_temperature" in state:
                self._set_light_color_temperature(prim, float(state["color_temperature"]))

    def _apply_saved_floor(self, sample_dict: dict) -> None:
        """Restore the saved floor color."""
        floor_color = sample_dict.get("floor_color")
        if floor_color is None:
            return
        from pxr import Gf, Sdf, UsdShade

        color = tuple(float(v) for v in floor_color[:3])
        looks_path = str(self.config.get("prims", {}).get("looks", "/World/Looks"))
        shader = UsdShade.Shader.Get(self.stage, f"{looks_path}/randomized_floor_material/PreviewSurface")
        if shader:
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        self._apply_floor_display_color(color)

    def _capture_generic_light_baselines(self) -> None:
        from pxr import UsdLux
        self._base_lights = {}
        for item in self.randomized_lights:
            path = item.get("path")
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                print(f"[DR] Warning: generic light prim not found at {path}")
                continue
            light = UsdLux.LightAPI(prim)
            position, orientation = self._get_world_pose(prim)
            local_orient = self._get_local_orientation_wxyz(prim) or orientation
            intensity = light.GetIntensityAttr().Get()
            exposure = light.GetExposureAttr().Get()
            
            enable_temp = False
            color_temp = 6500.0
            if prim.HasAttribute("inputs:enableColorTemperature"):
                val = prim.GetAttribute("inputs:enableColorTemperature").Get()
                if val is not None:
                    enable_temp = bool(val)
            if prim.HasAttribute("inputs:colorTemperature"):
                val = prim.GetAttribute("inputs:colorTemperature").Get()
                if val is not None:
                    color_temp = float(val)
                
            self._base_lights[path] = {
                "position": position,
                "orientation": local_orient,
                "intensity": float(intensity) if intensity is not None else 1000.0,
                "exposure": float(exposure) if exposure is not None else 0.0,
                "enable_color_temperature": enable_temp,
                "color_temperature": color_temp,
            }

    def _randomize_generic_lighting(self) -> tuple[float, dict]:
        from pxr import UsdLux
        
        light_states = {}
        total_intensity = 0.0
        valid_count = 0
        
        for item in self.randomized_lights:
            path = item.get("path")
            if path not in self._base_lights:
                continue
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue
                
            intensity_range = item.get("intensity", [1000.0, 5000.0])
            intensity = self.rng.uniform(float(intensity_range[0]), float(intensity_range[1]))
            
            base_exposure = self._base_lights[path]["exposure"]
            exposure_jitter = item.get("exposure_jitter", [0.0, 0.0])
            exposure = float(base_exposure) + self.rng.uniform(float(exposure_jitter[0]), float(exposure_jitter[1]))
            
            light = UsdLux.LightAPI(prim)
            light.CreateIntensityAttr(float(intensity))
            light.CreateExposureAttr(float(exposure))
            
            light_state = {
                "intensity": intensity,
                "exposure": exposure
            }
            
            temp_range = item.get("color_temperature")
            if temp_range:
                color_temp = self.rng.uniform(float(temp_range[0]), float(temp_range[1]))
                self._set_light_color_temperature(prim, color_temp)
                light_state["color_temperature"] = color_temp
                
            light_states[path] = light_state
            total_intensity += intensity
            valid_count += 1
            
        avg_intensity = (total_intensity / valid_count) if valid_count > 0 else 0.0
        return avg_intensity, light_states

    def _set_light_color_temperature(self, prim, temp_kelvin: float) -> None:
        try:
            from pxr import Sdf, UsdLux
            light = UsdLux.LightAPI(prim)
            if hasattr(light, "CreateEnableColorTemperatureAttr"):
                light.CreateEnableColorTemperatureAttr().Set(True)
                light.CreateColorTemperatureAttr().Set(float(temp_kelvin))
            else:
                if prim.HasAttribute("inputs:enableColorTemperature"):
                    prim.GetAttribute("inputs:enableColorTemperature").Set(True)
                else:
                    prim.CreateAttribute("inputs:enableColorTemperature", Sdf.ValueTypeNames.Bool).Set(True)
                
                if prim.HasAttribute("inputs:colorTemperature"):
                    prim.GetAttribute("inputs:colorTemperature").Set(float(temp_kelvin))
                else:
                    prim.CreateAttribute("inputs:colorTemperature", Sdf.ValueTypeNames.Float).Set(float(temp_kelvin))
        except Exception as e:
            print(f"[DR] Warning: could not set color temperature on {prim.GetPath()}: {e}")

    def _get_placement_bounds(self) -> tuple[float, float, float, float]:
        if getattr(self, "_corner_bounds", None) is not None:
            return self._corner_bounds
            
        placement_config = self.config.get("objects", {}).get("bounds", {})
        
        if "explicit" in placement_config:
            explicit = placement_config["explicit"]
            return (
                float(explicit["x"][0]),
                float(explicit["x"][1]),
                float(explicit["y"][0]),
                float(explicit["y"][1]),
            )
            
        ref_prim_path = placement_config.get("reference_prim")
        if ref_prim_path:
            from pxr import UsdGeom
            table = self.stage.GetPrimAtPath(ref_prim_path)
            if table.IsValid():
                bbox_cache = UsdGeom.BBoxCache(
                    0.0,
                    [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
                )
                box = bbox_cache.ComputeWorldBound(table).ComputeAlignedBox()
                min_p = box.GetMin()
                max_p = box.GetMax()
                values = [float(min_p[0]), float(max_p[0]), float(min_p[1]), float(max_p[1])]
                if all(math.isfinite(value) for value in values):
                    scale = placement_config.get("center_area_scale", [0.6, 0.6])
                    x_scale = max(0.05, min(1.0, float(scale[0])))
                    y_scale = max(0.05, min(1.0, float(scale[1])))
                    x_min, x_max, y_min, y_max = values
                    center_x = (x_min + x_max) * 0.5
                    center_y = (y_min + y_max) * 0.5
                    half_x = (x_max - x_min) * x_scale * 0.5
                    half_y = (y_max - y_min) * y_scale * 0.5
                    return center_x - half_x, center_x + half_x, center_y - half_y, center_y + half_y
                    
        fallback = placement_config.get("fallback_table_bounds_m", {"x": [-0.35, 0.35], "y": [-0.28, 0.28]})
        return (
            float(fallback["x"][0]),
            float(fallback["x"][1]),
            float(fallback["y"][0]),
            float(fallback["y"][1]),
        )

    def _randomize_generic_objects(self) -> tuple[dict, list[str]]:
        missing = []
        valid_items = []
        for item in self.randomized_objects:
            path = item.get("path")
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                missing.append(path)
            else:
                valid_items.append((prim, item))
                
        if missing:
            return {}, missing
            
        bounds = self._get_placement_bounds()
        min_distance = float(self.config.get("objects", {}).get("min_distance_m", 0.18))
        
        placed_poses = {}
        placed_xy = []
        
        for prim, item in valid_items:
            path = str(prim.GetPath())
            radius = float(item.get("radius_m", 0.05))
            yaw_range = item.get("yaw_deg", [-180.0, 180.0])
            z_offset = float(item.get("z_offset_m", 0.0))
            
            sampled_xy = self._sample_generic_separated_xy(bounds, radius, placed_xy, min_distance)
            
            base_position, base_orientation = self._base_object_poses.get(path, self._get_world_pose(prim))
            yaw = self.rng.uniform(float(yaw_range[0]), float(yaw_range[1]))
            
            position = [float(sampled_xy[0]), float(sampled_xy[1]), float(base_position[2]) + z_offset]
            orientation = _quat_multiply_wxyz(
                base_orientation,
                _euler_xyz_degrees_to_quat_wxyz((0.0, 0.0, yaw)),
            )
            
            self._set_world_pose(prim, position, orientation)
            placed_poses[path] = {"position": position, "orientation": orientation}
            placed_xy.append((sampled_xy[0], sampled_xy[1], radius))
            
        return placed_poses, []

    def _sample_generic_separated_xy(
        self,
        bounds: tuple[float, float, float, float],
        radius: float,
        placed_xy: list[tuple[float, float, float]],
        min_distance: float,
    ) -> tuple[float, float]:
        for _ in range(200):
            xy = self._sample_xy(bounds, radius)
            if all(math.hypot(xy[0] - px, xy[1] - py) >= max(min_distance, radius + pr) for px, py, pr in placed_xy):
                return xy
                
        candidates = [self._sample_xy(bounds, radius) for _ in range(50)]
        if not placed_xy:
            return candidates[0]
            
        return max(
            candidates,
            key=lambda xy: min(math.hypot(xy[0] - px, xy[1] - py) - (radius + pr) for px, py, pr in placed_xy)
        )

    def _sample_xy(self, bounds: tuple[float, float, float, float], radius: float) -> tuple[float, float]:
        x_min, x_max, y_min, y_max = bounds
        if x_min + radius >= x_max - radius:
            x = (x_min + x_max) * 0.5
        else:
            x = self.rng.uniform(x_min + radius, x_max - radius)
        if y_min + radius >= y_max - radius:
            y = (y_min + y_max) * 0.5
        else:
            y = self.rng.uniform(y_min + radius, y_max - radius)
        return x, y

    def _ensure_floor_material(self) -> None:
        from pxr import Sdf, Usd, UsdGeom, UsdShade

        looks_path = str(self.config.get("prims", {}).get("looks", "/World/Looks"))
        if not self.stage.GetPrimAtPath(looks_path).IsValid():
            self.stage.DefinePrim(looks_path, "Scope")
            
        material_path = f"{looks_path}/randomized_floor_material"
        if not self.stage.GetPrimAtPath(material_path).IsValid():
            material = UsdShade.Material.Define(self.stage, material_path)
            shader = UsdShade.Shader.Define(self.stage, f"{material_path}/PreviewSurface")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

            floor_path = self.config.get("prims", {}).get("floor")
            floor = self.stage.GetPrimAtPath(str(floor_path)) if floor_path else None
            if floor and floor.IsValid():
                for prim in Usd.PrimRange(floor):
                    if prim == floor or prim.IsA(UsdGeom.Gprim):
                        UsdShade.MaterialBindingAPI(prim).Bind(material)

    def _randomize_floor(self) -> tuple[float, float, float] | None:
        from pxr import Gf, Sdf, UsdShade

        color = self._sample_floor_color()
        looks_path = str(self.config.get("prims", {}).get("looks", "/World/Looks"))
        shader = UsdShade.Shader.Get(self.stage, f"{looks_path}/randomized_floor_material/PreviewSurface")
        if not shader:
            return None
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        self._apply_floor_display_color(color)
        return color

    def _sample_floor_color(self) -> tuple[float, float, float]:
        colors = self.config.get("floor", {}).get("colors", [])
        if colors:
            return tuple(float(v) for v in self.rng.choice(colors))

        floor_families = [
            (0.30, ((0.0, 360.0), (0.0, 0.05), (0.30, 0.70))),
            (0.12, ((30.0, 60.0), (0.05, 0.15), (0.75, 0.90))),
            (0.12, ((25.0, 45.0), (0.10, 0.25), (0.40, 0.65))),
            (0.11, ((0.0, 360.0), (0.0, 0.05), (0.10, 0.25))),
            (0.07, ((200.0, 220.0), (0.10, 0.25), (0.35, 0.55))),
            (0.06, ((80.0, 120.0), (0.10, 0.20), (0.30, 0.50))),
            (0.06, ((15.0, 35.0), (0.15, 0.30), (0.25, 0.45))),
            (0.06, ((210.0, 240.0), (0.05, 0.15), (0.45, 0.65))),
            (0.025, ((170.0, 190.0), (0.15, 0.30), (0.30, 0.50))),
            (0.025, ((340.0, 360.0), (0.10, 0.25), (0.40, 0.55))),
            (0.025, ((40.0, 55.0), (0.20, 0.35), (0.40, 0.55))),
            (0.025, ((220.0, 240.0), (0.15, 0.30), (0.15, 0.30))),
        ]
        _, (h_range, s_range, l_range) = self.rng.choices(
            floor_families,
            weights=[weight for weight, _ in floor_families],
            k=1,
        )[0]
        hue = self.rng.uniform(*h_range) / 360.0
        saturation = self.rng.uniform(*s_range)
        lightness = self.rng.uniform(*l_range)
        return colorsys.hls_to_rgb(hue, lightness, saturation)

    def _apply_floor_display_color(self, color: tuple[float, float, float]) -> None:
        from pxr import Gf, Usd, UsdGeom

        floor_path = self.config.get("prims", {}).get("floor")
        floor = self.stage.GetPrimAtPath(str(floor_path)) if floor_path else None
        if not floor or not floor.IsValid():
            return

        color_value = [Gf.Vec3f(*color)]
        for prim in Usd.PrimRange(floor):
            if prim.IsA(UsdGeom.Gprim):
                gprim = UsdGeom.Gprim(prim)
                gprim.CreateDisplayColorAttr().Set(color_value)

    def _get_prim_scale_usd(self, prim) -> tuple[float, float, float]:
        from pxr import UsdGeom
        xformable = UsdGeom.Xformable(prim)
        scale = (1.0, 1.0, 1.0)
        try:
            for op in xformable.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                    scale_val = op.Get()
                    if scale_val is not None:
                        scale = (float(scale_val[0]), float(scale_val[1]), float(scale_val[2]))
                    break
        except Exception:
            pass
        return scale

    def _get_local_orientation_wxyz(self, prim) -> tuple[float, float, float, float] | None:
        try:
            from pxr import UsdGeom

            translate, rotate, scale, _, _ = UsdGeom.XformCommonAPI(prim).GetXformVectors(0.0)
            if rotate:
                return _euler_xyz_degrees_to_quat_wxyz(rotate)
        except Exception:
            pass
        return None

    def _get_world_pose(self, prim) -> tuple[list[float], list[float]]:
        return get_world_pose_usd(prim)

    def _set_world_pose(self, prim, position, orientation) -> None:
        """Sets a prim's world pose cleanly using Pixar USD Xformable API.

        Clears existing conflicting transform operations (such as 4x4 matrices,
        different euler conventions, or conflicting rotation orders) directly
        on the USD prim, and sets clean translate and quaternion orient operations.
        This preserves the original scale of the prim.
        """
        from pxr import Gf, UsdGeom
        
        path = str(prim.GetPath())
        xformable = UsdGeom.Xformable(prim)
        
        scale = None
        if self.randomized_objects:
            for item in self.randomized_objects:
                if item.get("path") == path and "scale" in item:
                    scale_val = item["scale"]
                    if isinstance(scale_val, list | tuple) and len(scale_val) >= 3:
                        scale = (float(scale_val[0]), float(scale_val[1]), float(scale_val[2]))
                    elif isinstance(scale_val, int | float):
                        scale = (float(scale_val), float(scale_val), float(scale_val))
                    break
        
        if scale is None:
            scale = getattr(self, "_base_scales", {}).get(path)
            
        if scale is None:
            scale = self._get_prim_scale_usd(prim)
            
        try:
            xformable.ClearXformOpOrder()
            
            translate_op = xformable.AddTranslateOp()
            t_type = str(translate_op.GetAttr().GetTypeName())
            if t_type == "float3":
                translate_op.Set(Gf.Vec3f(float(position[0]), float(position[1]), float(position[2])))
            else:
                translate_op.Set(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))
            
            w, x, y, z = [float(v) for v in orientation]
            orient_op = xformable.AddOrientOp()
            o_type = str(orient_op.GetAttr().GetTypeName())
            if o_type == "quatf":
                orient_op.Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))
            else:
                orient_op.Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
            
            scale_op = xformable.AddScaleOp()
            s_type = str(scale_op.GetAttr().GetTypeName())
            if s_type == "double3":
                scale_op.Set(Gf.Vec3d(*scale))
            else:
                scale_op.Set(Gf.Vec3f(*scale))
            
        except Exception as e:
            print(f"[DR] Error setting clean USD pose for {path}: {e}")
            set_world_pose_usd(prim, position, orientation)
