from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any

from .domain_randomization import (
    DomainRandomizationSample,
    DomainRandomizer,
    _euler_xyz_degrees_to_quat_wxyz,
    _quat_multiply_wxyz,
)


@dataclass
class FFWBG2DomainRandomizationSample(DomainRandomizationSample):
    cube_pose: tuple[list[float], tuple[float, float, float, float]] | None = None
    tray_pose: tuple[list[float], tuple[float, float, float, float]] | None = None
    missing_prims: tuple[str, ...] = ()
    # Sampled directional light orientation (wxyz quaternion) for faithful replay.
    light_orientation: list[float] | None = None
    # Generic lights states map: prim_path -> {"intensity": float, "exposure": float, "color": [r,g,b]}
    light_states: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        """Serialize to a JSON-compatible dict including cube/tray poses and light direction."""
        d = super().to_dict()
        d["type"] = "FFWBG2DomainRandomizationSample"
        d["missing_prims"] = list(self.missing_prims)
        d["light_orientation"] = list(self.light_orientation) if self.light_orientation is not None else None
        d["cube_pose"] = (
            {"position": list(self.cube_pose[0]), "orientation": list(self.cube_pose[1])}
            if self.cube_pose is not None else None
        )
        d["tray_pose"] = (
            {"position": list(self.tray_pose[0]), "orientation": list(self.tray_pose[1])}
            if self.tray_pose is not None else None
        )
        d["light_states"] = self.light_states
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "FFWBG2DomainRandomizationSample":
        """Reconstruct from a plain dict deserialized from JSON."""
        base = DomainRandomizationSample.from_dict(d)
        cube_raw = d.get("cube_pose")
        tray_raw = d.get("tray_pose")
        lo = d.get("light_orientation")
        return cls(
            nut_count=base.nut_count,
            bolt_count=base.bolt_count,
            light_intensity=base.light_intensity,
            floor_color=base.floor_color,
            tray_poses=base.tray_poses,
            missing_prims=tuple(d.get("missing_prims", [])),
            light_orientation=[float(v) for v in lo] if lo is not None else None,
            cube_pose=(
                ([float(v) for v in cube_raw["position"]], tuple(float(v) for v in cube_raw["orientation"]))
                if cube_raw is not None else None
            ),
            tray_pose=(
                ([float(v) for v in tray_raw["position"]], tuple(float(v) for v in tray_raw["orientation"]))
                if tray_raw is not None else None
            ),
            light_states=d.get("light_states", {}),
        )


class FFWBG2DomainRandomizer(DomainRandomizer):
    def __init__(self, stage, config: dict[str, Any] | None):
        super().__init__(stage, config)
        self._base_object_poses: dict[str, tuple[list[float], list[float]]] = {}
        self._base_lights: dict[str, dict[str, Any]] = {}
        self.randomized_objects = []
        self.randomized_lights = []
        self._corner_bounds = None

    def initialize(self) -> None:
        if not self.enabled:
            return

        # Traversal diagnostic to locate all corner and island prims on the stage
        print("[DR] Traversing stage to search for corner and island prims...")
        for prim in self.stage.Traverse():
            name = prim.GetName()
            if "corner" in name.lower() or "island" in name.lower():
                print(f"[DR] Diagnostic: found prim path='{prim.GetPath()}', type='{prim.GetTypeName()}'")

        # 1. Parse generic randomized objects
        self.randomized_objects = self.config.get("objects", {}).get("items", [])
        self._base_scales = {}
        if self.randomized_objects:
            for item in self.randomized_objects:
                path = item.get("path")
                prim = self.stage.GetPrimAtPath(path)
                if prim.IsValid():
                    self._base_object_poses[path] = self._get_world_pose(prim)
                    self._base_scales[path] = self._get_prim_scale_usd(prim)
        else:
            # Fallback to old behavior: cube and tray
            for path in (self._cube_path(), self._tray_path()):
                prim = self.stage.GetPrimAtPath(path)
                if prim.IsValid():
                    self._base_object_poses[path] = self._get_world_pose(prim)

        # 2. Fetch coordinates of corners in runtime to construct the bounding box
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

        # 3. Parse generic randomized lights
        self.randomized_lights = self.config.get("lighting", {}).get("lights", [])
        if self.randomized_lights:
            self._capture_generic_light_baselines()
        else:
            self._capture_light_baselines()

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

    def randomize(self, settle_step) -> FFWBG2DomainRandomizationSample:
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

    def apply_randomization(self) -> FFWBG2DomainRandomizationSample:
        sample = FFWBG2DomainRandomizationSample()
        if not self.enabled:
            return sample

        # Check if generic config is used
        if self.randomized_objects:
            # Generic configuration-driven randomization
            intensity, light_states = self._randomize_generic_lighting()
            sample.light_intensity = intensity
            sample.light_states = light_states
            
            # Place the randomized objects
            tray_poses, missing = self._randomize_generic_objects()
            sample.tray_poses = tray_poses
            sample.missing_prims = tuple(missing)
            
            # Save poses for settle() override
            self._last_randomized_poses = tray_poses
            
            # For backward compatibility, also populate tray_pose and cube_pose if they exist in the items
            cube_path = self._cube_path()
            tray_path = self._tray_path()
            if cube_path in tray_poses:
                sample.cube_pose = (tray_poses[cube_path]["position"], tray_poses[cube_path]["orientation"])
            if tray_path in tray_poses:
                sample.tray_pose = (tray_poses[tray_path]["position"], tray_poses[tray_path]["orientation"])
        else:
            # Fallback to the legacy cube and tray behavior
            intensity, orientation = self._randomize_lighting()
            sample.light_intensity = intensity
            sample.light_orientation = list(orientation) if orientation is not None else None
            sample.tray_pose, sample.cube_pose, sample.missing_prims = self._randomize_cube_and_tray()
            sample.tray_poses = self._capture_tray_poses()
            
        return sample

    def apply_randomization_from_dict(self, sample_dict: dict) -> None:
        super().apply_randomization_from_dict(sample_dict)
        self._last_randomized_poses = sample_dict.get("tray_poses", {})

    def _apply_saved_object_poses(self, sample_dict: dict) -> None:
        """Restore cube and tray to their recorded poses for deferred rendering."""
        cube_raw = sample_dict.get("cube_pose")
        if cube_raw is not None:
            prim = self.stage.GetPrimAtPath(self._cube_path())
            if prim.IsValid():
                self._set_world_pose(prim, cube_raw["position"], cube_raw["orientation"])

        tray_raw = sample_dict.get("tray_pose")
        if tray_raw is not None:
            prim = self.stage.GetPrimAtPath(self._tray_path())
            if prim.IsValid():
                self._set_world_pose(prim, tray_raw["position"], tray_raw["orientation"])

        # Also restore any generic tray poses captured by the parent.
        super()._apply_saved_object_poses(sample_dict)

    def _apply_saved_lighting(self, sample_dict: dict) -> None:
        """Restore the saved directional light orientation and intensity."""
        from pxr import UsdLux

        # Check if we have generic light states saved
        light_states = sample_dict.get("light_states")
        if light_states:
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
            return

        intensity = sample_dict.get("light_intensity")
        if intensity is None:
            return

        # Restore using the captured base_lights dict so orientation is preserved.
        if not self._base_lights:
            self._capture_light_baselines()

        light_orientation = sample_dict.get("light_orientation")
        for path in self._base_lights:
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue
            UsdLux.LightAPI(prim).CreateIntensityAttr(float(intensity))
            if light_orientation is not None:
                self._set_light_pose(prim, self._base_lights[path]["position"], light_orientation)

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
            
            # Read color temperature if it exists
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
            
            # Exposure jitter/range
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
            
            # Color temperature
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
        
        # 1. Try explicit bounds
        if "explicit" in placement_config:
            explicit = placement_config["explicit"]
            return (
                float(explicit["x"][0]),
                float(explicit["x"][1]),
                float(explicit["y"][0]),
                float(explicit["y"][1]),
            )
            
        # 2. Try reference prim bounding box
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
                    # Scale center area
                    scale = placement_config.get("center_area_scale", [0.6, 0.6])
                    x_scale = max(0.05, min(1.0, float(scale[0])))
                    y_scale = max(0.05, min(1.0, float(scale[1])))
                    x_min, x_max, y_min, y_max = values
                    center_x = (x_min + x_max) * 0.5
                    center_y = (y_min + y_max) * 0.5
                    half_x = (x_max - x_min) * x_scale * 0.5
                    half_y = (y_max - y_min) * y_scale * 0.5
                    return center_x - half_x, center_x + half_x, center_y - half_y, center_y + half_y
                    
        # 3. Fallback table bounds
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
                
        print(f"[DR] _randomize_generic_objects: items={self.randomized_objects}, valid_items={[str(v[0].GetPath()) for v in valid_items]}, missing={missing}")
        if missing:
            return {}, missing
            
        bounds = self._get_placement_bounds()
        print(f"[DR] Using bounds: {bounds}")
        min_distance = float(self.config.get("objects", {}).get("min_distance_m", 0.18))
        
        placed_poses = {}
        placed_xy = []
        
        for prim, item in valid_items:
            path = str(prim.GetPath())
            radius = float(item.get("radius_m", 0.05))
            yaw_range = item.get("yaw_deg", [-180.0, 180.0])
            z_offset = float(item.get("z_offset_m", 0.0))
            
            # Sample position with spacing check
            sampled_xy = self._sample_generic_separated_xy(bounds, radius, placed_xy, min_distance)
            
            # Get base position/orientation
            base_position, base_orientation = self._base_object_poses.get(path, self._get_world_pose(prim))
            yaw = self.rng.uniform(float(yaw_range[0]), float(yaw_range[1]))
            
            position = [float(sampled_xy[0]), float(sampled_xy[1]), float(base_position[2]) + z_offset]
            orientation = _quat_multiply_wxyz(
                base_orientation,
                _euler_xyz_degrees_to_quat_wxyz((0.0, 0.0, yaw)),
            )
            
            print(f"[DR] Setting pose of {path} to position={position}, orientation={orientation}")
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
                
        # Fallback if sampling fails: sample candidates and select the one with maximum minimum distance
        candidates = [self._sample_xy(bounds, radius) for _ in range(50)]
        if not placed_xy:
            return candidates[0]
            
        return max(
            candidates,
            key=lambda xy: min(math.hypot(xy[0] - px, xy[1] - py) - (radius + pr) for px, py, pr in placed_xy)
        )

    def _set_world_pose(self, prim, position, orientation) -> None:
        """Sets a prim's world pose cleanly using Pixar USD Xformable API.

        Clears existing conflicting transform operations (such as 4x4 matrices,
        different euler conventions, or conflicting rotation orders) directly
        on the USD prim, and sets clean translate and quaternion orient operations.
        This preserves the original scale of the prim (including any runtime scale
        overrides or cached baseline scales), and works in standard Python
        environments (e.g. standalone teleop/ROS launchers) where 'omni.isaac.core'
        is not installed.
        """
        from pxr import Gf, UsdGeom
        
        path = str(prim.GetPath())
        xformable = UsdGeom.Xformable(prim)
        
        # 1. Retrieve the target scale (prioritize config overrides, then cached original scale)
        scale = None
        
        # Check config items for an explicit scale override
        if self.randomized_objects:
            for item in self.randomized_objects:
                if item.get("path") == path and "scale" in item:
                    scale_val = item["scale"]
                    if isinstance(scale_val, list | tuple) and len(scale_val) >= 3:
                        scale = (float(scale_val[0]), float(scale_val[1]), float(scale_val[2]))
                    elif isinstance(scale_val, int | float):
                        scale = (float(scale_val), float(scale_val), float(scale_val))
                    break
        
        # Fall back to baseline cache if config has no override
        if scale is None:
            scale = getattr(self, "_base_scales", {}).get(path)
            
        # If not cached, query it directly from USD
        if scale is None:
            scale = self._get_prim_scale_usd(prim)
            
        # 2. Clear xform op order to prevent conflicts with pre-existing xform ops/matrices
        try:
            xformable.ClearXformOpOrder()
            
            # 3. Add clean translate, orient, and scale ops, matching attribute type precision
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
            
            print(f"[DR] Successfully set clean USD pose for {path} (position={position}, scale={scale})")
        except Exception as e:
            print(f"[DR] Error setting clean USD pose for {path}: {e}")
            # Fallback to standard USD transform setting
            super()._set_world_pose(prim, position, orientation)

    def _cube_path(self) -> str:
        return str(self.config.get("prims", {}).get("cube", "/World/Cube"))

    def _tray_path(self) -> str:
        return str(self.config.get("prims", {}).get("tray", "/World/Tray"))

    def _table_path(self) -> str:
        return str(self.config.get("prims", {}).get("table", "/World/OakTableSmall"))

    def _randomize_cube_and_tray(
        self,
    ) -> tuple[
        tuple[list[float], tuple[float, float, float, float]] | None,
        tuple[list[float], tuple[float, float, float, float]] | None,
        tuple[str, ...],
    ]:
        cube = self.stage.GetPrimAtPath(self._cube_path())
        tray = self.stage.GetPrimAtPath(self._tray_path())
        missing = tuple(
            path
            for path, prim in ((self._cube_path(), cube), (self._tray_path(), tray))
            if not prim.IsValid()
        )
        if missing:
            return None, None, missing

        bounds = self._table_xy_bounds()
        placement = self.config.get("placement", {})
        gap = float(placement.get("object_gap_m", 0.035))
        cube_radius = self._footprint_radius(cube, float(placement.get("cube_radius_m", 0.035)))
        tray_radius = self._footprint_radius(tray, float(placement.get("tray_radius_m", 0.16)))
        min_distance = max(float(placement.get("min_center_distance_m", 0.0)), cube_radius + tray_radius + gap)

        tray_xy = self._sample_xy(bounds, tray_radius)
        cube_xy = self._sample_separated_xy(bounds, cube_radius, tray_xy, min_distance)

        tray_pose = self._set_randomized_pose(
            tray,
            self._tray_path(),
            tray_xy,
            placement.get("tray_yaw_deg", [-20.0, 20.0]),
        )
        cube_pose = self._set_randomized_pose(
            cube,
            self._cube_path(),
            cube_xy,
            placement.get("cube_yaw_deg", [-180.0, 180.0]),
        )
        return tray_pose, cube_pose, ()

    def _set_randomized_pose(self, prim, path: str, xy, yaw_range) -> tuple[list[float], tuple[float, float, float, float]]:
        base_position, base_orientation = self._base_object_poses.get(path, self._get_world_pose(prim))
        yaw = self.rng.uniform(float(yaw_range[0]), float(yaw_range[1]))
        position = [float(xy[0]), float(xy[1]), float(base_position[2])]
        orientation = _quat_multiply_wxyz(
            base_orientation,
            _euler_xyz_degrees_to_quat_wxyz((0.0, 0.0, yaw)),
        )
        self._set_world_pose(prim, position, orientation)
        return position, orientation

    def _table_xy_bounds(self) -> tuple[float, float, float, float]:
        from pxr import UsdGeom

        fallback = self.config.get("placement", {}).get(
            "fallback_table_bounds_m",
            {"x": [-0.35, 0.35], "y": [-0.28, 0.28]},
        )
        table = self.stage.GetPrimAtPath(self._table_path())
        if not table.IsValid():
            return (
                float(fallback["x"][0]),
                float(fallback["x"][1]),
                float(fallback["y"][0]),
                float(fallback["y"][1]),
            )

        bbox_cache = UsdGeom.BBoxCache(
            0.0,
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        )
        box = bbox_cache.ComputeWorldBound(table).ComputeAlignedBox()
        min_p = box.GetMin()
        max_p = box.GetMax()
        values = [float(min_p[0]), float(max_p[0]), float(min_p[1]), float(max_p[1])]
        if not all(math.isfinite(value) for value in values):
            return (
                float(fallback["x"][0]),
                float(fallback["x"][1]),
                float(fallback["y"][0]),
                float(fallback["y"][1]),
            )
        return self._shrink_bounds_to_center((values[0], values[1], values[2], values[3]))

    def _shrink_bounds_to_center(self, bounds: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        placement = self.config.get("placement", {})
        scale = placement.get("table_center_area_scale", [0.6, 0.6])
        x_scale = max(0.05, min(1.0, float(scale[0])))
        y_scale = max(0.05, min(1.0, float(scale[1])))
        x_min, x_max, y_min, y_max = bounds
        center_x = (x_min + x_max) * 0.5
        center_y = (y_min + y_max) * 0.5
        half_x = (x_max - x_min) * x_scale * 0.5
        half_y = (y_max - y_min) * y_scale * 0.5
        return center_x - half_x, center_x + half_x, center_y - half_y, center_y + half_y

    def _footprint_radius(self, prim, fallback: float) -> float:
        from pxr import UsdGeom

        try:
            bbox_cache = UsdGeom.BBoxCache(
                0.0,
                [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
            )
            box = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
            min_p = box.GetMin()
            max_p = box.GetMax()
            x_extent = float(max_p[0]) - float(min_p[0])
            y_extent = float(max_p[1]) - float(min_p[1])
            radius = max(x_extent, y_extent) * 0.5
            if math.isfinite(radius) and 0.0 < radius < 1.0:
                return radius + float(self.config.get("placement", {}).get("boundary_margin_m", 0.025))
        except Exception:
            pass
        return fallback + float(self.config.get("placement", {}).get("boundary_margin_m", 0.025))

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

    def _sample_separated_xy(
        self,
        bounds: tuple[float, float, float, float],
        radius: float,
        occupied_xy: tuple[float, float],
        min_distance: float,
    ) -> tuple[float, float]:
        for _ in range(200):
            xy = self._sample_xy(bounds, radius)
            if math.hypot(xy[0] - occupied_xy[0], xy[1] - occupied_xy[1]) >= min_distance:
                return xy

        x_min, x_max, y_min, y_max = bounds
        candidates = [
            (x_min + radius, y_min + radius),
            (x_min + radius, y_max - radius),
            (x_max - radius, y_min + radius),
            (x_max - radius, y_max - radius),
        ]
        return max(candidates, key=lambda xy: math.hypot(xy[0] - occupied_xy[0], xy[1] - occupied_xy[1]))

    def _capture_light_baselines(self) -> None:
        from pxr import UsdLux

        self._ensure_distant_light()
        self._set_dome_light_intensity()
        self._base_lights = {}
        for prim in self._light_prims():
            light = UsdLux.LightAPI(prim)
            path = str(prim.GetPath())
            position, orientation = self._get_world_pose(prim)
            orientation = self._get_local_orientation_wxyz(prim) or orientation
            intensity = light.GetIntensityAttr().Get()
            exposure = light.GetExposureAttr().Get()
            self._base_lights[path] = {
                "position": position,
                "orientation": orientation,
                "intensity": float(intensity) if intensity is not None else 3000.0,
                "exposure": float(exposure) if exposure is not None else 0.0,
            }

    def _ensure_distant_light(self) -> None:
        from pxr import Sdf, UsdLux

        lighting = self.config.get("lighting", {})
        light_path = str(lighting.get("distant_light_path", "/Environment/RandomizedDistantLight"))
        parent_path = light_path.rsplit("/", 1)[0]
        if parent_path:
            self.stage.DefinePrim(parent_path, "Xform")
        light = UsdLux.DistantLight.Define(self.stage, Sdf.Path(light_path))
        light.CreateIntensityAttr(float(lighting.get("distant_light_base_intensity", 3500.0)))
        light.CreateAngleAttr(float(lighting.get("distant_light_angle", 1.0)))

    def _set_dome_light_intensity(self) -> None:
        from pxr import UsdLux

        lighting = self.config.get("lighting", {})
        dome_intensity = lighting.get("dome_light_intensity", 0.0)
        for prim in self.stage.Traverse():
            if prim.GetTypeName() == "DomeLight":
                UsdLux.LightAPI(prim).CreateIntensityAttr(float(dome_intensity))

    def _light_prims(self):
        light_root = str(self.config.get("prims", {}).get("light_root", "/World/Environment"))
        lighting = self.config.get("lighting", {})
        dedicated_light_path = str(lighting.get("distant_light_path", "/Environment/RandomizedDistantLight"))
        dedicated = self.stage.GetPrimAtPath(dedicated_light_path)
        if dedicated.IsValid():
            return [dedicated]

        roots = [light_root, "/Environment", "/World/Environment"]
        for root_path in dict.fromkeys(roots):
            root = self.stage.GetPrimAtPath(root_path)
            if not root.IsValid():
                continue
            light_prims = [
                prim
                for prim in self.stage.Traverse()
                if (str(prim.GetPath()) == root_path or str(prim.GetPath()).startswith(root_path + "/"))
                and prim.GetTypeName() == "DistantLight"
            ]
            if light_prims:
                return light_prims
        return [prim for prim in self.stage.Traverse() if prim.GetTypeName() == "DistantLight"]

    def _get_local_orientation_wxyz(self, prim) -> tuple[float, float, float, float] | None:
        try:
            from pxr import UsdGeom

            translate, rotate, scale, _, _ = UsdGeom.XformCommonAPI(prim).GetXformVectors(0.0)
            if rotate:
                return _euler_xyz_degrees_to_quat_wxyz(rotate)
        except Exception:
            pass
        return None

    def _set_light_pose(self, prim, position, orientation) -> None:
        self._set_world_pose(prim, position, orientation)
        try:
            from pxr import Gf, UsdGeom

            w, x, y, z = [float(value) for value in orientation]
            xform = UsdGeom.Xformable(prim)
            orient_op = None
            for op in xform.GetOrderedXformOps():
                if op.GetOpName() == "xformOp:orient":
                    orient_op = op
                    break
            if orient_op is None:
                orient_op = xform.AddOrientOp()
            orient_op.Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
        except Exception:
            pass

    def _randomize_lighting(self) -> tuple[float | None, tuple | None]:
        from pxr import UsdLux

        if not self._base_lights:
            self._capture_light_baselines()
        if not self._base_lights:
            return None, None
        self._set_dome_light_intensity()

        lighting = self.config.get("lighting", {})
        intensity_range = lighting.get("intensity", [1800.0, 6500.0])
        exposure_jitter = lighting.get("exposure_jitter", [-0.2, 0.2])
        rotation = lighting.get("direction_deg", {})
        sampled_rotate = (
            self.rng.uniform(*[float(v) for v in rotation.get("x", [35.0, 75.0])]),
            self.rng.uniform(*[float(v) for v in rotation.get("y", [-25.0, 25.0])]),
            self.rng.uniform(*[float(v) for v in rotation.get("z", [-180.0, 180.0])]),
        )
        orientation = _euler_xyz_degrees_to_quat_wxyz(sampled_rotate)

        intensities = []
        for path, base in self._base_lights.items():
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue
            intensity = self.rng.uniform(float(intensity_range[0]), float(intensity_range[1]))
            exposure = float(base["exposure"]) + self.rng.uniform(float(exposure_jitter[0]), float(exposure_jitter[1]))
            light = UsdLux.LightAPI(prim)
            light.CreateIntensityAttr(float(intensity))
            light.CreateExposureAttr(float(exposure))
            self._set_light_pose(
                prim,
                base["position"],
                orientation,
            )
            intensities.append(intensity)

        if not intensities:
            return None, None
        return sum(intensities) / len(intensities), orientation

