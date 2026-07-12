from __future__ import annotations

import colorsys
from dataclasses import dataclass, field
import math
import random
from typing import Any


def get_world_pose_usd(prim) -> tuple[list[float], list[float]]:
    """Read a prim's world position and orientation directly from USD.

    Uses pxr.UsdGeom.XformCache (a lightweight C++ object with no Python
    ``__del__`` callback registration) instead of isaacsim's XFormPrim. The
    throwaway ``XFormPrim(prim_path=...)`` instances used elsewhere trigger an
    ``AttributeError: 'XFormPrim' object has no attribute '_callbacks'`` from
    ``Prim.__del__`` when they are garbage-collected after a partially-failed
    init (e.g. during shutdown / stage reload). The USD-native path has no such
    object lifecycle and is the reliable way to read/write poses here.

    Returns (position_xyz, orientation_wxyz). Orientation defaults to identity
    if the prim has no meaningful rotation.
    """
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
    """Set a prim's world pose directly via USD.

    Sets the prim's local transform so that its world transform matches the
    requested position/orientation, assuming the prim's parent is at the world
    origin (the common case for the top-level scene prims randomized here:
    trays, lights, spawned objects under /World). Uses UsdGeom.XformCommonAPI
    (no Python wrapper __del__). For prims with non-identity parent transforms
    this sets the *local* pose; callers that need true world-space placement on
    nested prims should re-parent or compute the relative transform.
    """
    from pxr import Gf, UsdGeom

    api = UsdGeom.XformCommonAPI(prim)
    api.SetTranslate(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))
    # XformCommonAPI.SetRotate expects XYZ euler degrees; convert from the
    # quaternion. Falling back to identity on a degenerate quaternion.
    roll, pitch, yaw = _quat_wxyz_to_euler_xyz_degrees(orientation_wxyz)
    api.SetRotate((roll, pitch, yaw), UsdGeom.XformCommonAPI.RotationOrderXYZ)


def _quat_wxyz_to_euler_xyz_degrees(quat_wxyz) -> tuple[float, float, float]:
    """Convert a [w, x, y, z] quaternion to XYZ euler angles in degrees."""
    qw, qx, qy, qz = [float(v) for v in quat_wxyz]
    # Normalize to guard against degenerate input.
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm < 1e-12:
        return 0.0, 0.0, 0.0
    qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (
        math.degrees(roll),
        math.degrees(pitch),
        math.degrees(yaw),
    )


@dataclass
class DomainRandomizationSample:
    nut_count: int = 0
    bolt_count: int = 0
    light_intensity: float | None = None
    floor_color: tuple[float, float, float] | None = None
    # Maps USD prim path -> {"position": [...], "orientation": [...]} for all
    # randomized trays / objects, enabling deterministic scene reconstruction
    # during deferred rendering.
    tray_poses: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        """Serialize this sample to a plain JSON-compatible dict."""
        return {
            "type": self.__class__.__name__,
            "nut_count": self.nut_count,
            "bolt_count": self.bolt_count,
            "light_intensity": self.light_intensity,
            "floor_color": list(self.floor_color) if self.floor_color is not None else None,
            "tray_poses": {
                path: {"position": list(pose["position"]), "orientation": list(pose["orientation"])}
                for path, pose in self.tray_poses.items()
            },
        }

    @classmethod
    def from_dict(cls, d: dict) -> "DomainRandomizationSample":
        """Reconstruct a sample from a plain dict (deserialized from JSON)."""
        floor_color = d.get("floor_color")
        tray_poses_raw = d.get("tray_poses", {})
        return cls(
            nut_count=int(d.get("nut_count", 0)),
            bolt_count=int(d.get("bolt_count", 0)),
            light_intensity=(
                float(d["light_intensity"]) if d.get("light_intensity") is not None else None
            ),
            floor_color=(
                tuple(float(v) for v in floor_color[:3]) if floor_color is not None else None
            ),
            tray_poses={
                path: {
                    "position": [float(v) for v in pose["position"]],
                    "orientation": [float(v) for v in pose["orientation"]],
                }
                for path, pose in tray_poses_raw.items()
            },
        )


class DomainRandomizer:
    def __init__(self, stage, config: dict[str, Any] | None):
        self.stage = stage
        self.config = config or {}
        self.enabled = bool(self.config.get("enabled", False))
        self.rng = random.Random(self.config.get("seed"))
        self._tray_base_poses: dict[str, tuple[list[float], list[float]]] = {}
        self._spawned_xy: list[tuple[float, float]] = []

    def initialize(self) -> None:
        if not self.enabled:
            return
        for path in self._tray_paths():
            prim = self.stage.GetPrimAtPath(path)
            if prim.IsValid():
                self._tray_base_poses[path] = self._get_world_pose(prim)
        self._ensure_floor_material()
        self._hide_original_objects()

    def randomize(self, settle_step) -> DomainRandomizationSample:
        sample = self.apply_randomization()

        for _ in range(max(0, int(self.config.get("settle_steps", 90)))):
            settle_step(render=True)
        return sample

    def apply_randomization_from_dict(self, sample_dict: dict) -> None:
        """Deterministically reconstruct a previously saved domain-randomization
        state from its serialized dict.

        Unlike ``apply_randomization()``, this method does NOT draw new random
        values — it directly sets the exact poses and lighting that were
        recorded during teleop so the rendered video matches the original scene.

        Subclasses should override ``_apply_saved_object_poses`` and
        ``_apply_saved_lighting`` to handle robot-specific prims.
        """
        if not self.enabled:
            return
        self._apply_saved_lighting(sample_dict)
        self._apply_saved_floor(sample_dict)
        self._apply_saved_object_poses(sample_dict)

    # ------------------------------------------------------------------
    # Deterministic replay helpers — override in subclasses as needed
    # ------------------------------------------------------------------

    def _apply_saved_lighting(self, sample_dict: dict) -> None:
        """Restore the saved light intensity to all lights under light_root."""
        from pxr import UsdLux

        intensity = sample_dict.get("light_intensity")
        if intensity is None:
            return
        light_root = str(self.config.get("prims", {}).get("light_root", "/World/Environment"))
        for prim in self.stage.Traverse():
            path = str(prim.GetPath())
            if path != light_root and not path.startswith(light_root + "/"):
                continue
            if prim.GetTypeName() not in {
                "CylinderLight", "DiskLight", "DistantLight",
                "DomeLight", "RectLight", "SphereLight",
            }:
                continue
            UsdLux.LightAPI(prim).CreateIntensityAttr(float(intensity))

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

    def _apply_saved_object_poses(self, sample_dict: dict) -> None:
        """Restore saved tray poses (base class handles the generic tray-jitter case)."""
        tray_poses = sample_dict.get("tray_poses", {})
        for path, pose_data in tray_poses.items():
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue
            position = pose_data["position"]
            orientation = pose_data["orientation"]
            self._set_world_pose(prim, position, orientation)

    def apply_randomization(self) -> DomainRandomizationSample:
        """Mutate the USD stage for a new random sample without stepping physics.

        This performs all prim additions/removals/edits (clearing spawned
        objects, lighting/floor changes, tray jitter, spawning nuts/bolts) and
        returns the sample descriptor. It must be called while physics is
        STOPPED so that PhysX rebuilds a consistent scene on the next
        play()/reset() -- mutating physics-enabled prims while the simulation
        is running leaves existing articulation tensor views stale or corrupt,
        which silently freezes teleop. Settle-stepping (which needs physics
        running) is left to the caller via ``settle`` or ``randomize``.
        """
        sample = DomainRandomizationSample()
        if not self.enabled:
            return sample

        self._restore_trays()
        self._clear_spawned_objects()
        sample.light_intensity = self._randomize_lighting()
        sample.floor_color = self._randomize_floor()
        # Capture tray poses after jitter so they can be serialized.
        self._randomize_trays()
        sample.tray_poses = self._capture_tray_poses()
        sample.nut_count, sample.bolt_count = self._spawn_objects()
        return sample

    def settle(self, settle_step) -> None:
        """Step physics so newly spawned objects fall/settle into place.

        Must be called AFTER the simulation has been restarted (play/reset) so
        the spawned bodies exist in the PhysX scene.
        """
        if not self.enabled:
            return
        for _ in range(max(0, int(self.config.get("settle_steps", 90)))):
            settle_step(render=True)

    def _tray_paths(self) -> list[str]:
        prims = self.config.get("prims", {})
        paths = [
            str(prims[key])
            for key in ("left_tray", "middle_tray", "right_tray")
            if prims.get(key)
        ]
        paths.extend(str(path) for path in self.config.get("trays", {}).get("extra_paths", []))
        paths.extend(str(path) for path in self.config.get("objects", {}).get("spawn_tray_candidates", []))
        return list(dict.fromkeys(paths))

    def _get_xform_vectors(self, prim):
        from pxr import UsdGeom

        translate, rotate, scale, _, _ = UsdGeom.XformCommonAPI(prim).GetXformVectors(0.0)
        return list(translate), list(rotate), list(scale)

    def _set_xform_vectors(self, prim, translate, rotate, scale) -> None:
        from pxr import UsdGeom

        api = UsdGeom.XformCommonAPI(prim)
        api.SetTranslate(tuple(float(v) for v in translate))
        api.SetRotate(tuple(float(v) for v in rotate), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        api.SetScale(tuple(float(v) for v in scale))

    def _get_world_pose(self, prim) -> tuple[list[float], list[float]]:
        return get_world_pose_usd(prim)

    def _set_world_pose(self, prim, position, orientation) -> None:
        set_world_pose_usd(prim, position, orientation)

    def _restore_trays(self) -> None:
        for path, (position, orientation) in self._tray_base_poses.items():
            prim = self.stage.GetPrimAtPath(path)
            if prim.IsValid():
                self._set_world_pose(prim, position, orientation)

    def _randomize_trays(self) -> None:
        xy_jitter = float(self.config.get("trays", {}).get("xy_jitter_m", 0.035))
        yaw_jitter = float(self.config.get("trays", {}).get("yaw_jitter_deg", 7.0))
        for path, (position, orientation) in self._tray_base_poses.items():
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue
            randomized_position = [
                position[0] + self.rng.uniform(-xy_jitter, xy_jitter),
                position[1] + self.rng.uniform(-xy_jitter, xy_jitter),
                position[2],
            ]
            yaw_delta = _euler_xyz_degrees_to_quat_wxyz((0.0, 0.0, self.rng.uniform(-yaw_jitter, yaw_jitter)))
            self._set_world_pose(prim, randomized_position, _quat_multiply_wxyz(orientation, yaw_delta))

    def _capture_tray_poses(self) -> dict:
        """Read back the current world pose of every tracked tray prim.

        Called after ``_randomize_trays()`` so the returned dict contains the
        actual randomized poses (not the base poses). The dict can be stored in
        ``DomainRandomizationSample.tray_poses`` and later passed to
        ``_apply_saved_object_poses()`` during deferred rendering.
        """
        poses = {}
        for path in self._tray_base_poses:
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue
            position, orientation = self._get_world_pose(prim)
            poses[path] = {"position": position, "orientation": orientation}
        return poses

    def _randomize_lighting(self) -> float | None:
        from pxr import UsdLux

        lighting = self.config.get("lighting", {})
        intensity_range = lighting.get("intensity", [300.0, 1800.0])
        exposure_range = lighting.get("exposure", [-1.0, 1.0])
        intensity = self.rng.uniform(float(intensity_range[0]), float(intensity_range[1]))
        exposure = self.rng.uniform(float(exposure_range[0]), float(exposure_range[1]))
        light_root = str(self.config.get("prims", {}).get("light_root", "/World/Environment"))
        root_prim = self.stage.GetPrimAtPath(light_root)
        if not root_prim.IsValid():
            return None

        light_prims = []
        for prim in self.stage.Traverse():
            path = str(prim.GetPath())
            if path != light_root and not path.startswith(light_root + "/"):
                continue
            if prim.GetTypeName() not in {
                "CylinderLight",
                "DiskLight",
                "DistantLight",
                "DomeLight",
                "RectLight",
                "SphereLight",
            }:
                continue
            light_prims.append(prim)
            light = UsdLux.LightAPI(prim)
            light.CreateIntensityAttr(float(intensity))
            light.CreateExposureAttr(float(exposure))

        rotation = lighting.get("rotation_deg", {})
        sampled_rotate = [
            self.rng.uniform(*[float(v) for v in rotation.get("x", [35.0, 75.0])]),
            self.rng.uniform(*[float(v) for v in rotation.get("y", [-15.0, 15.0])]),
            self.rng.uniform(*[float(v) for v in rotation.get("z", [-180.0, 180.0])]),
        ]
        if root_prim.GetTypeName() == "Xform":
            translate, _, scale = self._get_xform_vectors(root_prim)
            self._set_xform_vectors(root_prim, translate, sampled_rotate, scale)
        return intensity

    def _ensure_floor_material(self) -> None:
        from pxr import Sdf, Usd, UsdGeom, UsdShade

        looks_path = str(self.config.get("prims", {}).get("looks", "/World/Looks"))
        self.stage.DefinePrim(looks_path, "Scope")
        material_path = f"{looks_path}/randomized_floor_material"
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
            # 30% similar to metallic nuts/bolts: learn shape and texture, not only contrast.
            (0.30, ((0.0, 360.0), (0.0, 0.05), (0.30, 0.70))),
            # 35% remaining core realistic industrial/lab contrast surfaces.
            (0.12, ((30.0, 60.0), (0.05, 0.15), (0.75, 0.90))),
            (0.12, ((25.0, 45.0), (0.10, 0.25), (0.40, 0.65))),
            (0.11, ((0.0, 360.0), (0.0, 0.05), (0.10, 0.25))),
            # 25% moderate plausible but less common surfaces.
            (0.07, ((200.0, 220.0), (0.10, 0.25), (0.35, 0.55))),
            (0.06, ((80.0, 120.0), (0.10, 0.20), (0.30, 0.50))),
            (0.06, ((15.0, 35.0), (0.15, 0.30), (0.25, 0.45))),
            (0.06, ((210.0, 240.0), (0.05, 0.15), (0.45, 0.65))),
            # 10% mild extremes. Saturation and lightness stay bounded to avoid neon/black/white.
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

    def _clear_spawned_objects(self) -> None:
        spawn_root = str(self.config.get("prims", {}).get("spawn_root", "/World/RandomizedObjects"))
        root = self.stage.GetPrimAtPath(spawn_root)
        self._spawned_xy = []
        if not root.IsValid():
            self.stage.DefinePrim(spawn_root, "Xform")
            return
        for child in list(root.GetChildren()):
            self.stage.RemovePrim(child.GetPath())

    def _hide_original_objects(self) -> None:
        from pxr import UsdGeom

        prims = self.config.get("prims", {})
        for key in ("nut", "bolt"):
            path = prims.get(key)
            if not path:
                continue
            prim = self.stage.GetPrimAtPath(str(path))
            if prim.IsValid():
                UsdGeom.Imageable(prim).MakeInvisible()

    def _spawn_objects(self) -> tuple[int, int]:
        objects = self.config.get("objects", {})
        nut_count = self.rng.randint(*[int(v) for v in objects.get("nut_count", [3, 6])])
        bolt_count = self.rng.randint(*[int(v) for v in objects.get("bolt_count", [4, 8])])
        for kind, count in (("nut", nut_count), ("bolt", bolt_count)):
            for index in range(count):
                self._spawn_object(kind, index)
        return nut_count, bolt_count

    def _spawn_object(self, kind: str, index: int) -> None:
        from pxr import Gf, UsdGeom

        asset_key = f"{kind}_usd"
        asset_path = self.config.get("assets", {}).get(asset_key)
        if not asset_path:
            return

        spawn_root = str(self.config.get("prims", {}).get("spawn_root", "/World/RandomizedObjects"))
        prim_path = f"{spawn_root}/{kind}_{index:02d}"
        prim = self.stage.DefinePrim(prim_path, "Xform")
        prim.GetReferences().AddReference(str(asset_path))

        x, y, z = self._sample_spawn_position()
        yaw_range = self.config.get("objects", {}).get("yaw_deg", [-180.0, 180.0])
        yaw = self.rng.uniform(float(yaw_range[0]), float(yaw_range[1]))
        object_scale = self._object_scale(kind)
        api = UsdGeom.XformCommonAPI(prim)
        api.SetTranslate(Gf.Vec3d(x, y, z))
        api.SetRotate((0.0, 0.0, yaw), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        api.SetScale(Gf.Vec3f(float(object_scale[0]), float(object_scale[1]), float(object_scale[2])))

    def _object_scale(self, kind: str) -> tuple[float, float, float]:
        objects = self.config.get("objects", {})
        scale = objects.get(f"{kind}_scale", objects.get("scale", [2.0, 2.0, 3.5]))
        if isinstance(scale, int | float):
            return float(scale), float(scale), float(scale)
        return tuple(float(value) for value in scale[:3])

    def _sample_spawn_position(self) -> tuple[float, float, float]:
        from pxr import UsdGeom

        middle = self._spawn_tray_prim()
        z_offset = float(self.config.get("objects", {}).get("spawn_z_offset_m", 0.06))
        if not middle.IsValid():
            return self._sample_spawn_position_from_config(
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0, 0.0),
                z_offset,
            )

        if not bool(self.config.get("objects", {}).get("use_bbox", False)):
            position, orientation = self._get_world_pose(middle)
            return self._sample_spawn_position_from_config(position, orientation, z_offset)

        bbox_cache = UsdGeom.BBoxCache(
            0.0,
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        )
        box = bbox_cache.ComputeWorldBound(middle).ComputeAlignedBox()
        min_p = box.GetMin()
        max_p = box.GetMax()
        margin = float(self.config.get("objects", {}).get("bbox_margin_m", 0.05))
        min_distance = float(self.config.get("objects", {}).get("min_distance_m", 0.045))
        if not self._valid_bbox(min_p, max_p, margin):
            position, orientation = self._get_world_pose(middle)
            return self._sample_spawn_position_from_config(position, orientation, z_offset)

        x_min = float(min_p[0]) + margin
        x_max = float(max_p[0]) - margin
        y_min = float(min_p[1]) + margin
        y_max = float(max_p[1]) - margin
        if x_min >= x_max:
            x_min, x_max = float(min_p[0]), float(max_p[0])
        if y_min >= y_max:
            y_min, y_max = float(min_p[1]), float(max_p[1])
        for _ in range(100):
            x = self.rng.uniform(x_min, x_max)
            y = self.rng.uniform(y_min, y_max)
            if all(math.hypot(x - px, y - py) >= min_distance for px, py in self._spawned_xy):
                self._spawned_xy.append((x, y))
                return x, y, float(max_p[2]) + z_offset
        x = self.rng.uniform(x_min, x_max)
        y = self.rng.uniform(y_min, y_max)
        self._spawned_xy.append((x, y))
        return x, y, float(max_p[2]) + z_offset

    def _sample_spawn_position_from_config(
        self,
        center: tuple[float, float, float] | list[float],
        orientation: tuple[float, float, float, float] | list[float],
        z_offset: float,
    ) -> tuple[float, float, float]:
        objects = self.config.get("objects", {})
        spawn_area = objects.get("spawn_area_m", {"x": [-0.16, 0.16], "y": [-0.10, 0.10]})
        x_range = spawn_area.get("x", [-0.16, 0.16])
        y_range = spawn_area.get("y", [-0.10, 0.10])
        min_distance = float(objects.get("min_distance_m", 0.045))
        center_x = float(center[0])
        center_y = float(center[1])
        center_z = float(center[2])

        for _ in range(100):
            x, y = self._local_xy_to_world(
                center_x,
                center_y,
                orientation,
                self.rng.uniform(float(x_range[0]), float(x_range[1])),
                self.rng.uniform(float(y_range[0]), float(y_range[1])),
            )
            if all(math.hypot(x - px, y - py) >= min_distance for px, py in self._spawned_xy):
                self._spawned_xy.append((x, y))
                return x, y, center_z + z_offset
        x, y = self._local_xy_to_world(
            center_x,
            center_y,
            orientation,
            self.rng.uniform(float(x_range[0]), float(x_range[1])),
            self.rng.uniform(float(y_range[0]), float(y_range[1])),
        )
        self._spawned_xy.append((x, y))
        return x, y, center_z + z_offset

    def _local_xy_to_world(
        self,
        center_x: float,
        center_y: float,
        orientation,
        local_x: float,
        local_y: float,
    ) -> tuple[float, float]:
        w, x, y, z = [float(value) for value in orientation]
        world_x = center_x + (1.0 - 2.0 * (y * y + z * z)) * local_x + 2.0 * (x * y - z * w) * local_y
        world_y = center_y + 2.0 * (x * y + z * w) * local_x + (1.0 - 2.0 * (x * x + z * z)) * local_y
        return world_x, world_y

    def _valid_bbox(self, min_p, max_p, margin: float) -> bool:
        values = [float(min_p[index]) for index in range(3)] + [float(max_p[index]) for index in range(3)]
        if not all(math.isfinite(value) for value in values):
            return False
        extents = [float(max_p[index]) - float(min_p[index]) for index in range(3)]
        if any(extent <= 0.0 or extent > 10.0 for extent in extents):
            return False
        return extents[0] > margin * 2.0 and extents[1] > margin * 2.0

    def _spawn_tray_prim(self):
        prims = self.config.get("prims", {})
        paths = list(self.config.get("objects", {}).get("spawn_tray_candidates", []))
        if prims.get("middle_tray"):
            paths.append(prims["middle_tray"])
        for path in dict.fromkeys(str(path) for path in paths):
            prim = self.stage.GetPrimAtPath(path)
            if prim.IsValid():
                return prim
        return self.stage.GetPrimAtPath(str(prims.get("middle_tray", "")))


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
