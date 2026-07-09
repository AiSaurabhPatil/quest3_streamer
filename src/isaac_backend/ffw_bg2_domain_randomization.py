from __future__ import annotations

from dataclasses import dataclass
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


class FFWBG2DomainRandomizer(DomainRandomizer):
    def __init__(self, stage, config: dict[str, Any] | None):
        super().__init__(stage, config)
        self._base_object_poses: dict[str, tuple[list[float], list[float]]] = {}
        self._base_lights: dict[str, dict[str, Any]] = {}

    def initialize(self) -> None:
        if not self.enabled:
            return
        for path in (self._cube_path(), self._tray_path()):
            prim = self.stage.GetPrimAtPath(path)
            if prim.IsValid():
                self._base_object_poses[path] = self._get_world_pose(prim)
        self._capture_light_baselines()

    def randomize(self, settle_step) -> FFWBG2DomainRandomizationSample:
        sample = self.apply_randomization()

        for _ in range(max(0, int(self.config.get("settle_steps", 30)))):
            settle_step(render=True)
        return sample

    def apply_randomization(self) -> FFWBG2DomainRandomizationSample:
        sample = FFWBG2DomainRandomizationSample()
        if not self.enabled:
            return sample

        sample.light_intensity = self._randomize_lighting()
        sample.tray_pose, sample.cube_pose, sample.missing_prims = self._randomize_cube_and_tray()
        return sample

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

    def _randomize_lighting(self) -> float | None:
        from pxr import UsdLux

        if not self._base_lights:
            self._capture_light_baselines()
        if not self._base_lights:
            return None
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
            return None
        return sum(intensities) / len(intensities)
