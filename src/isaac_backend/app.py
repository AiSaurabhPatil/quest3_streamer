from __future__ import annotations

from dataclasses import dataclass, field
import sys


DEFAULT_SIMULATION_CONFIG = {
    "headless": False,
    "width": 1280,
    "height": 720,
    "window_width": 1280,
    "window_height": 720,
}

DEFAULT_HIDDEN_WINDOWS = (
    "Stage",
    "Layer",
    "Render Settings",
    "Content",
    "Content Library",
    "Console",
    "Property",
    "Properties",
    "Semantics",
    "Visual Scripting",
)


@dataclass
class IsaacAppConfig:
    simulation: dict = field(default_factory=lambda: dict(DEFAULT_SIMULATION_CONFIG))
    experience: str | None = None
    webrtc_streaming: bool = False
    quiet_logging: bool = False
    warmup_updates: int = 30
    stage_stabilization_updates: int = 50
    world_stabilization_updates: int = 20
    post_reset_updates: int = 20
    hide_ui_panels: bool = True
    hidden_windows: tuple[str, ...] = DEFAULT_HIDDEN_WINDOWS
    # Control-loop pacing. The control loop (physics + IK) runs at up to
    # target_control_rate_hz, while rendering is throttled to one render every
    # render_every_n_steps. This decouples control latency from the render frame
    # time, which is the dominant per-iteration cost. Set render_every_n_steps=1
    # to render every step (legacy behavior).
    target_control_rate_hz: float = 120.0
    render_every_n_steps: int = 2

    @classmethod
    def from_mapping(cls, values: dict | None):
        values = values or {}
        simulation = dict(DEFAULT_SIMULATION_CONFIG)
        simulation.update(values.get("simulation", {}))
        for key in DEFAULT_SIMULATION_CONFIG:
            if key in values:
                simulation[key] = values[key]
        hidden_windows = tuple(values.get("hidden_windows", DEFAULT_HIDDEN_WINDOWS))
        return cls(
            simulation=simulation,
            experience=values.get("experience"),
            webrtc_streaming=bool(values.get("webrtc_streaming", False)),
            quiet_logging=bool(values.get("quiet_logging", False)),
            warmup_updates=int(values.get("warmup_updates", 30)),
            stage_stabilization_updates=int(values.get("stage_stabilization_updates", 50)),
            world_stabilization_updates=int(values.get("world_stabilization_updates", 20)),
            post_reset_updates=int(values.get("post_reset_updates", 20)),
            hide_ui_panels=bool(values.get("hide_ui_panels", True)),
            hidden_windows=hidden_windows,
            target_control_rate_hz=max(1.0, float(values.get("target_control_rate_hz", 120.0))),
            render_every_n_steps=max(1, int(values.get("render_every_n_steps", 2))),
        )


def _import_simulation_app():
    """Import SimulationApp supporting Isaac Sim 6.x (isaacsim) and 5.x (omni.isaac.kit)."""
    try:
        # Isaac Sim 6.0+ new API
        from isaacsim import SimulationApp
        return SimulationApp
    except ImportError:
        pass
    try:
        # Isaac Sim 5.x / 4.x legacy API
        from omni.isaac.kit import SimulationApp
        return SimulationApp
    except ImportError:
        raise ImportError(
            "Cannot import SimulationApp. Ensure Isaac Sim is installed and "
            "this script is run via Isaac Sim's python.sh."
        )


def _enable_extension(name: str) -> None:
    """Enable an Isaac Sim extension, supporting both 6.x and 5.x APIs."""
    try:
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension(name)
        return
    except ImportError:
        pass
    try:
        from omni.isaac.core.utils.extensions import enable_extension
        enable_extension(name)
    except ImportError as exc:
        raise ImportError(f"Cannot enable extension '{name}': {exc}") from exc


def _ros2_bridge_extension_name() -> str:
    """Return the correct ROS2 bridge extension name for the installed Isaac Sim version."""
    try:
        import importlib.util
        if importlib.util.find_spec("isaacsim") is not None:
            return "isaacsim.ros2.bridge"
    except Exception:
        pass
    return "omni.isaac.ros2_bridge"


def _resolve_livestream_extension() -> str | None:
    """Return the name of the livestream extension present in this Isaac Sim install.

    Different Isaac Sim builds ship different livestream extensions:
      - Cloud/NVCF builds:  isaacsim.services.livestream.nvcf / omni.services.livestream.nvcf
      - Local WebRTC builds: omni.kit.livestream.app  (the common local-install case)
    We probe the extension manager/registry for what is actually available rather
    than hardcoding a name, so we never log a spurious "Failed to resolve
    extension dependencies" error for an extension that isn't installed. When the
    app is launched with the streaming kit experience file, the correct livestream
    extension is already enabled by the kit, so returning None here is safe.
    """
    try:
        from omni.kit.app import get_app_manager  # noqa: F401
        manager = get_app_manager().get_extension_manager()
    except Exception:
        manager = None

    candidates = (
        "isaacsim.services.livestream.nvcf",  # Isaac Sim 6.x cloud/NVCF
        "omni.services.livestream.nvcf",       # Isaac Sim 5.x cloud/NVCF
        "omni.kit.livestream.app",             # local WebRTC streaming (common local install)
    )

    if manager is not None:
        try:
            available_extensions = manager.get_extensions()
            for candidate in candidates:
                for ext in available_extensions:
                    ext_id = getattr(ext, "id", "") or str(ext)
                    if ext_id.startswith(candidate):
                        return candidate
        except Exception:
            pass

    # Fallback: check the filesystem exts directories for a matching folder.
    import os
    for base in ("exts", "extsDeprecated", "extscache"):
        isaac_root = os.environ.get("ISAAC_SIM_PATH", "/home/saurabh/isaac_sim")
        exts_dir = os.path.join(isaac_root, base)
        if not os.path.isdir(exts_dir):
            continue
        try:
            entries = os.listdir(exts_dir)
        except OSError:
            continue
        for candidate in candidates:
            if any(entry.startswith(candidate) for entry in entries):
                return candidate

    return None


class IsaacApp:
    def __init__(self, app_config: IsaacAppConfig | dict | None = None):
        if isinstance(app_config, IsaacAppConfig):
            self.config = app_config
        else:
            self.config = IsaacAppConfig.from_mapping(app_config)
        self._simulation_app = None
        self.world = None

    @property
    def stage(self):
        if self.world is None:
            return None
        return self.world.stage

    def start(self):
        if self._simulation_app is not None:
            return self

        SimulationApp = _import_simulation_app()

        original_argv = sys.argv
        try:
            sys.argv = [original_argv[0]]
            self._simulation_app = SimulationApp(
                self.config.simulation,
                experience=self.config.experience or "",
            )
        finally:
            sys.argv = original_argv

        if self.config.quiet_logging:
            import carb
            carb.logging.acquire_logging().set_level_threshold(carb.logging.LEVEL_ERROR)

        ros2_ext = _ros2_bridge_extension_name()
        _enable_extension(ros2_ext)

        if self.config.webrtc_streaming:
            self._simulation_app.set_setting("/app/window/drawMouse", True)
            # Enable the livestream extension that is actually present in this
            # install. Hardcoding a name (e.g. isaacsim.services.livestream.nvcf)
            # produces a spurious "Failed to resolve extension dependencies"
            # error on installs that ship the WebRTC (omni.kit.livestream.app)
            # extension instead. When launching with the streaming kit experience
            # file, the extension is already enabled by the kit, so a None result
            # here is harmless.
            livestream_ext = _resolve_livestream_extension()
            if livestream_ext is not None:
                try:
                    _enable_extension(livestream_ext)
                except Exception:
                    pass

        self.warmup(self.config.warmup_updates)
        return self

    def warmup(self, steps: int):
        if self._simulation_app is None:
            raise RuntimeError("SimulationApp has not been started")
        for _ in range(max(0, int(steps))):
            self._simulation_app.update()

    def load_stage(self, usd_path: str):
        if self._simulation_app is None:
            raise RuntimeError("SimulationApp has not been started")

        try:
            from isaacsim.core.utils.stage import open_stage
        except ImportError:
            from omni.isaac.core.utils.stage import open_stage

        open_stage(usd_path)
        self.warmup(self.config.stage_stabilization_updates)

    def create_world(self, stage_units_in_meters: float = 1.0):
        if self._simulation_app is None:
            raise RuntimeError("SimulationApp has not been started")

        try:
            from isaacsim.core.api import World
        except ImportError:
            from omni.isaac.core import World

        self.world = World(stage_units_in_meters=stage_units_in_meters)
        self.warmup(self.config.world_stabilization_updates)
        return self.world

    def hide_ui_panels(self):
        if not self.config.hide_ui_panels:
            return

        import omni.ui

        for name in self.config.hidden_windows:
            try:
                window = omni.ui.Workspace.get_window(name)
                if window is not None:
                    window.visible = False
            except Exception:
                continue

    def reset_world(self):
        if self.world is None:
            raise RuntimeError("World has not been created")
        self.world.reset()
        self.warmup(self.config.post_reset_updates)

    def stop(self):
        """Stop the physics simulation.

        Required when the USD stage's physics body composition changes (prims
        with collision/rigid-body APIs are added or removed) so that PhysX
        rebuilds a consistent scene on the next play()/reset(). Mutating such
        prims while physics is running leaves the tensor API views of existing
        articulations stale or corrupt, which silently freezes teleop.
        """
        if self.world is None:
            raise RuntimeError("World has not been created")
        self.world.stop()

    def play(self):
        if self.world is None:
            raise RuntimeError("World has not been created")
        self.world.play()

    def step(self, render: bool = True):
        if self.world is not None:
            self.world.step(render=render)
            return
        if self._simulation_app is None:
            raise RuntimeError("SimulationApp has not been started")
        self._simulation_app.update()

    def is_running(self) -> bool:
        return bool(self._simulation_app and self._simulation_app.is_running())

    def close(self):
        if self._simulation_app is None:
            return
        self._simulation_app.close()
        self._simulation_app = None
        self.world = None
