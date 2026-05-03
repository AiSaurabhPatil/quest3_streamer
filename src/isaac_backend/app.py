from __future__ import annotations

from dataclasses import dataclass, field


DEFAULT_SIMULATION_CONFIG = {
    "headless": False,
    "width": 1920,
    "height": 1080,
    "window_width": 1920,
    "window_height": 1080,
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
    warmup_updates: int = 30
    stage_stabilization_updates: int = 50
    world_stabilization_updates: int = 20
    post_reset_updates: int = 20
    hide_ui_panels: bool = True
    hidden_windows: tuple[str, ...] = DEFAULT_HIDDEN_WINDOWS

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
            warmup_updates=int(values.get("warmup_updates", 30)),
            stage_stabilization_updates=int(values.get("stage_stabilization_updates", 50)),
            world_stabilization_updates=int(values.get("world_stabilization_updates", 20)),
            post_reset_updates=int(values.get("post_reset_updates", 20)),
            hide_ui_panels=bool(values.get("hide_ui_panels", True)),
            hidden_windows=hidden_windows,
        )


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

        from omni.isaac.kit import SimulationApp

        self._simulation_app = SimulationApp(self.config.simulation)

        from omni.isaac.core.utils.extensions import enable_extension

        enable_extension("omni.isaac.ros2_bridge")
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

        from omni.isaac.core.utils.stage import open_stage

        open_stage(usd_path)
        self.warmup(self.config.stage_stabilization_updates)

    def create_world(self, stage_units_in_meters: float = 1.0):
        if self._simulation_app is None:
            raise RuntimeError("SimulationApp has not been started")

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
