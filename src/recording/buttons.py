from __future__ import annotations

from dataclasses import dataclass

from .config import RecordingButtonConfig


@dataclass(frozen=True)
class RecordingButtonEvents:
    switch_camera: bool = False
    reset_scene: bool = False
    save_episode: bool = False
    discard_episode: bool = False


class ButtonEdgeMapper:
    def __init__(self, config: RecordingButtonConfig):
        self._config = config
        self._previous = {
            "switch_camera": False,
            "reset_scene": False,
            "save_episode": False,
            "discard_episode": False,
        }

    def update(self, latest_states: dict[str, object | None]) -> RecordingButtonEvents:
        current = {
            "switch_camera": self._resolve_symbol(self._config.switch_camera, latest_states),
            "reset_scene": self._resolve_symbol(self._config.reset_scene, latest_states),
            "save_episode": self._resolve_symbol(self._config.save_episode, latest_states),
            "discard_episode": self._resolve_symbol(self._config.discard_episode, latest_states),
        }
        events = RecordingButtonEvents(
            switch_camera=current["switch_camera"] and not self._previous["switch_camera"],
            reset_scene=current["reset_scene"] and not self._previous["reset_scene"],
            save_episode=current["save_episode"] and not self._previous["save_episode"],
            discard_episode=current["discard_episode"] and not self._previous["discard_episode"],
        )
        self._previous = current
        return events

    @staticmethod
    def _resolve_symbol(symbol: str, latest_states: dict[str, object | None]) -> bool:
        if symbol == "any_primary":
            return ButtonEdgeMapper._button_value(latest_states.get("left"), "primary") or (
                ButtonEdgeMapper._button_value(latest_states.get("right"), "primary")
            )
        if symbol == "any_secondary":
            return ButtonEdgeMapper._button_value(latest_states.get("left"), "secondary") or (
                ButtonEdgeMapper._button_value(latest_states.get("right"), "secondary")
            )
        hand, button_name = symbol.split("_", 1)
        return ButtonEdgeMapper._button_value(latest_states.get(hand), button_name)

    @staticmethod
    def _button_value(controller_state: object | None, button_name: str) -> bool:
        buttons = getattr(controller_state, "buttons", None)
        return bool(getattr(buttons, button_name, False))
