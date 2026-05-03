from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import dataclass
import json
import os
from typing import Any

import yaml


DEFAULT_MAIN_CONFIG = {
    "active_robot": "openarm",
    "paths": {
        "isaac_sim": "/home/saurabh/isaac_sim",
        "certs": {
            "cert": "certs/cert.pem",
            "key": "certs/key.pem",
        },
    },
    "server": {
        "host": "0.0.0.0",
        "websocket_port": 9999,
        "https_port": 8000,
    },
    "isaac": {
        "headless": False,
        "width": 1920,
        "height": 1080,
        "window_width": 1920,
        "window_height": 1080,
        "warmup_updates": 30,
        "stage_stabilization_updates": 50,
        "world_stabilization_updates": 20,
        "post_reset_updates": 20,
        "hide_ui_panels": True,
        "hidden_windows": [
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
        ],
    },
    "teleop": {
        "calibration_samples": 30,
        "position_scale": [1.0, 1.0, 1.0],
        "workspace_center": [0.3, 0.0, 0.3],
        "smoothing": {
            "position_alpha": 0.9,
            "orientation_alpha": 0.9,
        },
        "deadman_timeout_ms": 250,
        "hard_timeout_ms": 1000,
        "max_target_jump_m": 0.25,
    },
    "transport": {
        "log_interval_s": 3.0,
        "enable_prediction": False,
        "prediction_horizon_ms": 50,
        "jitter_buffer_frames": 0,
        "forward_queue_size": 64,
        "forward_retry_interval_s": 2.0,
        "remote_receiver": {
            "host": "0.0.0.0",
            "port": 9998,
        },
    },
    "cameras": {
        "enabled": True,
        "resolution": [480, 360],
        "publish_interval_frames": 2,
        "queue_size": 3,
        "log_errors": True,
        "error_log_interval_s": 5.0,
    },
}

MAIN_REQUIRED_KEYS = (
    "active_robot",
    "paths.isaac_sim",
    "paths.certs.cert",
    "paths.certs.key",
    "server.host",
    "server.websocket_port",
    "server.https_port",
)

ROBOT_REQUIRED_KEYS = {
    "openarm": (
        "robot_type",
        "usd",
        "urdf",
        "left_arm.frame_name",
        "left_arm.joints",
        "right_arm.frame_name",
        "right_arm.joints",
        "grippers.left_joints",
        "grippers.right_joints",
    ),
    "panda": (
        "robot_type",
        "usd",
        "arm.frame_name",
        "arm.joints",
        "grippers.joints",
        "teleop.robot_home",
        "teleop.workspace",
    ),
}

MAIN_PATH_KEYS = (
    "paths.isaac_sim",
    "paths.certs.cert",
    "paths.certs.key",
)

ROBOT_PATH_KEYS = (
    "usd",
    "urdf",
    "left_arm_config",
    "right_arm_config",
)


class ConfigError(ValueError):
    """Raised when repository configuration is missing or invalid."""


@dataclass(frozen=True)
class RuntimeConfig:
    project_root: str
    config_path: str
    main: dict[str, Any]
    robot_name: str
    robot_config_path: str
    robot: dict[str, Any]

    def get(self, dotted_key: str, *, scope: str = "main") -> Any:
        mapping = self.main if scope == "main" else self.robot
        return get_dotted_value(mapping, dotted_key)


def default_project_root() -> str:
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_runtime_config(
    config_path: str | None = None,
    *,
    robot: str | None = None,
    project_root: str | None = None,
) -> RuntimeConfig:
    repo_root = project_root or default_project_root()
    resolved_config_path = config_path or os.path.join(repo_root, "config", "config.yaml")
    main_raw = _load_yaml_file(resolved_config_path)
    main = _normalize_main_config(main_raw, repo_root)
    _validate_required_keys(main, MAIN_REQUIRED_KEYS, f"main config '{resolved_config_path}'")

    robot_name = str(robot or main.get("active_robot") or "openarm")
    robot_config_path = os.path.join(repo_root, "config", "robots", f"{robot_name}.yaml")
    if not os.path.exists(robot_config_path):
        raise ConfigError(
            f"Robot config for '{robot_name}' was not found at {robot_config_path}. "
            "Create config/robots/<robot>.yaml or choose a valid active_robot."
        )

    robot_raw = _load_yaml_file(robot_config_path)
    robot_mapping = _resolve_path_keys(dict(robot_raw), ROBOT_PATH_KEYS, repo_root)
    required_robot_keys = ROBOT_REQUIRED_KEYS.get(robot_name, ("robot_type", "usd"))
    _validate_required_keys(
        robot_mapping,
        required_robot_keys,
        f"robot config '{robot_config_path}'",
    )

    robot_type = str(robot_mapping.get("robot_type", "")).strip()
    if robot_type and robot_type != robot_name:
        raise ConfigError(
            f"Robot config mismatch: active robot '{robot_name}' points to a file with "
            f"robot_type '{robot_type}'."
        )

    return RuntimeConfig(
        project_root=repo_root,
        config_path=resolved_config_path,
        main=main,
        robot_name=robot_name,
        robot_config_path=robot_config_path,
        robot=robot_mapping,
    )


def get_dotted_value(mapping: dict[str, Any], dotted_key: str) -> Any:
    value: Any = mapping
    for part in dotted_key.split("."):
        if not isinstance(value, dict) or part not in value:
            raise ConfigError(f"Missing config key '{dotted_key}'")
        value = value[part]
    return value


def _load_yaml_file(path: str) -> dict[str, Any]:
    if not os.path.exists(path):
        raise ConfigError(f"Config file not found: {path}")
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ConfigError(f"Config file must contain a YAML mapping: {path}")
    return data


def _normalize_main_config(values: dict[str, Any], project_root: str) -> dict[str, Any]:
    merged = _deep_merge_dicts(DEFAULT_MAIN_CONFIG, values)

    isaac_values = dict(merged.get("isaac", {}))
    simulation_values = dict(isaac_values.get("simulation", {}))
    for key in ("headless", "width", "height", "window_width", "window_height"):
        if key in isaac_values:
            simulation_values[key] = isaac_values[key]
        elif key not in simulation_values:
            simulation_values[key] = DEFAULT_MAIN_CONFIG["isaac"][key]
    isaac_values["simulation"] = simulation_values
    for key, value in simulation_values.items():
        isaac_values[key] = value
    merged["isaac"] = isaac_values

    teleop_values = dict(merged.get("teleop", {}))
    smoothing_values = teleop_values.get("smoothing", {})
    if isinstance(smoothing_values, (float, int)):
        smoothing_values = {
            "position_alpha": float(smoothing_values),
            "orientation_alpha": float(smoothing_values),
        }
    elif not isinstance(smoothing_values, dict):
        smoothing_values = {}
    smoothing_defaults = deepcopy(DEFAULT_MAIN_CONFIG["teleop"]["smoothing"])
    smoothing_defaults.update(smoothing_values)
    teleop_values["smoothing"] = smoothing_defaults
    merged["teleop"] = teleop_values

    transport_values = dict(merged.get("transport", {}))
    legacy_log_period = transport_values.get("metrics_log_period_s")
    if legacy_log_period is not None and "log_interval_s" not in transport_values:
        transport_values["log_interval_s"] = legacy_log_period
    transport_values["metrics_log_period_s"] = transport_values.get("log_interval_s", 3.0)
    merged["transport"] = transport_values

    return _resolve_path_keys(merged, MAIN_PATH_KEYS, project_root)


def _deep_merge_dicts(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge_dicts(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def _resolve_path_keys(
    mapping: dict[str, Any],
    dotted_keys: tuple[str, ...],
    project_root: str,
) -> dict[str, Any]:
    resolved = deepcopy(mapping)
    for dotted_key in dotted_keys:
        try:
            value = get_dotted_value(resolved, dotted_key)
        except ConfigError:
            continue
        if isinstance(value, str) and value:
            _set_dotted_value(
                resolved,
                dotted_key,
                _resolve_repo_path(value, project_root),
            )
    return resolved


def _resolve_repo_path(path: str, project_root: str) -> str:
    if os.path.isabs(path):
        return path
    return os.path.normpath(os.path.join(project_root, path))


def _set_dotted_value(mapping: dict[str, Any], dotted_key: str, value: Any) -> None:
    current = mapping
    parts = dotted_key.split(".")
    for part in parts[:-1]:
        current = current.setdefault(part, {})
    current[parts[-1]] = value


def _validate_required_keys(mapping: dict[str, Any], dotted_keys: tuple[str, ...], label: str) -> None:
    missing = []
    for dotted_key in dotted_keys:
        try:
            value = get_dotted_value(mapping, dotted_key)
        except ConfigError:
            missing.append(dotted_key)
            continue
        if value is None or value == "":
            missing.append(dotted_key)
    if missing:
        missing_text = ", ".join(missing)
        raise ConfigError(f"Missing required keys in {label}: {missing_text}")


def _format_cli_value(value: Any) -> str:
    if isinstance(value, (dict, list, tuple)):
        return json.dumps(value)
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _build_cli_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Quest3 Streamer config helper")
    parser.add_argument(
        "command",
        choices=("get",),
        help="Command to execute",
    )
    parser.add_argument(
        "key",
        help="Dotted config key to read, e.g. server.websocket_port",
    )
    parser.add_argument(
        "--config",
        dest="config_path",
        help="Path to the main YAML config file",
    )
    parser.add_argument(
        "--robot",
        help="Override active_robot when loading config",
    )
    parser.add_argument(
        "--scope",
        choices=("main", "robot"),
        default="main",
        help="Whether to read from the main config or the selected robot config",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _build_cli_parser()
    args = parser.parse_args(argv)
    runtime = load_runtime_config(config_path=args.config_path, robot=args.robot)
    value = runtime.get(args.key, scope=args.scope)
    print(_format_cli_value(value))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
