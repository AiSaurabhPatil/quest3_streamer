from __future__ import annotations

import numpy as np

from .bimanual_lula import BimanualLulaAdapter


class AconeAdapter(BimanualLulaAdapter):
    """Bimanual custom-URDF adapter for the AC One robot."""

    def __init__(self, config: dict, project_root: str):
        super().__init__(config, project_root)
        # IK config (ik_config, orientation_mode, tolerances, fallback) is now
        # parsed by the BimanualLulaAdapter base. Acone historically defaulted to
        # position-only IK, so preserve that default when the config omits an
        # explicit orientation_mode.
        if "orientation_mode" not in self.ik_config:
            self.orientation_mode = "position_only"
            self._diagnostics.details["orientation_mode"] = self.orientation_mode
        # orientation_fallback_to_position defaults to True for Acone (the base
        # default is False) so a full-pose miss degrades to position-only.
        if "orientation_fallback_to_position" not in self.ik_config:
            self.orientation_fallback_to_position = True
            self._diagnostics.details["orientation_fallback_to_position"] = True
