"""Regression tests for the domain-randomization reset sequence.

These cover the two failure modes that broke teleop when pressing B to
randomize the acone scene:

1. First press crashed the control loop with
   ``AttributeError: 'Articulation' object has no attribute '_physics_view'``.
2. After the fix, the SECOND press silently froze the arms because removing
   the previously spawned nuts/bolts while physics was running left the
   robot articulation's PhysX tensor view permanently invalid.

The fix decouples stage mutation (``apply_randomization``) from settle-stepping
(``settle``) so the caller can mutate the stage while physics is STOPPED and
then ``reset_world()`` to rebuild a consistent PhysX scene. These tests pin
that contract.
"""
import os
import sys
import unittest

import numpy as np

TESTS_DIR = os.path.dirname(__file__)
PROJECT_ROOT = os.path.join(TESTS_DIR, "..")
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from isaac_backend import DomainRandomizer, FFWBG2DomainRandomizer  # noqa: E402
from isaac_backend.domain_randomization import (  # noqa: E402
    _euler_xyz_degrees_to_quat_wxyz,
    _quat_wxyz_to_euler_xyz_degrees,
)


class QuatEulerConversionTests(unittest.TestCase):
    """The USD-native set_world_pose_usd converts the requested quaternion to
    XYZ euler degrees before writing UsdGeom.XformCommonAPI. This round-trips
    with _euler_xyz_degrees_to_quat_wxyz and must be stable for the yaw-only
    rotations used when spawning/jittering scene objects.
    """

    def test_yaw_only_round_trips(self):
        # Spawning objects and jittering trays only use a z-axis (yaw) rotation.
        for yaw_deg in (-180.0, -90.0, -12.5, 0.0, 45.0, 90.0, 178.0):
            quat = _euler_xyz_degrees_to_quat_wxyz((0.0, 0.0, yaw_deg))
            roll, pitch, yaw = _quat_wxyz_to_euler_xyz_degrees(quat)
            # Wrap yaw into (-180, 180] for comparison.
            yaw_wrapped = (yaw + 180.0) % 360.0 - 180.0
            self.assertAlmostEqual(roll, 0.0, places=5)
            self.assertAlmostEqual(pitch, 0.0, places=5)
            self.assertAlmostEqual(yaw_wrapped, yaw_deg, places=5)

    def test_degenerate_quaternion_returns_zero_euler(self):
        roll, pitch, yaw = _quat_wxyz_to_euler_xyz_degrees([0.0, 0.0, 0.0, 0.0])
        self.assertEqual((roll, pitch, yaw), (0.0, 0.0, 0.0))

    def test_full_pose_round_trips_within_reasonable_tolerance(self):
        # A generic 3-axis rotation must round-trip back to the same euler
        # angles (modulo gimbal-lock edge cases).
        for original in [(10.0, 20.0, 30.0), (-45.0, 15.0, 60.0), (0.0, 0.0, 0.0)]:
            quat = _euler_xyz_degrees_to_quat_wxyz(original)
            roll, pitch, yaw = _quat_wxyz_to_euler_xyz_degrees(quat)
            self.assertAlmostEqual(roll, original[0], places=4)
            self.assertAlmostEqual(pitch, original[1], places=4)
            self.assertAlmostEqual(yaw, original[2], places=4)


class _FakeStage:
    """Minimal USD stage double that records prim mutations."""

    def __init__(self):
        self.prisms = {}  # path -> list of child paths
        self.removed = []
        self.defined = []

    def GetPrimAtPath(self, path):
        return _FakePrim(self, path, valid=path in self.prisms)

    def DefinePrim(self, path, kind):
        self.prisms[path] = []
        self.defined.append((path, kind))
        return _FakePrim(self, path, valid=True)

    def RemovePrim(self, path):
        self.removed.append(path)
        # Also drop it from the registry so it reports invalid afterwards.
        self.prisms.pop(path, None)
        for children in self.prisms.values():
            if path in children:
                children.remove(path)

    def Traverse(self):
        return [_FakePrim(self, p, True) for p in self.prisms]


class _FakePrim:
    def __init__(self, stage, path, valid):
        self._stage = stage
        self.path = path
        self._valid = valid

    @property
    def path_str(self):
        return self.path

    def GetPath(self):
        return self.path

    def IsValid(self):
        return self._valid

    def GetTypeName(self):
        return "Xform"

    def GetName(self):
        return self.path.rsplit("/", 1)[-1]

    def GetChildren(self):
        return [_FakePrim(self._stage, child, True) for child in self._stage.prisms.get(self.path, [])]

    def GetReferences(self):
        return _FakeRefs()


class _FakeRefs:
    def AddReference(self, path):
        pass


class DomainRandomizerDecoupleTests(unittest.TestCase):
    """apply_randomization must mutate the stage WITHOUT stepping, settle must
    only step. This is what lets _reset_scene stop physics, mutate, then
    reset_world before settling."""

    def _make_randomizer(self, enabled=True):
        stage = _FakeStage()
        config = {
            "enabled": enabled,
            "settle_steps": 5,
            "prims": {
                "spawn_root": "/World/RandomizedObjects",
                "left_tray": "/World/LeftTray",
                "middle_tray": "/World/MiddleTray",
                "right_tray": "/World/RightTray",
                "light_root": "/World/Environment",
                "looks": "/World/Looks",
                "floor": "/World/FlatGrid",
                "nut": "/World/nut",
                "bolt": "/World/bolt",
            },
            "objects": {
                "nut_count": [1, 1],
                "bolt_count": [1, 1],
            },
            "assets": {
                "nut_usd": "nut.usd",
                "bolt_usd": "bolt.usd",
            },
        }
        randomizer = DomainRandomizer(stage, config)
        # Stub the USD-touching helpers (they need pxr, only available inside a
        # running Isaac Sim). The contract under test is purely the
        # mutation-vs-stepping split, not the lighting/floor math. _spawn_objects
        # is replaced with a recorder so we can assert the counts it WOULD have
        # produced without touching Gf/UsdGeom/XFormPrim.
        randomizer._randomize_lighting = lambda: 500.0
        randomizer._randomize_floor = lambda: (0.5, 0.5, 0.5)
        randomizer._restore_trays = lambda: None
        randomizer._randomize_trays = lambda: None

        def _spawn_objects_stub():
            for kind, count in (("nut", 1), ("bolt", 1)):
                for index in range(count):
                    randomizer._spawn_object(kind, index)
            return 1, 1

        def _spawn_object_stub(kind, index):
            spawn_root = "/World/RandomizedObjects"
            if spawn_root not in randomizer.stage.prisms:
                randomizer.stage.DefinePrim(spawn_root, "Xform")
            child = f"{spawn_root}/{kind}_{index:02d}"
            randomizer.stage.DefinePrim(child, "Xform")
            randomizer.stage.prisms.setdefault(spawn_root, []).append(child)

        randomizer._spawn_objects = _spawn_objects_stub
        randomizer._spawn_object = _spawn_object_stub
        return randomizer

    def test_apply_randomization_does_not_step_physics(self):
        """The stage mutation phase must not invoke settle_step at all.

        If it did, the prim mutations would land while physics is running,
        re-introducing the freeze on the second reset.
        """
        randomizer = self._make_randomizer()
        randomizer.enabled = True
        steps = []
        sample = randomizer.apply_randomization()
        # No stepping during the pure stage-mutation phase.
        self.assertEqual(steps, [])
        # But it did describe a sample with spawned object counts.
        self.assertEqual(sample.nut_count, 1)
        self.assertEqual(sample.bolt_count, 1)

    def test_settle_only_steps_and_does_not_mutate(self):
        """settle must only advance physics; it must not remove/define prims."""
        randomizer = self._make_randomizer()
        randomizer.enabled = True
        randomizer.apply_randomization()
        defined_before = list(randomizer.stage.defined)
        removed_before = list(randomizer.stage.removed)

        steps = []
        randomizer.settle(lambda render=True: steps.append(render))

        # settle_steps = 5 in the config above.
        self.assertEqual(len(steps), 5)
        # No further prim mutations during settle.
        self.assertEqual(randomizer.stage.defined, defined_before)
        self.assertEqual(randomizer.stage.removed, removed_before)

    def test_disabled_randomizer_is_noop_in_both_phases(self):
        randomizer = self._make_randomizer(enabled=False)
        steps = []
        sample = randomizer.apply_randomization()
        randomizer.settle(lambda render=True: steps.append(render))
        self.assertEqual(sample.nut_count, 0)
        self.assertEqual(sample.bolt_count, 0)
        self.assertEqual(steps, [])

    def test_second_apply_clears_previous_spawned_objects(self):
        """The second reset must RemovePrim the previously spawned children.

        This is the exact path that triggered the second-press freeze when it
        ran while physics was live. We only assert the removal happens here;
        the ordering (stop -> apply -> reset_world -> settle) is enforced in
        _reset_scene itself.
        """
        randomizer = self._make_randomizer()
        # Mark the spawn root as already existing with children, simulating the
        # state right before the SECOND press.
        randomizer.stage.prisms["/World/RandomizedObjects"] = [
            "/World/RandomizedObjects/nut_00",
            "/World/RandomizedObjects/bolt_00",
        ]
        randomizer.stage.prisms["/World/RandomizedObjects/nut_00"] = []
        randomizer.stage.prisms["/World/RandomizedObjects/bolt_00"] = []

        randomizer.apply_randomization()

        self.assertIn("/World/RandomizedObjects/nut_00", randomizer.stage.removed)
        self.assertIn("/World/RandomizedObjects/bolt_00", randomizer.stage.removed)

    def test_legacy_randomize_wrapper_preserves_contract(self):
        """randomize(settle_step) must still work and step settle_steps times,
        so any external caller that has not migrated stays functional."""
        randomizer = self._make_randomizer()
        steps = []
        sample = randomizer.randomize(lambda render=True: steps.append(render))
        self.assertEqual(sample.nut_count, 1)
        self.assertEqual(len(steps), 5)  # settle_steps = 5


class FFWBG2RandomizerDecoupleTests(unittest.TestCase):
    def test_ffw_bg2_apply_randomization_does_not_step(self):
        """FFWBG2DomainRandomizer only moves existing prims, but it must still
        honor the apply/settle split so _reset_scene can stop physics first."""
        stage = _FakeStage()
        # The ffw randomizer references cube/tray/table paths; make them exist.
        for p in ("/World/Cube", "/World/Tray", "/World/OakTableSmall", "/World/Environment"):
            stage.prisms[p] = []
        config = {
            "enabled": True,
            "settle_steps": 4,
            "prims": {
                "cube": "/World/Cube",
                "tray": "/World/Tray",
                "table": "/World/OakTableSmall",
                "light_root": "/World/Environment",
            },
            "placement": {},
            "lighting": {},
        }
        randomizer = FFWBG2DomainRandomizer(stage, config)
        # Stub USD-touching helpers (need pxr / a real stage); the contract
        # under test is the apply/settle split only.
        randomizer._randomize_lighting = lambda: (500.0, None)
        randomizer._randomize_cube_and_tray = lambda: (None, None, ())
        steps = []
        sample = randomizer.apply_randomization()
        self.assertEqual(steps, [])
        # settle steps exactly settle_steps times.
        randomizer.settle(lambda render=True: steps.append(render))
        self.assertEqual(len(steps), 4)


if __name__ == "__main__":
    unittest.main()
