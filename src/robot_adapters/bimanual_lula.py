from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import os

import numpy as np
import yaml

from .base import AdapterDiagnostics, CameraSpec, RobotAction, RobotAdapter


@dataclass
class _ArmRuntime:
    frame_name: str
    joint_names: list[str]
    preferred_config: np.ndarray
    last_arm_positions: np.ndarray | None = None


class BimanualLulaAdapter(RobotAdapter):
    def __init__(self, config: dict, project_root: str):
        self.config = config
        self.project_root = project_root
        self.articulation = None
        self.robot_prim_path: str | None = None
        self.left_ik_solver = None
        self.right_ik_solver = None
        self.ik_enabled = False
        self.dof_names: list[str] = []
        self.left_arm_indices: list[int] = []
        self.right_arm_indices: list[int] = []
        self.left_gripper_indices: list[int] = []
        self.right_gripper_indices: list[int] = []
        self.left_runtime = _ArmRuntime(
            frame_name=self.config["left_arm"]["frame_name"],
            joint_names=list(self.config["left_arm"]["joints"]),
            preferred_config=np.asarray(
                self.config["left_arm"]["preferred_config"],
                dtype=float,
            ).reshape(-1),
        )
        self.right_runtime = _ArmRuntime(
            frame_name=self.config["right_arm"]["frame_name"],
            joint_names=list(self.config["right_arm"]["joints"]),
            preferred_config=np.asarray(
                self.config["right_arm"]["preferred_config"],
                dtype=float,
            ).reshape(-1),
        )
        self._smoothed_left_gripper = float(self.config["grippers"]["open_position"])
        self._smoothed_right_gripper = float(self.config["grippers"]["open_position"])
        # IK tuning lives in the base so OpenArm (which does not inherit the
        # Acone override path) can still read tolerances / orientation fallback
        # from config. AconeAdapter and FFWBG2Adapter inherit these via
        # super().__init__ and stop needing to re-parse the ik block themselves.
        self.ik_config = dict(self.config.get("ik", {}))
        self.orientation_mode = str(
            self.ik_config.get("orientation_mode", "full_pose")
        ).strip()
        self.position_tolerance = _optional_float(self.ik_config.get("position_tolerance"))
        self.orientation_tolerance = _optional_float(self.ik_config.get("orientation_tolerance"))
        self.orientation_fallback_to_position = bool(
            self.ik_config.get("orientation_fallback_to_position", False)
        )
        self._diagnostics = AdapterDiagnostics(
            counters={
                "left_ik_success": 0,
                "left_ik_fail": 0,
                "right_ik_success": 0,
                "right_ik_fail": 0,
                "left_ik_step_limited": 0,
                "right_ik_step_limited": 0,
                "left_orientation_fallback": 0,
                "right_orientation_fallback": 0,
            }
        )
        self._diagnostics.details["orientation_mode"] = self.orientation_mode
        self._diagnostics.details["orientation_fallback_to_position"] = (
            self.orientation_fallback_to_position
        )
        # The two arms' IK solves are independent and Lula releases the GIL
        # during the C++ solve, so solving them concurrently in two threads
        # halves the per-frame IK wall time. The executor is created once and
        # reused for every control iteration; two workers = one per arm.
        self._ik_executor: ThreadPoolExecutor | None = None
        # Cached in initialize_joint_mappings(); whether the articulation's
        # get_joint_positions() returns a 2D array (shape (1, N)).
        self._joint_positions_are_2d = False

    @classmethod
    def from_yaml(cls, config_path: str, project_root: str):
        with open(config_path, "r", encoding="utf-8") as handle:
            return cls(yaml.safe_load(handle), project_root=project_root)

    @classmethod
    def from_mapping(cls, config: dict, project_root: str):
        return cls(config, project_root=project_root)

    @property
    def usd_path(self) -> str:
        return self._resolve_path(self.config["usd"])

    @property
    def left_workspace_offset(self) -> np.ndarray:
        return np.asarray(
            self.config["left_arm"].get("workspace_offset", [0.0, 0.15, 0.0]),
            dtype=float,
        ).reshape(3)

    @property
    def right_workspace_offset(self) -> np.ndarray:
        return np.asarray(
            self.config["right_arm"].get("workspace_offset", [0.0, -0.15, 0.0]),
            dtype=float,
        ).reshape(3)

    @property
    def gripper_threshold(self) -> float:
        return float(self.config["grippers"].get("threshold", 0.5))

    @property
    def gripper_speed(self) -> float:
        return float(self.config["grippers"]["speed"])

    @property
    def robot_label(self) -> str:
        return str(
            self.config.get("display_name")
            or self.config.get("robot_type")
            or self.__class__.__name__.replace("Adapter", "")
        )

    def load(self, world, stage):
        try:
            from isaacsim.core.prims import Articulation
        except ImportError:
            from omni.isaac.core.articulations import Articulation

        robot_prim_path = None
        for path in self.config.get("prim_search_paths", []):
            robot_prim = stage.GetPrimAtPath(path)
            if robot_prim.IsValid():
                robot_prim_path = path
                break

        if robot_prim_path is None:
            available = [str(prim.GetPath()) for prim in stage.GetPseudoRoot().GetChildren()]
            raise RuntimeError(
                f"Could not find {self.robot_label} robot in the USD stage. "
                f"Checked: {self.config.get('prim_search_paths', [])}. "
                f"Available roots: {available}"
            )

        self.robot_prim_path = robot_prim_path
        try:
            self.articulation = world.scene.add(
                Articulation(
                    prim_paths_expr=robot_prim_path,
                    name=str(self.config.get("articulation_name", self.config.get("robot_type", "robot"))),
                )
            )
        except TypeError:
            self.articulation = world.scene.add(
                Articulation(
                    prim_path=robot_prim_path,
                    name=str(self.config.get("articulation_name", self.config.get("robot_type", "robot"))),
                )
            )
        return self.articulation

    def initialize_ik(self):
        try:
            from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver
        except ImportError:
            from omni.isaac.motion_generation import LulaKinematicsSolver

        left_robot_desc_path = os.path.join(
            self._resolve_path(self.config["left_arm_config"]),
            "robot_descriptor.yaml",
        )
        right_robot_desc_path = os.path.join(
            self._resolve_path(self.config["right_arm_config"]),
            "robot_descriptor.yaml",
        )
        urdf_path = self._resolve_path(self.config["urdf"])

        try:
            self.left_ik_solver = LulaKinematicsSolver(
                robot_description_path=left_robot_desc_path,
                urdf_path=urdf_path,
            )
            self.right_ik_solver = LulaKinematicsSolver(
                robot_description_path=right_robot_desc_path,
                urdf_path=urdf_path,
            )
            self.ik_enabled = True
        except Exception as exc:
            # IK init failure freezes the arms, so surface the full cause
            # immediately rather than burying it in diagnostics.details.
            import traceback
            tb = traceback.format_exc()
            self.left_ik_solver = None
            self.right_ik_solver = None
            self.ik_enabled = False
            self._diagnostics.details["ik_init_error"] = str(exc)
            self._diagnostics.details["ik_init_traceback"] = tb
            print(f"[IK Init] FAILED to construct LulaKinematicsSolver: {exc}")
            print(f"[IK Init] urdf={urdf_path}")
            print(f"[IK Init] left_desc={left_robot_desc_path}")
            print(f"[IK Init] right_desc={right_robot_desc_path}")
            print(tb)
            return self.ik_enabled

        # Lula's root_link (e.g. "world") is assumed to sit at the USD stage
        # origin with identity orientation. When the robot prim is actually
        # placed elsewhere in the scene (the common case for OpenArm, which
        # lives under /World/Robot), the world-space teleop targets no longer
        # line up with the kinematic chain's base, and IK fails on every
        # frame. Applying the robot prim's true world transform to the
        # solver (set_robot_base_pose) re-aligns the two so world-space
        # targets resolve correctly. Opt-in via config so robots that
        # already work (acone, ffw_bg2) keep their existing behavior.
        #
        # This runs AFTER solver construction succeeds and in its own try/except
        # so a failure reading/applying the base pose can NEVER disable IK
        # entirely (which would freeze the arms). Worst case the base pose is
        # left at identity and IK may miss — strictly better than no IK.
        if self.config.get("ik", {}).get("set_solver_base_pose_from_stage", False):
            try:
                self._apply_solver_base_pose_from_stage()
            except Exception as exc:
                self._diagnostics.details["solver_base_pose_error"] = (
                    f"uncaught: {type(exc).__name__}: {exc}"
                )
        return self.ik_enabled

    def _apply_solver_base_pose_from_stage(self) -> None:
        """Read the robot prim's world transform from the USD stage and apply
        it to both IK solvers via set_robot_base_pose.

        Resolves the base-frame mismatch that occurs when the Lula descriptor's
        root_link is the URDF ``world`` link but the robot is nested under a
        non-origin prim (e.g. /World/Robot) in the scene. Reads the transform
        of the robot articulation root prim so the solver's base frame matches
        where the robot actually stands in the world.
        """
        if self.left_ik_solver is None or self.right_ik_solver is None:
            return
        if self.articulation is None:
            self._diagnostics.details["solver_base_pose_error"] = (
                "articulation not loaded; cannot read base pose"
            )
            return

        base_position, base_orientation = self._read_robot_world_pose()
        if base_position is None:
            return

        for solver, label in (
            (self.left_ik_solver, "left"),
            (self.right_ik_solver, "right"),
        ):
            try:
                solver.set_robot_base_pose(base_position, base_orientation)
            except Exception as exc:
                key = f"solver_base_pose_error_{label}"
                self._diagnostics.details[key] = str(exc)

        self._diagnostics.details["solver_base_position"] = base_position.tolist()
        self._diagnostics.details["solver_base_orientation_wxyz"] = (
            base_orientation.tolist()
        )
        print(
            f"[Init] Applied solver base pose from stage: "
            f"pos={base_position.tolist()}, "
            f"orient_wxyz={base_orientation.tolist()}"
        )

    def _read_robot_world_pose(self):
        """Return (world_position_xyz, world_orientation_wxyz) of the robot
        articulation root prim, or (None, None) if it cannot be read.

        Reads the USD world transform directly via pxr/XformCache. This avoids
        creating a throwaway isaacsim XFormPrim wrapper, whose Prim.__del__ can
        emit ``AttributeError: 'XFormPrim' object has no attribute '_callbacks'``
        when the wrapper is garbage-collected after a partially-failed init.
        """
        if self.articulation is None:
            self._diagnostics.details["solver_base_pose_error"] = (
                "articulation not loaded; cannot read base pose"
            )
            return None, None

        try:
            from pxr import Usd, UsdGeom
        except ImportError as exc:
            self._diagnostics.details["solver_base_pose_error"] = (
                f"pxr not importable to read base pose: {exc}"
            )
            return None, None

        prim = self.articulation.prim
        stage = prim.GetStage()
        robot_prim = stage.GetPrimAtPath(self.robot_prim_path)
        if not robot_prim.IsValid():
            self._diagnostics.details["solver_base_pose_error"] = (
                f"robot prim not valid: {self.robot_prim_path}"
            )
            return None, None
        transform = UsdGeom.XformCache(0.0).GetLocalToWorldTransform(robot_prim)
        translation = transform.ExtractTranslation()
        rotation = transform.ExtractRotationQuat()
        position = np.array(
            [translation[0], translation[1], translation[2]], dtype=float
        )
        # Gf.Quat stores (real, imaginary) -> already [w, x, y, z].
        imaginary = rotation.GetImaginary()
        orientation_wxyz = np.array(
            [rotation.GetReal(), imaginary[0], imaginary[1], imaginary[2]],
            dtype=float,
        )
        return position, self._normalize_wxyz(orientation_wxyz)

    @staticmethod
    def _normalize_wxyz(quat) -> np.ndarray:
        """Normalize a [w, x, y, z] quaternion; return identity on degenerate."""
        quat = np.asarray(quat, dtype=float).reshape(-1)
        if quat.size != 4:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        norm = np.linalg.norm(quat)
        if norm <= 1e-9:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        quat = quat / norm
        if quat[0] < 0.0:
            quat = -quat
        return quat

    def configure_runtime_home_from_current_pose(self, runtime_config):
        """Derive the teleop home pose from the robot's CURRENT joint positions
        via forward kinematics, instead of relying on a hand-tuned
        ``workspace_center``/``workspace_offset`` guess.

        Computes each end-effector's real world-space pose (position +
        orientation) at startup, then centers the bimanual teleop workspace on
        the midpoint of the two TCPs with each arm offset relative to that
        center. This is what makes ffw_bg2 track naturally: the controller's
        calibration pose maps exactly to where the robot's hands actually are,
        so there is no coordinate-frame guesswork. Generic for any robot whose
        Lula FK returns world-space EE poses (the OpenArm/acone/ffw case).

        Opt-in via ``ik.configure_home_from_fk`` so robots that already work
        with a tuned static workspace_center (acone) keep their behavior.
        """
        if not self.ik_config.get("configure_home_from_fk", False):
            return runtime_config
        if self.left_ik_solver is None or self.right_ik_solver is None:
            return runtime_config
        current_positions = self.get_current_joint_positions()
        if current_positions is None:
            return runtime_config

        left_home = self._home_pose_from_fk(
            self.left_ik_solver,
            self.left_runtime.frame_name,
            np.asarray(current_positions, dtype=float)[self.left_arm_indices],
        )
        right_home = self._home_pose_from_fk(
            self.right_ik_solver,
            self.right_runtime.frame_name,
            np.asarray(current_positions, dtype=float)[self.right_arm_indices],
        )
        if left_home is None or right_home is None:
            return runtime_config

        left_position, left_orientation = left_home
        right_position, right_orientation = right_home
        center = (left_position + right_position) * 0.5
        runtime_config.robot_workspace_center = center
        runtime_config.left_arm_offset = left_position - center
        runtime_config.right_arm_offset = right_position - center
        runtime_config.left_arm_home_orientation = left_orientation
        runtime_config.right_arm_home_orientation = right_orientation
        self._diagnostics.details["dynamic_left_home"] = left_position.tolist()
        self._diagnostics.details["dynamic_right_home"] = right_position.tolist()
        self._diagnostics.details["dynamic_left_home_orientation"] = left_orientation.tolist()
        self._diagnostics.details["dynamic_right_home_orientation"] = right_orientation.tolist()
        print(f"[Init] Dynamic left home: {left_position}, orientation: {left_orientation}")
        print(f"[Init] Dynamic right home: {right_position}, orientation: {right_orientation}")
        return runtime_config

    def _home_pose_from_fk(self, solver, frame_name: str, joint_positions: np.ndarray):
        """Compute the end-effector world pose via Lula forward kinematics.

        Returns (position_xyz, orientation_wxyz) or None on failure. Subclasses
        that apply a base-frame offset (e.g. FFWBG2Adapter.target_position_offset)
        override _fk_position_to_teleop_position to subtract it back out.
        """
        try:
            result = solver.compute_forward_kinematics(frame_name, joint_positions)
        except Exception as exc:
            self._diagnostics.details["dynamic_home_error"] = str(exc)
            print(f"[Init] FK home-pose computation failed: {exc}")
            return None
        if not isinstance(result, tuple) or len(result) != 2:
            return None
        root_position = np.asarray(result[0], dtype=float).reshape(-1)[:3]
        return (
            self._fk_position_to_teleop_position(root_position),
            self._wxyz_from_fk_orientation(result[1]),
        )

    def _fk_position_to_teleop_position(self, position: np.ndarray) -> np.ndarray:
        """Map an FK world position to the teleop target frame.

        Base implementation is the identity (FK world == teleop world).
        FFWBG2Adapter overrides this to subtract its target_position_offset.
        """
        return np.asarray(position, dtype=float).reshape(3)

    def _wxyz_from_fk_orientation(self, orientation) -> np.ndarray:
        """Normalize an FK orientation (rotation matrix or wxyz quat) to a
        normalized [w, x, y, z] quaternion."""
        try:
            from scipy.spatial.transform import Rotation as R
        except ImportError:
            orientation = np.asarray(orientation, dtype=float)
            if orientation.size == 9:
                # No scipy: take the diagonal-free quaternion extraction is not
                # trivial; fall back to identity rather than guess.
                return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
            return self._normalize_wxyz(orientation)
        orientation = np.asarray(orientation, dtype=float)
        if orientation.shape == (3, 3):
            quat_xyzw = R.from_matrix(orientation).as_quat()
            quat_wxyz = np.array(
                [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]],
                dtype=float,
            )
        else:
            quat_wxyz = orientation.reshape(4).astype(float)
        return self._normalize_wxyz(quat_wxyz)

    def initialize_joint_mappings(self) -> None:
        if self.articulation is None:
            raise RuntimeError("Robot articulation is not loaded")

        self.dof_names = list(self.articulation.dof_names)
        # Cache whether the articulation returns 2D joint-position arrays so that
        # apply_action can reshape the command without re-reading joint positions
        # every frame (the shape is fixed once the articulation is loaded).
        self._joint_positions_are_2d = (
            len(np.asarray(self.articulation.get_joint_positions()).shape) == 2
        )
        name_to_index = {name: index for index, name in enumerate(self.dof_names)}

        self.left_arm_indices = self._indices_for(self.left_runtime.joint_names, name_to_index)
        self.right_arm_indices = self._indices_for(self.right_runtime.joint_names, name_to_index)
        self.left_gripper_indices = self._indices_for(
            self.config["grippers"]["left_joints"],
            name_to_index,
        )
        self.right_gripper_indices = self._indices_for(
            self.config["grippers"]["right_joints"],
            name_to_index,
        )
        self._apply_gripper_drive_overrides()

    def _apply_gripper_drive_overrides(self) -> None:
        drive_config = self.config.get("grippers", {}).get("drive", {})
        if not drive_config or self.articulation is None:
            return

        try:
            from pxr import UsdPhysics
        except Exception as exc:
            self._diagnostics.details["gripper_drive_error"] = str(exc)
            return

        stage = self.articulation.prim.GetStage()
        gripper_joints = set(self.config["grippers"]["left_joints"]) | set(
            self.config["grippers"]["right_joints"]
        )
        for prim in stage.Traverse():
            if prim.GetName() not in gripper_joints:
                continue
            drive = UsdPhysics.DriveAPI.Apply(prim, "linear")
            if "stiffness" in drive_config:
                drive.CreateStiffnessAttr(float(drive_config["stiffness"]))
            if "damping" in drive_config:
                drive.CreateDampingAttr(float(drive_config["damping"]))
            if "max_force" in drive_config:
                drive.CreateMaxForceAttr(float(drive_config["max_force"]))

    def get_current_joint_positions(self):
        if self.articulation is None:
            return None
        # The Isaac Sim 6.0 deprecated Articulation deletes its `_physics_view`
        # attribute (rather than setting it to None) when PhysX invalidates the
        # tensor view after scene mutations -- e.g. spawning/removing physics
        # bodies while the simulation is running (domain randomization) or a
        # world stop/restart. Accessing the missing attribute then raises
        # AttributeError from inside is_physics_handle_valid(), which would
        # otherwise crash the control loop. Treat any handle/physics-view fault
        # as "positions unavailable this frame" and let the caller skip rather
        # than crash; reinitialize_physics_handles() should be called to rebuild
        # the view after such a scene mutation.
        try:
            pos = self.articulation.get_joint_positions()
        except AttributeError:
            return None
        if pos is not None and len(pos.shape) == 2:
            return pos[0]
        return pos

    def reinitialize_physics_handles(self) -> bool:
        """Rebuild the articulation's PhysX tensor view after the scene changes.

        Spawning or removing physics-enabled prims (nuts/bolts during domain
        randomization) while the simulation is running invalidates the cached
        ``_physics_view`` of every existing Articulation. The deprecated
        Isaac Sim 6.0 Articulation actually *deletes* the attribute in that
        case, so the next ``get_joint_positions`` raises AttributeError. Calling
        ``articulation.initialize()`` asks Isaac Sim to recreate the view from
        the current physics sim view, restoring joint reads.

        Returns True if the handle is valid after the (re-)initialize attempt.
        """
        if self.articulation is None:
            return False
        try:
            self.articulation.initialize()
            return bool(self.articulation.is_physics_handle_valid())
        except Exception:
            return False

    def compute_action(self, teleop_targets):
        current_positions = self.get_current_joint_positions()
        if current_positions is None:
            raise RuntimeError("Robot articulation joint positions are unavailable")

        target_positions = np.asarray(current_positions, dtype=float).copy()

        if self.ik_enabled:
            # Solve both arms concurrently. Each _solve_arm_ik runs a Lula C++
            # IK call (which releases the GIL) and touches only its own
            # _ArmRuntime, so the two tasks are fully independent. The shared
            # target_positions vector and the diagnostics counters are only
            # touched on this thread when we scatter/merge the results below.
            executor = self._get_ik_executor()
            left_future = executor.submit(
                self._solve_arm_ik,
                solver=self.left_ik_solver,
                runtime=self.left_runtime,
                indices=self.left_arm_indices,
                ee_target=teleop_targets.left_ee,
            )
            right_future = executor.submit(
                self._solve_arm_ik,
                solver=self.right_ik_solver,
                runtime=self.right_runtime,
                indices=self.right_arm_indices,
                ee_target=teleop_targets.right_ee,
            )
            self._scatter_arm(
                result=left_future.result(),
                indices=self.left_arm_indices,
                success_key="left_ik_success",
                fail_key="left_ik_fail",
                step_limit_key="left_ik_step_limited",
                target_positions=target_positions,
            )
            self._scatter_arm(
                result=right_future.result(),
                indices=self.right_arm_indices,
                success_key="right_ik_success",
                fail_key="right_ik_fail",
                step_limit_key="right_ik_step_limited",
                target_positions=target_positions,
            )

        self._smoothed_left_gripper = self._step_gripper(
            self._smoothed_left_gripper,
            teleop_targets.left_gripper.closed,
        )
        self._smoothed_right_gripper = self._step_gripper(
            self._smoothed_right_gripper,
            teleop_targets.right_gripper.closed,
        )

        for index in self.left_gripper_indices:
            target_positions[index] = self._smoothed_left_gripper
        for index in self.right_gripper_indices:
            target_positions[index] = self._smoothed_right_gripper

        return RobotAction(joint_positions=target_positions)

    def apply_action(self, action):
        try:
            from isaacsim.core.utils.types import ArticulationAction
        except ImportError:
            from omni.isaac.core.utils.types import ArticulationAction

        if self.articulation is None:
            raise RuntimeError("Robot articulation is not loaded")

        # Reshape the command to match the articulation's expected array shape
        # using the cached flag instead of re-reading joint positions every
        # frame (a redundant GPU read on the hot path).
        pos = action.joint_positions
        if pos is not None and self._joint_positions_are_2d and len(pos.shape) == 1:
            pos = pos.reshape(1, -1)

        action_to_apply = ArticulationAction(joint_positions=pos)

        # In Isaac Sim 6.0, the deprecated Articulation class checks for 'joint_names'
        # but the ArticulationAction class no longer defines it, causing an AttributeError.
        if not hasattr(action_to_apply, "joint_names"):
            action_to_apply.joint_names = None

        # The same _physics_view invalidation that get_current_joint_positions
        # guards against can strike here if a prim-deletion event arrives
        # between the joint read and this apply. A dropped command for one frame
        # is strictly preferable to crashing the whole teleop session; the next
        # reinitialize_physics_handles()/fresh frame recovers the handle.
        try:
            self.articulation.apply_action(action_to_apply)
        except AttributeError:
            pass

    def get_joint_names(self) -> list[str]:
        return list(self.dof_names)

    def get_camera_specs(self) -> dict[str, CameraSpec]:
        return {
            name: CameraSpec(
                name=name,
                prim_path=spec["prim_path"],
                topic=spec["topic"],
            )
            for name, spec in self.config.get("cameras", {}).items()
        }

    def get_viewport_cameras(self) -> list[tuple[str, str]]:
        cameras = [("Perspective", "/OmniverseKit_Persp")]
        for name, spec in self.config.get("cameras", {}).items():
            cameras.append((name.replace("_", " ").title(), spec["prim_path"]))
        return cameras

    def get_diagnostics(self) -> AdapterDiagnostics:
        diagnostics = AdapterDiagnostics(
            counters=dict(self._diagnostics.counters),
            details=dict(self._diagnostics.details),
        )
        diagnostics.details["robot_prim_path"] = self.robot_prim_path
        diagnostics.details["ik_enabled"] = self.ik_enabled
        return diagnostics

    def reset_runtime_state(self) -> None:
        self.left_runtime.last_arm_positions = None
        self.right_runtime.last_arm_positions = None
        self._smoothed_left_gripper = float(self.config["grippers"]["open_position"])
        self._smoothed_right_gripper = float(self.config["grippers"]["open_position"])

    def get_recording_vector(
        self,
        *,
        vector_config,
        current_joint_positions,
        commanded_action,
        teleop_targets=None,
    ) -> tuple[list[str], np.ndarray]:
        mode = getattr(vector_config, "mode", None)
        if mode is None and isinstance(vector_config, dict):
            mode = vector_config.get("mode", "articulation_joints")
        if mode != "named_groups":
            return super().get_recording_vector(
                vector_config=vector_config,
                current_joint_positions=current_joint_positions,
                commanded_action=commanded_action,
                teleop_targets=teleop_targets,
            )

        config_mapping = (
            vector_config.to_mapping() if hasattr(vector_config, "to_mapping") else dict(vector_config)
        )
        group_names = list(config_mapping.get("groups", []))
        joint_groups = self.config.get("recording", {}).get("joint_groups", {})
        if not group_names:
            raise ValueError("Recording vector config must define at least one group")

        state_source = np.asarray(current_joint_positions, dtype=float).reshape(-1)
        action_source = np.asarray(commanded_action.joint_positions, dtype=float).reshape(-1)
        use_action_source = config_mapping.get("key") == "action"

        names: list[str] = []
        values: list[float] = []
        for group_name in group_names:
            group = joint_groups.get(group_name)
            if group is None:
                raise KeyError(f"Unknown recording joint group '{group_name}'")

            source = str(group.get("source", "articulation"))
            if source == "articulation":
                joint_names = list(group.get("joints", []))
                vector = self.get_articulation_vector_by_joint_names(
                    joint_names,
                    action_source if use_action_source else state_source,
                )
                names.extend(joint_names)
                values.extend(vector.tolist())
                continue

            if source == "adapter" and group.get("value") == "normalized_gripper":
                side = str(group.get("side", "")).strip().lower()
                names.append(str(group.get("name", f"{side}_gripper")))
                raw_value = self._recording_gripper_source_value(
                    side=side,
                    source_positions=action_source if use_action_source else state_source,
                )
                values.append(self._normalize_gripper(raw_value))
                continue

            raise ValueError(f"Unsupported recording group definition for '{group_name}': {group}")

        return names, np.asarray(values, dtype=np.float32)

    def _get_ik_executor(self) -> ThreadPoolExecutor:
        """Lazily create (and reuse) the IK thread pool. Two workers = one per arm."""
        if self._ik_executor is None:
            self._ik_executor = ThreadPoolExecutor(
                max_workers=2,
                thread_name_prefix="bimanual-lula-ik",
            )
        return self._ik_executor

    def close(self) -> None:
        """Shut down the IK thread pool. Safe to call multiple times."""
        if self._ik_executor is not None:
            self._ik_executor.shutdown(wait=False)
            self._ik_executor = None

    @dataclass
    class _ArmIKResult:
        """Outcome of a single arm IK solve, produced by a worker thread.

        All shared state (target_positions, diagnostics counters) is updated on
        the control thread from this pure result, so the worker touches no
        mutable shared structures.
        """
        success: bool
        step_limited: bool
        arm_positions: np.ndarray | None  # post-step-limit solution (on success)
        fallback: np.ndarray | None       # last successful solution (on failure)
        indices: list[int]
        # Acone uses position-only fallback after full-pose IK fails; other
        # adapters never set this. Tracked here so the (main-thread) scatter
        # path can bump the diagnostics counter without touching it from a worker.
        orientation_fallback: bool = False

    def _solve_arm_ik(
        self,
        solver,
        runtime: _ArmRuntime,
        indices: list[int],
        ee_target,
    ):
        """Worker: run one arm's IK solve + joint-step limit.

        Thread-safe: only touches the per-arm `runtime` (last_arm_positions).
        Returns a pure result; no shared array or counter mutation happens here.
        """
        if solver is None or not ee_target.valid:
            return self._ArmIKResult(
                success=False,
                step_limited=False,
                arm_positions=None,
                fallback=runtime.last_arm_positions,
                indices=list(indices),
            )

        warm_start = (
            runtime.last_arm_positions
            if runtime.last_arm_positions is not None
            else runtime.preferred_config
        )
        # Optional tolerances and orientation fallback are driven by the
        # adapter's ik config block. AconeAdapter/FFWBG2Adapter override this
        # method entirely, so changing the base behavior only affects robots
        # that solve through OpenArmAdapter directly (i.e. OpenArm itself).
        actions, success = self._compute_ik(
            solver=solver,
            runtime=runtime,
            ee_target=ee_target,
            warm_start=warm_start,
            orientation=self._target_orientation(ee_target),
        )

        orientation_fallback = False
        if not success and self.orientation_fallback_to_position:
            actions, success = self._compute_ik(
                solver=solver,
                runtime=runtime,
                ee_target=ee_target,
                warm_start=warm_start,
                orientation=None,
            )
            orientation_fallback = success

        if not success:
            return self._ArmIKResult(
                success=False,
                step_limited=False,
                arm_positions=None,
                fallback=runtime.last_arm_positions,
                indices=list(indices),
                orientation_fallback=False,
            )

        arm_positions = np.asarray(actions, dtype=float).reshape(-1)[: len(indices)]
        limited, step_limited = self._limit_arm_step_pure(
            runtime=runtime,
            arm_positions=arm_positions,
        )
        runtime.last_arm_positions = limited.copy()
        return self._ArmIKResult(
            success=True,
            step_limited=step_limited,
            arm_positions=limited,
            fallback=None,
            indices=list(indices),
            orientation_fallback=orientation_fallback,
        )

    def _scatter_arm(
        self,
        result,
        indices: list[int],
        success_key: str,
        fail_key: str,
        step_limit_key: str,
        target_positions: np.ndarray,
    ) -> None:
        """Main-thread: merge a worker's IK result into the shared command
        vector and update diagnostics counters."""
        if result.success and result.arm_positions is not None:
            self._diagnostics.counters[success_key] += 1
            if result.step_limited:
                self._diagnostics.counters[step_limit_key] = (
                    self._diagnostics.counters.get(step_limit_key, 0) + 1
                )
            if getattr(result, "orientation_fallback", False):
                fallback_key = success_key.replace("_ik_success", "_orientation_fallback")
                self._diagnostics.counters[fallback_key] = (
                    self._diagnostics.counters.get(fallback_key, 0) + 1
                )
            for offset, joint_index in enumerate(indices):
                if offset < result.arm_positions.size:
                    target_positions[joint_index] = result.arm_positions[offset]
            return

        self._diagnostics.counters[fail_key] += 1
        if result.fallback is None:
            return
        for offset, joint_index in enumerate(indices):
            if offset < result.fallback.size:
                target_positions[joint_index] = result.fallback[offset]

    @property
    def max_ik_joint_step_rad(self) -> float | None:
        value = self.config.get("safety", {}).get(
            "max_ik_joint_step_rad",
            self.config.get("ik", {}).get("max_ik_joint_step_rad", 0.04),
        )
        if value is None:
            return None
        return max(0.0, float(value))

    def _compute_ik(self, *, solver, runtime, ee_target, warm_start, orientation):
        """Call the Lula solver with the adapter's configured tolerances.

        Centralized here so subclasses (e.g. FFWBG2Adapter) can override just
        the target translation (e.g. apply a base-frame offset) while reusing
        the tolerance/orientation handling.
        """
        return solver.compute_inverse_kinematics(
            target_position=ee_target.position_xyz,
            target_orientation=orientation,
            frame_name=runtime.frame_name,
            warm_start=warm_start,
            position_tolerance=self.position_tolerance,
            orientation_tolerance=self.orientation_tolerance,
        )

    def _target_orientation(self, ee_target):
        """Resolve the orientation target for IK based on orientation_mode.

        - ``full_pose`` (default): pass the teleop orientation through.
        - ``position_only``/``none``: pass None so Lula solves position-only.
        """
        if self.orientation_mode in {"position_only", "position-only", "none"}:
            return None
        return ee_target.orientation_wxyz

    def _limit_arm_step_pure(
        self,
        *,
        runtime: _ArmRuntime,
        arm_positions: np.ndarray,
    ) -> tuple[np.ndarray, bool]:
        """Pure joint-step limiter. Returns (limited_positions, step_limited).

        Does not mutate any shared state, so it is safe to call from an IK
        worker thread. The caller is responsible for updating diagnostics.
        """
        max_step = self.max_ik_joint_step_rad
        if max_step is None or max_step <= 0.0 or runtime.last_arm_positions is None:
            return arm_positions.copy(), False

        previous = np.asarray(runtime.last_arm_positions, dtype=float).reshape(-1)
        candidate = np.asarray(arm_positions, dtype=float).reshape(-1)
        if previous.size != candidate.size:
            return candidate.copy(), False

        delta = candidate - previous
        limited_delta = np.clip(delta, -max_step, max_step)
        step_limited = bool(np.any(np.abs(delta - limited_delta) > 1e-9))
        return previous + limited_delta, step_limited

    def _step_gripper(self, current: float, closed: bool) -> float:
        open_position = float(self.config["grippers"]["open_position"])
        closed_position = float(self.config["grippers"]["closed_position"])
        target = closed_position if closed else open_position

        if current < target:
            return min(current + self.gripper_speed, target)
        if current > target:
            return max(current - self.gripper_speed, target)
        return current

    def _resolve_path(self, path: str) -> str:
        if os.path.isabs(path):
            return path
        return os.path.join(self.project_root, path)

    @staticmethod
    def _indices_for(joint_names: list[str], name_to_index: dict[str, int]) -> list[int]:
        return [name_to_index[name] for name in joint_names if name in name_to_index]

    def _recording_gripper_source_value(self, *, side: str, source_positions: np.ndarray) -> float:
        if side == "left":
            indices = self.left_gripper_indices
        elif side == "right":
            indices = self.right_gripper_indices
        else:
            raise ValueError(f"Unsupported gripper side '{side}'")
        if not indices:
            raise RuntimeError(f"Recording gripper indices for side '{side}' are not initialized")
        values = np.asarray(source_positions, dtype=float).reshape(-1)[indices]
        return float(np.mean(values))

    def _normalize_gripper(self, raw_value: float) -> float:
        open_position = float(self.config["grippers"]["open_position"])
        closed_position = float(self.config["grippers"]["closed_position"])
        denom = closed_position - open_position
        if abs(denom) < 1e-9:
            return 0.0
        normalized = (float(raw_value) - open_position) / denom
        return float(np.clip(normalized, 0.0, 1.0))


def _optional_float(value) -> float | None:
    if value in (None, ""):
        return None
    return float(value)
