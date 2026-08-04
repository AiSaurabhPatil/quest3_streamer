# TeleSim Clutched Teleoperation — GPT-5.5 Implementation Spec

## Implementation Context

### Project Context

TeleSim is an Isaac Sim teleoperation and dataset-collection system. It currently receives 6-DoF poses and button states from Meta Quest controllers, maps controller motion to robot end-effector targets, solves arm IK through the robot adapter, applies joint commands, and records LeRobot-compatible episodes.

The current default teleoperation mapping is continuous: once controller calibration is complete, controller motion continuously changes the robot end-effector target.

### Problem Being Solved

Continuous absolute/relative teleoperation is suitable for collecting full expert demonstrations, but it is unsuitable for policy intervention workflows.

During an autonomous rollout:

- the policy may move the robot while the human controllers remain elsewhere
- directly switching to the existing teleoperation target can cause an end-effector jump
- the operator needs motion to begin only while a button is held
- releasing the button must stop human motion safely
- the system must distinguish policy actions, human corrections, and the action actually executed
- the collected dataset must identify intervention segments for later DAgger, recovery-and-correction, or behavior-cloning training

### Feature Being Implemented

Add clutch-based human intervention to the modular TeleSim runtime.

The clutch is a hold-to-control mechanism:

```text
button released:
  human controller movement does not move the selected robot arm

button pressed:
  selected robot arm follows controller motion relative to the exact pose
  where takeover started

button released again:
  human authority ends
```

Default mapping:

```text
left controller stick click  -> left arm authority
right controller stick click -> right arm authority
both pressed                 -> bimanual authority
```

This supports:

```text
continuous mode:
  preserve existing TeleSim behavior

clutched mode:
  human-only teleoperation with movement gated by clutch buttons

policy_intervention mode:
  autonomous policy normally controls the robot
  human temporarily overrides one or both arms
```

### Zero-Jump Takeover Strategy

When a clutch button changes from released to pressed:

1. read the latest Quest controller pose
2. read the robot’s current joint positions
3. compute the selected end-effector’s measured pose using Lula FK
4. store both poses as takeover anchors
5. calculate all later human targets as motion relative to those anchors

At the exact engagement frame:

```text
controller displacement = zero
target end-effector pose = measured current end-effector pose
```

Therefore, engaging human control must not move the robot by itself.

The original calibration reference remains used only by `continuous` mode. Clutch modes use a new dynamic reference every time the operator engages control.

### Runtime Control Flow

The intended control path is:

```text
Quest state
  -> clutch edge detection
  -> measured FK only on engagement
  -> human relative-pose target generation
  -> IK converts human target to a candidate joint action

policy provider
  -> policy candidate joint action

human candidate + policy candidate + intervention state
  -> action arbiter
  -> executed joint action
  -> robot
  -> recorder
```

Authority rules:

```text
continuous:
  execute human candidate

clutched:
  active human arm -> human candidate
  inactive arm     -> hold last executed command

policy_intervention:
  active human arm -> human candidate
  inactive arm     -> policy candidate
```

When human control is released in policy-intervention mode, affected joints blend back to the policy command instead of switching instantly.

### Recording Outcome

When intervention recording is enabled, each recorded frame must contain:

```text
action:
  command actually applied to the robot

action.human:
  human candidate command

action.policy:
  policy candidate command

control.source:
  POLICY, HUMAN, MIXED, or HOLD

intervention.left / intervention.right:
  which arm is currently human-controlled

intervention.id:
  identifier shared by one contiguous takeover interval
```

This makes the dataset usable for:

- human-gated DAgger
- recovery-and-correction training
- intervention-only behavior cloning
- policy-error analysis
- separating autonomous rollout data from corrective expert data

### Expected Final Outcome

After implementation:

1. Existing continuous teleoperation remains unchanged by default.
2. In clutched mode, moving a Quest controller while the clutch is released has no effect.
3. Pressing the clutch takes control without an arm jump.
4. Releasing the clutch holds the current robot command.
5. Re-engaging creates a new anchor from the robot’s current measured pose.
6. In policy-intervention mode, human control can override one or both arms.
7. Releasing human control blends safely back to the policy.
8. Invalid policy output, missing FK, or controller timeout results in hold behavior rather than unsafe motion.
9. Recorded actions and intervention metadata correctly represent what the policy proposed, what the human proposed, and what the robot executed.

### Scope

Implement the reusable control architecture and generic policy-provider interface.

Do not implement:

- a specific OpenPI, Pi0, ACT, Diffusion Policy, or VLA client
- policy image preprocessing
- action-chunk inference
- asynchronous model serving
- a new Quest transport protocol
- changes to the legacy monolithic OpenPI script

## Rules

- Modify only the modular runtime under `src/launch`, `src/teleop_core`, `src/robot_adapters`, and `src/recording`.
- Do not modify `src/isaac_openarm_teleop_openpi.py`.
- Preserve existing behavior by default.
- Do not add model-specific policy inference.
- Do not add tests. Run only the syntax checks listed per file.
- Call measured EE FK only on clutch rising edges.
- Reject takeover if measured EE FK is unavailable. Never substitute configured home pose.
- Record the executed action under the existing action key.

Default configuration:

```yaml
teleop:
  control_mode: continuous
recording:
  intervention:
    enabled: false
```

Modes:

```text
continuous:
  existing human teleoperation; clutch ignored

clutched:
  released -> hold last executed action
  held -> human relative-pose control

policy_intervention:
  released -> policy action
  held -> human override
  release -> joint-space blend back to policy
```

Defaults:

```text
button: stick_click
arbitration: per_hand
policy re-entry: 200 ms
gripper: existing trigger/squeeze logic
```

Relative takeover:

```python
delta = current_controller_pos - anchor_controller_pos
target_pos = anchor_ee_pos + (
    frame_transform.position_offset_to_robot(delta) * pos_scale
)
target_rot = _relative_target_orientation(
    current_orientation=current_controller_rot_robot,
    reference_orientation=anchor_controller_rot_robot,
    home_orientation=anchor_ee_rot,
)
```

---

# 1. Create `src/teleop_core/intervention.py`

Add imports:

```python
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum, IntEnum
from typing import Mapping
import numpy as np
from .controller_state import ControllerState
```

Add:

```python
class ControlMode(str, Enum):
    CONTINUOUS = "continuous"
    CLUTCHED = "clutched"
    POLICY_INTERVENTION = "policy_intervention"


class ControlSource(IntEnum):
    POLICY = 0
    HUMAN = 1
    MIXED = 2
    HOLD = 3
```

Add:

```python
@dataclass(frozen=True)
class ClutchConfig:
    enabled: bool = False
    button: str = "stick_click"
    arbitration: str = "per_hand"
    policy_reentry_blend_s: float = 0.2
```

Implement `ClutchConfig.from_mapping(values)`.

Validation:
- `button`: `primary`, `secondary`, `menu`, `stick_click`
- `arbitration`: `global`, `per_hand`
- `policy_reentry_blend_s >= 0`
- raise `ValueError` on invalid values

Add:

```python
@dataclass(frozen=True)
class EndEffectorPose:
    position_xyz: np.ndarray
    orientation_wxyz: np.ndarray
    valid: bool = True
```

In `__post_init__`:
- reshape position to `(3,)`
- reshape quaternion to `(4,)`
- normalize quaternion
- use identity quaternion for invalid/degenerate input
- copy arrays with `object.__setattr__`

Add:

```python
@dataclass(frozen=True)
class HandClutchInput:
    pressed: bool = False
    rising: bool = False
    falling: bool = False


@dataclass(frozen=True)
class BimanualClutchInput:
    left: HandClutchInput = field(default_factory=HandClutchInput)
    right: HandClutchInput = field(default_factory=HandClutchInput)

    @property
    def any_rising(self) -> bool:
        return self.left.rising or self.right.rising
```

Add `ClutchInputMapper`:
- constructor stores `ClutchConfig`
- `_previous = {"left": False, "right": False}`
- `reset()` clears previous state
- `update(states)` reads `getattr(state.buttons, config.button, False)`
- disabled: return all-false input and reset previous state
- `per_hand`: calculate edges independently
- `global`: OR physical left/right, then return identical logical state for both hands

Add:

```python
@dataclass
class ClutchAnchor:
    controller_position_xyz: np.ndarray
    controller_orientation_wxyz: np.ndarray
    ee_position_xyz: np.ndarray
    ee_orientation_wxyz: np.ndarray
    engaged_at_s: float


@dataclass(frozen=True)
class HandInterventionStatus:
    active: bool = False
    forced_hold: bool = False


@dataclass(frozen=True)
class InterventionStatus:
    left: HandInterventionStatus = field(default_factory=HandInterventionStatus)
    right: HandInterventionStatus = field(default_factory=HandInterventionStatus)
    intervention_id: int = -1

    @property
    def active(self) -> bool:
        return self.left.active or self.right.active
```

Syntax:

```bash
python -m py_compile src/teleop_core/intervention.py
```

---

# 2. Modify `src/teleop_core/session.py`

Import all new intervention types used by this file.

## `TeleopSessionConfig`

Add:

```python
control_mode: ControlMode = ControlMode.CONTINUOUS
clutch: ClutchConfig = field(default_factory=ClutchConfig)
```

In `__post_init__`:
1. convert string `control_mode` to `ControlMode`
2. convert mapping `clutch` to `ClutchConfig`
3. force `clutch.enabled = control_mode != CONTINUOUS`
4. preserve all existing configuration normalization

Update `from_dict()` to convert nested `clutch` mapping before `cls(**values)`.

## Output state

Add to `HandSessionState`:

```python
clutch_active: bool = False
clutch_forced_hold: bool = False
```

Add to `TeleopSessionUpdate`:

```python
intervention: InterventionStatus = field(default_factory=InterventionStatus)
```

## `_HandRuntime`

Add:

```python
self.clutch_active = False
self.clutch_forced_hold = False
self.clutch_anchor: ClutchAnchor | None = None
```

Change `reset_filters()` to accept optional position/orientation and reset filters to current target when omitted.

In `reset_runtime_state()` clear clutch state and anchor.

## `BimanualTeleopSession.__init__`

Add:

```python
self._next_intervention_id = 0
self._active_intervention_id = -1
self._previous_any_human_active = False
```

## `update()`

Use signature:

```python
def update(
    self,
    controller_states: Mapping[str, ControllerState | None],
    *,
    clutch_input: BimanualClutchInput | None = None,
    measured_ee_poses: Mapping[str, EndEffectorPose] | None = None,
    now_s: float | None = None,
    dt_s: float | None = None,
) -> TeleopSessionUpdate:
```

Default missing clutch input and EE mapping to empty values.

Pass per-hand clutch input, measured EE pose, and `now_s` into `_ingest_controller_state()`.

After both hands:
- update intervention ID
- build `InterventionStatus`
- include it in `TeleopSessionUpdate`

## `_ingest_controller_state()`

Extend signature with:

```python
clutch_input: HandClutchInput
measured_ee_pose: EndEffectorPose | None
now_s: float
```

Keep existing:
- timestamp updates
- pose validation
- duplicate rejection
- jitter buffering
- calibration

After calibration:
- `CONTINUOUS` -> call `_update_continuous_target()`
- other modes -> call `_update_clutched_target()`

## `_update_continuous_target()`

Move current post-calibration mapping into this helper without changing math.

## `_apply_target()`

Move current shared target update block into a helper preserving:
- target `dt`
- stale recovery
- safety clamping
- velocity update
- target position/orientation update
- `last_target_update_s`

## `_update_clutched_target()`

Ordered logic:

1. If forced hold is active and button is now released, clear forced hold.
2. On rising edge, reject takeover if:
   - forced hold remains active
   - controller pose invalid
   - measured EE pose missing
   - measured EE pose invalid
3. Rejection:
   - clear active clutch and anchor
   - set forced hold
   - zero target velocity
   - emit warning
   - return
4. Successful rising edge:
   - convert controller quaternion with existing `FrameTransform`
   - create `ClutchAnchor`
   - set active clutch
   - clear forced hold
   - set raw and smoothed target to measured EE pose
   - zero target velocity
   - set `last_target_update_s`
   - prime safety with anchor position
   - reset filters to anchor pose
   - emit engagement event
   - return
5. Falling edge:
   - clear active clutch and anchor
   - keep current target unchanged
   - zero target velocity
   - reset filters to current target
   - emit release event
   - return
6. If clutch inactive, return without updating pose.
7. If clutch active:
   - require anchor; otherwise enter forced hold
   - calculate relative position/orientation using the formulas at top
   - call `_apply_target()`

## Intervention ID

Add `_update_intervention_id()`:
- inactive -> active: assign `_next_intervention_id`, then increment it
- active -> inactive: set active ID to `-1`
- one contiguous takeover keeps the same ID even if hand masks change

## Hard timeout

In existing hard-timeout branch, if clutch active:
- clear clutch and anchor
- set forced hold
- zero target velocity
- reset filters to current target
- do not change held target

Keep current gripper-open behavior.

## Snapshot/reset

`_snapshot()` must expose clutch fields.

`reset()`:
- set active intervention ID to `-1`
- clear previous active flag
- do not reset `_next_intervention_id`

Single-arm session must call `_ingest_controller_state()` with default empty clutch input and no measured EE pose.

Syntax:

```bash
python -m py_compile src/teleop_core/session.py
```

---

# 3. Modify `src/teleop_core/__init__.py`

Import and export:

```text
BimanualClutchInput
ClutchAnchor
ClutchConfig
ClutchInputMapper
ControlMode
ControlSource
EndEffectorPose
HandClutchInput
HandInterventionStatus
InterventionStatus
```

Syntax:

```bash
python -m py_compile src/teleop_core/__init__.py
```

---

# 4. Modify `src/robot_adapters/base.py`

Import `EndEffectorPose` with repository-compatible fallback:

```python
try:
    from src.teleop_core.intervention import EndEffectorPose
except ImportError:
    from teleop_core.intervention import EndEffectorPose
```

Add non-abstract method:

```python
def get_current_end_effector_poses(
    self,
    joint_positions=None,
) -> dict[str, EndEffectorPose]:
    return {}
```

Add:

```python
def validate_action(
    self,
    action: RobotAction | None,
) -> bool:
```

Return true only when:
- action exists
- flattened vector length equals `len(get_joint_names())`
- all values are finite

Syntax:

```bash
python -m py_compile src/robot_adapters/base.py
```

---

# 5. Modify `src/robot_adapters/bimanual_lula.py`

Import `EndEffectorPose`.

Add:

```python
def get_current_end_effector_poses(
    self,
    joint_positions=None,
) -> dict[str, EndEffectorPose]:
```

Implementation:
- use supplied joints or `get_current_joint_positions()`
- return `{}` if unavailable
- for each available solver call existing `_home_pose_from_fk()`
- pass corresponding arm joints
- return `{"left": EndEffectorPose(...), "right": EndEffectorPose(...)}`
- omit failed arms instead of raising
- keep positions in the teleop target frame through existing FK helper

Add:

```python
def sync_ik_warm_start(
    self,
    joint_positions=None,
) -> None:
```

Set each arm runtime’s `last_arm_positions` from current articulation joints.

Do not call either method from `compute_action()`.

Syntax:

```bash
python -m py_compile src/robot_adapters/bimanual_lula.py
```

---

# 6. Create `src/launch/action_arbitration.py`

Imports:

```python
from __future__ import annotations
from dataclasses import dataclass
from typing import Sequence
import numpy as np
from src.robot_adapters import RobotAction
from src.teleop_core import ControlMode, ControlSource, InterventionStatus
```

Add:

```python
@dataclass(frozen=True)
class ActionArbiterConfig:
    mode: ControlMode
    arbitration: str = "per_hand"
    policy_reentry_blend_s: float = 0.2


@dataclass(frozen=True)
class ArbitrationResult:
    executed_action: RobotAction
    human_action: RobotAction | None
    policy_action: RobotAction | None
    control_source: ControlSource
    left_human: bool
    right_human: bool
    intervention_id: int
    human_action_valid: bool
    policy_action_valid: bool
    reentry_blend_active: bool


@dataclass
class _ReleaseBlend:
    start_s: float
    start_values: np.ndarray
```

Add `BimanualActionArbiter` with constructor:

```python
def __init__(
    self,
    *,
    config: ActionArbiterConfig,
    left_indices: Sequence[int],
    right_indices: Sequence[int],
):
```

Store index arrays and state:
- last executed vector
- previous left/right human flags
- left/right release blends

Add `reset()`.

Add `_valid(action, expected_size)` requiring exact finite shape.

Add:

```python
def select(
    self,
    *,
    current_positions: np.ndarray,
    human_action: RobotAction | None,
    policy_action: RobotAction | None,
    intervention: InterventionStatus,
    now_s: float,
) -> ArbitrationResult:
```

Logic:

1. Flatten/copy current positions.
2. Validate/copy candidate actions.
3. `hold = last_executed` or current positions.
4. `CONTINUOUS`: execute valid human action, otherwise hold.
5. Human masks:
   - `global`: both masks equal `intervention.active`
   - `per_hand`: use left/right status
6. Base action:
   - `CLUTCHED`: hold
   - `POLICY_INTERVENTION`: valid policy or hold
7. Forced-hold side always uses hold values.
8. Active human side overwrites matching indices from human action.
9. Detect human true -> false transition per side.
10. On release, store previous executed values and release time.
11. In policy mode, blend released indices toward current policy:
    ```python
    alpha = clip((now_s - start_s) / blend_s, 0, 1)
    ```
12. Do not blend forced-hold sides.
13. Source:
    - both human -> `HUMAN`
    - one human -> `MIXED`
    - active release blend -> `MIXED`
    - valid policy with no forced hold -> `POLICY`
    - otherwise -> `HOLD`
14. Copy output; never mutate input actions.
15. Update internal state.
16. Return all candidate validity/mask/ID fields.

Syntax:

```bash
python -m py_compile src/launch/action_arbitration.py
```

---

# 7. Create `src/launch/policy_provider.py`

Add only:

```python
@dataclass(frozen=True)
class PolicyObservation:
    joint_positions: np.ndarray
    monotonic_time_s: float


class PolicyActionProvider(Protocol):
    def reset(self) -> None: ...
    def get_action(
        self,
        observation: PolicyObservation,
    ) -> RobotAction | None: ...


class NullPolicyActionProvider:
    def reset(self) -> None:
        return None

    def get_action(
        self,
        observation: PolicyObservation,
    ) -> RobotAction | None:
        return None
```

Required imports:
- `dataclass`
- `Protocol`
- `numpy`
- `RobotAction`

Do not add model/network/camera code.

Syntax:

```bash
python -m py_compile src/launch/policy_provider.py
```

---

# 8. Modify `src/launch/teleop_utils.py`

Import `ClutchConfig` and `ControlMode`.

Add CLI options:

```text
--control-mode
--clutch-button
--clutch-arbitration
--policy-reentry-blend-ms
```

Allowed values:
- control: `continuous`, `clutched`, `policy_intervention`
- button: `primary`, `secondary`, `menu`, `stick_click`
- arbitration: `global`, `per_hand`

Add:

```python
def apply_teleop_cli_overrides(
    settings: dict,
    args: argparse.Namespace,
) -> dict:
```

Return a copied mapping with overrides applied to:
- `control_mode`
- `intervention.button`
- `intervention.arbitration`
- `intervention.policy_reentry_blend_ms`

In `build_common_teleop_config()`:
- parse `ControlMode`
- read `settings["intervention"]`
- construct `ClutchConfig`
- convert blend ms to seconds
- pass `control_mode` and `clutch` into `TeleopSessionConfig`
- preserve all existing fields

Syntax:

```bash
python -m py_compile src/launch/teleop_utils.py
```

---

# 9. Modify `src/config_loader.py`

Add under default teleop config:

```python
"control_mode": "continuous",
"intervention": {
    "button": "stick_click",
    "arbitration": "per_hand",
    "policy_reentry_blend_ms": 200,
},
```

In `_normalize_main_config()`:
- ensure `teleop.intervention` is a dict
- merge it with default intervention mapping
- preserve current smoothing normalization

Syntax:

```bash
python -m py_compile src/config_loader.py
```

---

# 10. Modify `config/config.yaml`

Merge without deleting existing keys:

```yaml
teleop:
  control_mode: continuous
  intervention:
    button: stick_click
    arbitration: per_hand
    policy_reentry_blend_ms: 200

recording:
  intervention:
    enabled: false
    include_candidate_actions: true
```

Syntax:

```bash
python - <<'PY'
import yaml
data = yaml.safe_load(open("config/config.yaml", encoding="utf-8"))
assert isinstance(data, dict)
assert data["teleop"]["control_mode"] in {
    "continuous", "clutched", "policy_intervention"
}
assert data["teleop"]["intervention"]["button"] in {
    "primary", "secondary", "menu", "stick_click"
}
assert data["teleop"]["intervention"]["arbitration"] in {
    "global", "per_hand"
}
assert float(
    data["teleop"]["intervention"]["policy_reentry_blend_ms"]
) >= 0
assert isinstance(
    data["recording"]["intervention"]["enabled"],
    bool,
)
print("config/config.yaml: OK")
PY
```

---

# 11. Modify Bimanual Launchers

Files:

```text
src/launch/openarm_teleop.py
src/launch/acone_teleop.py
src/launch/ffw_bg2_teleop.py
```

In each:
- import `apply_teleop_cli_overrides`
- after main/robot teleop merge, apply CLI overrides
- then call `build_common_teleop_config()`
- do not create a policy model/provider

Syntax:

```bash
python -m py_compile src/launch/openarm_teleop.py
python -m py_compile src/launch/acone_teleop.py
python -m py_compile src/launch/ffw_bg2_teleop.py
```

---

# 12. Modify `src/launch/bimanual_runtime.py`

Import:
- `ActionArbiterConfig`
- `ArbitrationResult`
- `BimanualActionArbiter`
- `NullPolicyActionProvider`
- `PolicyActionProvider`
- `PolicyObservation`
- `ClutchInputMapper`
- `ControlMode`

Extend `run_bimanual_runtime()` with optional:

```python
policy_provider: PolicyActionProvider | None = None
```

After session creation:
- create `ClutchInputMapper(runtime_config.clutch)`

After joint mappings:
- build left indices = arm + gripper
- build right indices = arm + gripper
- create `BimanualActionArbiter`
- replace missing provider with `NullPolicyActionProvider`

Pass mapper, arbiter, provider into `_run_control_loop()`.

## Control-loop order

Use this order:

1. spin ROS
2. get latest controller states
3. update clutch mapper
4. calculate `now_s` and `loop_dt_s`
5. on any rising edge:
   - read current joints
   - call measured EE FK
   - sync IK warm start
6. call `teleop_session.update(...)`
7. read current joints if not already read
8. compute human action
9. request policy action only in policy mode
10. arbitrate
11. apply executed action
12. publish executed action
13. record executed/candidate data
14. step simulation

Session call:

```python
session_update = teleop_session.update(
    latest_states,
    clutch_input=clutch_input,
    measured_ee_poses=measured_ee_poses,
    now_s=now_s,
    dt_s=loop_dt_s,
)
```

Human candidate:

```python
human_action = adapter.compute_action(session_update.targets)
```

Policy candidate:
- only call provider in `POLICY_INTERVENTION`
- pass copied current joint positions and `now_s`
- catch provider exceptions and use `None`
- log using existing rate-limited style

Arbitration:

```python
arbitration = action_arbiter.select(
    current_positions=current_positions,
    human_action=human_action,
    policy_action=policy_action,
    intervention=session_update.intervention,
    now_s=now_s,
)
```

Apply/publish:

```python
adapter.apply_action(arbitration.executed_action)
joint_state_publisher.publish(
    dof_names,
    arbitration.executed_action.joint_positions,
    stamp=stamp,
)
```

Keep `teleop_session.mark_isaac_apply()` after apply.

IK debug uses human candidate positions.

Recording call must receive `arbitration` and `session_update`.

On scene reset:
- `clutch_mapper.reset()`
- `action_arbiter.reset()`
- `policy_provider.reset()`
- preserve current teleop session calibration reset behavior

Ready output:
- print control mode
- for non-continuous mode print clutch button, arbitration, and blend ms
- if policy mode uses null provider, print once that released arms will hold

Change `_maybe_record_frame()` signature to accept:
- `arbitration: ArbitrationResult`
- `session_update`

Syntax:

```bash
python -m py_compile src/launch/bimanual_runtime.py
```

---

# 13. Modify `src/recording/config.py`

Add:

```python
@dataclass(frozen=True)
class RecordingInterventionConfig:
    enabled: bool = False
    include_candidate_actions: bool = True
```

Add `from_mapping()`.

Add to `RecordingConfig`:

```python
intervention: RecordingInterventionConfig = field(
    default_factory=RecordingInterventionConfig
)
```

Parse `values.get("intervention")` in `RecordingConfig.from_mapping()`.

Do not auto-enable based on teleop mode.

Syntax:

```bash
python -m py_compile src/recording/config.py
```

---

# 14. Modify `src/recording/schema.py`

Extend:

```python
@dataclass(frozen=True)
class RecordingSchema:
    ...
    auxiliary_specs: tuple[VectorSpec, ...] = ()
```

In `build_recording_schema()`:
- create `auxiliary_specs = []`
- when intervention recording disabled, leave current schema unchanged
- when enabled, add scalar `int64`, shape `(1,)` features:
  ```text
  action.human_valid
  action.policy_valid
  control.source
  intervention.active
  intervention.left
  intervention.right
  intervention.id
  intervention.reentry_blend_active
  ```
- when candidate actions enabled, add:
  ```text
  action.human
  action.policy
  ```
  using the normal action shape, names, and dtype
- return `auxiliary_specs=tuple(auxiliary_specs)`

Syntax:

```bash
python -m py_compile src/recording/schema.py
```

---

# 15. Modify `src/recording/snapshots.py`

Add to `RecordingFrameSnapshot` after `cameras`:

```python
extra_features: dict[str, np.ndarray] = field(default_factory=dict)
```

Do not change diagnostics.

Syntax:

```bash
python -m py_compile src/recording/snapshots.py
```

---

# 16. Modify `src/recording/worker_process.py`

Add NumPy import.

In `_frame_to_payload()`:
- iterate `self._schema.auxiliary_specs`
- require each key in `snapshot.extra_features`
- cast with `np.asarray(..., dtype=spec.dtype)`
- reshape to `spec.shape`
- add to payload
- do not serialize undeclared extra keys

Syntax:

```bash
python -m py_compile src/recording/worker_process.py
```

---

# 17. Modify `src/recording/__init__.py`

Import/export:

```text
RecordingInterventionConfig
```

Syntax:

```bash
python -m py_compile src/recording/__init__.py
```

---

# 18. Complete Recording Logic in `src/launch/bimanual_runtime.py`

Inside `_maybe_record_frame()`:

1. Build state vector using `arbitration.executed_action`.
2. Build normal action vector using `arbitration.executed_action`.
3. Initialize `extra_features = {}`.
4. If intervention recording enabled, add `(1,) int64` arrays:
   - human valid
   - policy valid
   - control source enum value
   - intervention active
   - left human mask
   - right human mask
   - intervention ID
   - re-entry blend active
5. If candidate actions enabled:
   - convert valid candidate through `adapter.get_recording_vector()`
   - otherwise use zero vector matching action shape
   - store `action.human` and `action.policy`
6. Copy state/action/candidate arrays before enqueue.
7. `RecordingFrameSnapshot.action` must be executed action vector.
8. Pass `extra_features` into snapshot.
9. Never store NaN candidate values.

Syntax:

```bash
python -m py_compile src/launch/bimanual_runtime.py
```

---

# Do Not Modify

```text
src/teleop_core/controller_state.py
src/launch/controller_provider.py
src/recording/buttons.py
src/recording/lerobot_recorder.py
src/isaac_openarm_teleop_openpi.py
```

`stick_click` and Joy index 3 already exist. Recorder IPC already transfers the snapshot object.

---

# Final Syntax Validation

```bash
python -m py_compile \
  src/teleop_core/intervention.py \
  src/teleop_core/session.py \
  src/teleop_core/__init__.py \
  src/robot_adapters/base.py \
  src/robot_adapters/bimanual_lula.py \
  src/launch/action_arbitration.py \
  src/launch/policy_provider.py \
  src/launch/teleop_utils.py \
  src/launch/openarm_teleop.py \
  src/launch/acone_teleop.py \
  src/launch/ffw_bg2_teleop.py \
  src/launch/bimanual_runtime.py \
  src/config_loader.py \
  src/recording/config.py \
  src/recording/schema.py \
  src/recording/snapshots.py \
  src/recording/worker_process.py \
  src/recording/__init__.py

python -m compileall -q src
```

Parse YAML with the command in section 10.

Fix every syntax failure before completion.

# Completion Output

```text
Changed files:
- ...

Implemented:
- continuous mode preserved
- clutched relative takeover
- global/per-hand arbitration
- policy re-entry blending
- intervention recording metadata

Syntax:
- py_compile: pass/fail
- compileall: pass/fail
- YAML parse: pass/fail

Not implemented:
- model-specific policy provider
```
