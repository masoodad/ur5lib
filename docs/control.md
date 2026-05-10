# Control Module

**Package:** `ur5lib.control`

The control module closes the loop between trajectory planning and robot
execution.  Where `MotionExecutor` sends a pre-computed path open-loop,
`ControlExecutor` reads the actual joint state at every servo tick, corrects
errors with a PID controller, and streams commands via `servoJ` — making
trajectory tracking robust to load, friction, and communication jitter.

---

## Architecture

```
Trajectory Generator  →  list[joints]
                               │
                    ┌──────────▼──────────┐
                    │  SafetyChecker       │  validate before execution
                    └──────────┬──────────┘
                               │
              ┌────────────────▼────────────────┐
              │         ControlExecutor          │
              │  ┌─────────────────────────┐     │
              │  │  q_ref (trajectory)     │     │
              │  │        ↓               │     │
              │  │  q_actual ← robot      │     │
              │  │  error = q_ref−q_actual │     │
              │  │        ↓               │     │
              │  │     JointPID           │     │
              │  │        ↓               │     │
              │  │  q_cmd → robot.servoJ  │     │
              │  └─────────────────────────┘     │
              └──────────────────────────────────┘
```

---

## Modules

| Module | Classes / Functions |
|---|---|
| `control.kinematics` | `tcp_pose`, `geometric_jacobian`, `ik_numerical`, `forward_kinematics_transforms` |
| `control.profiles` | `TrapezoidalProfile`, `SCurveProfile` |
| `control.safety` | `SafetyChecker`, `UR5_JOINT_LIMITS`, `UR5_MAX_JOINT_VEL` |
| `control.joint_pid` | `PIDController`, `JointPID` |
| `control.cartesian` | `CartesianController` |
| `control.executor` | `ControlExecutor` |

---

## `ControlExecutor`

**Module:** `ur5lib.control.executor`

Top-level closed-loop executor.  Accepts any trajectory list and drives the
robot via `servoJ` with per-joint PID correction.

```python
class ControlExecutor:
    def __init__(self, robot, pid=None, safety=None)
```

| Parameter | Type | Default | Description |
|---|---|---|---|
| `robot` | `UR5Base` | required | Connected robot instance |
| `pid` | `JointPID` | `JointPID()` | Per-joint PID controller |
| `safety` | `SafetyChecker` | `None` | Pre-execution safety validation |

### Methods

#### `execute(trajectory, dt=0.008)`

Run a trajectory at a fixed servo rate.

```python
executor.execute(trajectory, dt=0.008)
```

| Parameter | Type | Description |
|---|---|---|
| `trajectory` | `list[list[float]]` | Joint configurations from any `generate()` call |
| `dt` | `float` | Servo period in seconds (default 8 ms = 125 Hz) |

**Example:**
```python
from ur5lib import UR5Sim, JointAngles
from ur5lib.control import ControlExecutor, SafetyChecker
from ur5lib.trajectories.spline import SplineTrajectory

robot = UR5Sim(); robot.connect_()

traj = SplineTrajectory([
    JointAngles([0.0, -1.57, 0.0, -1.57, 0.0, 0.0]),
    JointAngles([0.5, -1.2,  0.6, -1.2,  0.2, 0.0]),
    JointAngles([0.0, -1.57, 0.0, -1.57, 0.0, 0.0]),
]).generate()

executor = ControlExecutor(robot, safety=SafetyChecker())
executor.execute(traj, dt=0.008)
```

#### `execute_timed(timed_trajectory)`

Execute a time-stamped trajectory from a profile.  Sleeps between steps to
honour the requested timestamps.

```python
from ur5lib.control import TrapezoidalProfile

timed = TrapezoidalProfile(v_max=1.0, a_max=2.0).parameterize(traj)
executor.execute_timed(timed)
```

---

## `JointPID` / `PIDController`

**Module:** `ur5lib.control.joint_pid`

Discrete-time PID with integral anti-windup.

```python
class PIDController:
    def __init__(self, kp, ki, kd, dt, output_limits=(-π, π))
    def reset()
    def step(error: float) -> float

class JointPID:
    def __init__(self, kp=2.0, ki=0.05, kd=0.1, dt=0.008, n_joints=6)
    def reset()
    def step(q_ref, q_actual) -> list[float]
```

Gains may be a scalar (applied to all joints) or a per-joint list.

**Example:**
```python
from ur5lib.control import JointPID

pid = JointPID(kp=3.0, ki=0.1, kd=0.2, dt=0.008)
q_cmd = pid.step(
    q_ref    = [0.5, -1.0, 0.3, -1.5, 0.0, 0.0],
    q_actual = [0.48, -0.99, 0.31, -1.49, 0.01, 0.0],
)
```

---

## `SafetyChecker`

**Module:** `ur5lib.control.safety`

Validates joint configurations and trajectories against UR5 hardware limits.

```python
class SafetyChecker:
    def __init__(self, joint_limits=None, v_max=None, a_max=None)
```

| Default | Value |
|---|---|
| Position limits | ±360° (±2π rad) on all 6 joints |
| Velocity limit | 180°/s (π rad/s) |
| Acceleration limit | 400°/s² (≈ 6.98 rad/s²) |

### Methods

| Method | Raises | Returns |
|---|---|---|
| `check_configuration(joints)` | — | `list[str]` violation messages |
| `validate_configuration(joints)` | `JointLimitError` | — |
| `check_trajectory(trajectory, dt)` | — | `list[str]` violation messages |
| `validate_trajectory(trajectory, dt)` | `SafetyViolationError` | — |

**Example:**
```python
from ur5lib.control import SafetyChecker
from ur5lib.exceptions import JointLimitError

safety = SafetyChecker()

violations = safety.check_configuration([0.1, -0.5, 0.3, -1.2, 1.5, 0.0])
# [] — no violations

try:
    safety.validate_configuration([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
except JointLimitError as e:
    print(e)  # Joint 1: 572.96° outside [-360°, 360°]
```

---

## `TrapezoidalProfile` / `SCurveProfile`

**Module:** `ur5lib.control.profiles`

Assign timestamps to a position-only trajectory so that velocity and
acceleration limits are respected during servo playback.

```python
class TrapezoidalProfile:
    def __init__(self, v_max=1.0, a_max=2.0)
    def parameterize(trajectory) -> list[(float, list[float])]

class SCurveProfile:
    def __init__(self, v_max=1.0)
    def parameterize(trajectory) -> list[(float, list[float])]
```

| Profile | Velocity shape | Acceleration | Best for |
|---|---|---|---|
| `TrapezoidalProfile` | Ramp up → cruise → ramp down | Stepped | General motion |
| `SCurveProfile` | Quintic smoothstep | Continuous | Precision tasks, low vibration |

**Example:**
```python
from ur5lib.control import TrapezoidalProfile, SCurveProfile

traj = my_trajectory_generator.generate()

timed_trap  = TrapezoidalProfile(v_max=1.0, a_max=2.0).parameterize(traj)
timed_scurve = SCurveProfile(v_max=1.0).parameterize(traj)

# Each entry: (timestamp_seconds, joint_config)
t0, q0 = timed_trap[0]    # (0.0, [...])
tN, qN = timed_trap[-1]   # (total_time, [...])
```

---

## `CartesianController`

**Module:** `ur5lib.control.cartesian`

Jacobian-based Cartesian velocity controller.  Maps a target TCP pose to
joint position increments without requiring an explicit IK solution.

```python
class CartesianController:
    def __init__(self, robot, kp=1.0, lam=0.05, dh_params=None)
    def step(target_pose, dt=0.008) -> list[float]
    def move_to_pose(target_pose, tol=1e-3, max_steps=500, dt=0.008) -> list[float]
```

| Parameter | Description |
|---|---|
| `kp` | Cartesian proportional gain |
| `lam` | DLS damping factor — increase near singularities |
| `dh_params` | Robot DH table; `None` uses UR5 defaults from `robots/` |

**Example:**
```python
from ur5lib import UR5Sim, Pose
from ur5lib.control import CartesianController

robot = UR5Sim(); robot.connect_()
ctrl  = CartesianController(robot, kp=1.5)

target = Pose(0.4, 0.0, 0.5, 0.0, 3.14159, 0.0)
final_joints = ctrl.move_to_pose(target, tol=1e-3)
```

---

## Kinematics

**Module:** `ur5lib.control.kinematics`

Analytical forward kinematics, geometric Jacobian, and numerical IK.
All functions accept an optional `dh_params` argument and work for any
serial manipulator defined by a DH table.

### `tcp_pose(joints, dh_params=None) → Pose`

Forward kinematics — returns TCP pose (position + axis-angle orientation).

```python
from ur5lib.control import tcp_pose

pose = tcp_pose([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])
# Pose(x=..., y=..., z=..., rx=..., ry=..., rz=...)
```

### `geometric_jacobian(joints, dh_params=None) → ndarray`

Returns the (6 × n) geometric Jacobian at the given configuration.

```python
from ur5lib.control import geometric_jacobian
import numpy as np

J = geometric_jacobian([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])
# shape (6, 6) for UR5
```

### `ik_numerical(target_pose, q_init, ...) → list[float]`

Iterative IK via damped least-squares Jacobian.

```python
from ur5lib.control import ik_numerical, tcp_pose
from ur5lib.types.common_types import Pose

target = Pose(0.4, 0.1, 0.5, 0.0, 3.14, 0.0)
q0     = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

q_sol = ik_numerical(target, q0, tol=1e-5)
print(tcp_pose(q_sol))   # should match target within tol
```

| Parameter | Default | Description |
|---|---|---|
| `tol` | `1e-4` | Convergence threshold on ‖Δx‖ |
| `max_iter` | `200` | Maximum iterations |
| `lam` | `0.05` | Damping for DLS pseudoinverse |

Raises `KinematicsError` if the solver does not converge.

---

## Complete Closed-Loop Example

```python
from ur5lib import UR5Sim, JointAngles
from ur5lib.control import (ControlExecutor, JointPID,
                             SafetyChecker, TrapezoidalProfile)
from ur5lib.trajectories.spline import SplineTrajectory

# 1. Connect
robot = UR5Sim()
robot.connect_()

# 2. Build trajectory
waypoints = [
    JointAngles([0.0, -1.57, 0.0, -1.57, 0.0, 0.0]),
    JointAngles([0.5, -1.2,  0.8, -1.4,  0.3, 0.0]),
    JointAngles([0.8, -0.9,  1.0, -1.8,  0.5, 0.0]),
    JointAngles([0.0, -1.57, 0.0, -1.57, 0.0, 0.0]),
]
traj = SplineTrajectory(waypoints).generate()

# 3. Time-parameterise (respect velocity limits)
timed = TrapezoidalProfile(v_max=0.8, a_max=1.5).parameterize(traj)

# 4. Execute with PID + safety
executor = ControlExecutor(
    robot,
    pid    = JointPID(kp=2.5, ki=0.05, kd=0.15, dt=0.008),
    safety = SafetyChecker(),
)
executor.execute_timed(timed)
```
