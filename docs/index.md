# ur5lib Documentation

**Version:** 1.1.0
**Author:** Masood Ahmad
**Python:** >= 3.7

ur5lib is a Python library for controlling UR5 robots. It provides a unified
interface for both simulation and real hardware control via RTDE, a full suite
of trajectory generators, and a closed-loop control layer for accurate,
safe execution.

---

## Contents

- [Types](types.md) — `JointAngles`, `Pose`
- [Core](core.md) — `UR5Base` abstract interface
- [IO / Backends](io.md) — `UR5Sim`, `UR5RTDE`
- [Motion](motion.md) — `MotionPlanner`, `MotionExecutor`
- [Control](control.md) — `ControlExecutor`, `JointPID`, `SafetyChecker`, `TrapezoidalProfile`, `SCurveProfile`, `CartesianController`, kinematics
- [Exceptions](exceptions.md) — all library exceptions
- [CLI](cli.md) — `ur5lib-cli` command
- [Examples](examples.md) — usage examples

---

## Quick Start

**Open-loop (simulation):**
```python
from ur5lib import UR5Sim, MotionExecutor, JointAngles

robot = UR5Sim()
robot.connect_()

executor = MotionExecutor(robot)
executor.move_to_joint_position(JointAngles([0.1, -0.5, 0.3, -1.2, 1.5, 0.0]))
```

**Closed-loop with safety (real or sim):**
```python
from ur5lib import UR5Sim, JointAngles
from ur5lib.control import ControlExecutor, SafetyChecker, TrapezoidalProfile
from ur5lib.trajectories.spline import SplineTrajectory

robot = UR5Sim()
robot.connect_()

traj  = SplineTrajectory([
    JointAngles([0.0, -1.57, 0.0, -1.57, 0.0, 0.0]),
    JointAngles([0.5, -1.2,  0.8, -1.4,  0.3, 0.0]),
    JointAngles([0.0, -1.57, 0.0, -1.57, 0.0, 0.0]),
]).generate()

timed    = TrapezoidalProfile(v_max=1.0, a_max=2.0).parameterize(traj)
executor = ControlExecutor(robot, safety=SafetyChecker())
executor.execute_timed(timed)
```

**Forward / inverse kinematics:**
```python
from ur5lib.control import tcp_pose, ik_numerical

# FK
pose = tcp_pose([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])

# IK
q = ik_numerical(pose, q_init=[0.1, -1.5, 0.1, -1.5, 0.1, 0.0])
```

---

## Architecture

```
ur5lib
├── types/            JointAngles, Pose
├── core.py           UR5Base (abstract interface)
├── robots/
│   └── ur5_dh        UR5_DH  (DH parameters — single source of truth)
├── io/
│   ├── simulator     UR5Sim  (no hardware required)
│   └── ur_rtde       UR5RTDE (real robot via RTDE)
├── motion/
│   ├── planner       MotionPlanner  (linear interpolation)
│   └── executor      MotionExecutor (open-loop plan + execute)
├── trajectories/
│   ├── joint_space   JointSpaceTrajectory   (LERP)
│   ├── cartesian_space CartesianLinearTrajectory
│   └── spline        SplineTrajectory (Catmull-Rom)
├── control/                ★ NEW
│   ├── kinematics    FK, Jacobian, numerical IK
│   ├── profiles      TrapezoidalProfile, SCurveProfile
│   ├── safety        SafetyChecker (position / velocity / accel limits)
│   ├── joint_pid     PIDController, JointPID
│   ├── cartesian     CartesianController (Jacobian velocity control)
│   └── executor      ControlExecutor (closed-loop servoJ loop)
├── simulation/       TrajectorySimulator
├── animations/       animate_trajectories, forward_kinematics
├── exceptions.py     UR5Error hierarchy
└── cli.py            ur5lib-cli
```

---

## Exported Symbols

All public classes and functions are importable directly from `ur5lib`:

```python
from ur5lib import (
    # Robot backends
    UR5Base, UR5Sim, UR5RTDE,

    # Open-loop motion
    MotionPlanner, MotionExecutor,

    # Types
    JointAngles, Pose,

    # Control  ★ NEW
    ControlExecutor, CartesianController,
    JointPID, PIDController,
    SafetyChecker,
    TrapezoidalProfile, SCurveProfile,
    tcp_pose, geometric_jacobian,
    ik_numerical, forward_kinematics_transforms,

    # Exceptions
    UR5Error, NotConnectedError, InvalidConfigurationError,
    JointLimitError, SafetyViolationError,
    KinematicsError, ControlError,
)
```
