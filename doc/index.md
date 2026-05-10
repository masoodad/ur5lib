# ur5lib Documentation

**Version:** 0.1.0  
**Author:** Masood Ahmad  
**Python:** >= 3.7

ur5lib is a Python library for controlling UR5 robots. It provides a unified interface for both simulation and real hardware control via RTDE (Real-Time Data Exchange).

---

## Contents

- [Types](types.md) — `JointAngles`, `Pose`
- [Core](core.md) — `UR5Base` abstract interface
- [IO / Backends](io.md) — `UR5Sim`, `UR5RTDE`
- [Motion](motion.md) — `MotionPlanner`, `MotionExecutor`
- [Exceptions](exceptions.md) — `UR5Error`, `NotConnectedError`, `InvalidConfigurationError`
- [CLI](cli.md) — `ur5lib-cli` command
- [Examples](examples.md) — Usage examples

---

## Quick Start

```python
from ur5lib import UR5Sim, MotionExecutor, JointAngles

robot = UR5Sim()
robot.connect_()

executor = MotionExecutor(robot)
executor.move_to_joint_position(JointAngles([0.1, -0.5, 0.3, -1.2, 1.5, 0.0]))
```

For a real robot:

```python
from ur5lib import UR5RTDE, MotionExecutor, JointAngles

robot = UR5RTDE(config={"robot_ip": "192.168.0.100"})
robot.connect_()

executor = MotionExecutor(robot)
executor.move_to_joint_position(JointAngles([0.1, -0.5, 0.3, -1.2, 1.5, 0.0]))
```

---

## Architecture

```
ur5lib
├── types/          JointAngles, Pose
├── core.py         UR5Base (abstract interface)
├── io/
│   ├── simulator   UR5Sim  (no hardware required)
│   └── ur_rtde     UR5RTDE (real robot via RTDE)
├── motion/
│   ├── planner     MotionPlanner (path interpolation)
│   └── executor    MotionExecutor (plan + execute)
├── exceptions.py   Library exceptions
└── cli.py          Command-line interface
```

---

## Exported Symbols

All public classes are importable directly from `ur5lib`:

```python
from ur5lib import (
    UR5Base,
    UR5Sim,
    UR5RTDE,
    MotionPlanner,
    MotionExecutor,
    JointAngles,
    Pose,
)
```
