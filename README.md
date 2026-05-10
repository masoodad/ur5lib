# ur5lib

Modular Python library for controlling UR5 robots — supports both a real robot (via RTDE) and a built-in simulator.

## Requirements

- Python >= 3.7
- `numpy`
- `ur_rtde` (only required when connecting to a real robot)

## Installation

Clone the repo and install in editable mode:

```bash
git clone <repo-url>
cd ur5lib
pip install -e .
```

Or install directly:

```bash
pip install .
```

For real robot support, also install the RTDE package:

```bash
pip install ur_rtde
```

## Quick Start

### Simulator mode

```python
from ur5lib.io.simulator import UR5Sim
from ur5lib.motion.executor import MotionExecutor
from ur5lib.types.common_types import JointAngles, Pose

# Create and connect a simulated robot
robot = UR5Sim()
robot.connect_()

executor = MotionExecutor(robot)

# Move to a joint configuration (angles in radians)
target = JointAngles(joints=[0.1, -0.5, 0.3, -1.2, 1.5, 0.0])
executor.move_to_joint_position(target)

# Move to a Cartesian pose (x, y, z, rx, ry, rz)
pose = Pose(0.4, -0.2, 0.3, 0.0, 3.14, 0.0)
executor.move_to_pose(pose)
```

### Real robot mode (RTDE)

```python
from ur5lib.io.ur_rtde import UR5RTDE
from ur5lib.motion.executor import MotionExecutor
from ur5lib.types.common_types import JointAngles

robot = UR5RTDE(config={"robot_ip": "192.168.0.100"})
robot.connect_()

executor = MotionExecutor(robot)
target = JointAngles(joints=[0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
executor.move_to_joint_position(target)
```

## CLI

After installation, a `ur5lib-cli` command is available:

```bash
# Simulator — move joints to specified angles (radians)
ur5lib-cli --sim --move-joints 0.1 -0.5 0.3 -1.2 1.5 0.0

# Real robot — specify IP and target joint angles
ur5lib-cli --ip 192.168.0.100 --move-joints 0.0 -1.57 1.57 -1.57 -1.57 0.0
```

## Key Types

| Type | Fields | Description |
|------|--------|-------------|
| `JointAngles` | `joints: List[float]` | Six joint angles in radians |
| `Pose` | `x, y, z, rx, ry, rz` | TCP pose in meters / axis-angle rotation |

## Project Structure

```
ur5lib/
├── core.py              # Abstract base class UR5Base
├── cli.py               # Command-line interface
├── io/
│   ├── simulator.py     # UR5Sim — software-only simulator
│   └── ur_rtde.py       # UR5RTDE — real robot via RTDE
├── motion/
│   ├── planner.py       # MotionPlanner — joint & Cartesian interpolation
│   └── executor.py      # MotionExecutor — plan + execute moves
├── types/
│   └── common_types.py  # JointAngles, Pose
├── examples/
│   └── demo_joint_motion.py
└── config/
    └── default_config.yaml
```

## Running the Example

```bash
python examples/demo_joint_motion.py
```

## Extending the Library

To add a new robot backend, subclass `UR5Base` and implement the three abstract methods:

```python
from ur5lib.core import UR5Base

class MyRobot(UR5Base):
    def connect_rtde(self): ...
    def get_joint_angles(self): ...
    def get_current_pose(self): ...
    def run_motion(self, motion_plan): ...
```
