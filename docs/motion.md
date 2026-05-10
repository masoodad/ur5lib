# Motion — Planner & Executor

---

## MotionPlanner

**Module:** `ur5lib.motion.planner`

Generates interpolated waypoint paths between a start and goal configuration. Uses linear (straight-line) interpolation in either joint or Cartesian space.

### Class: `MotionPlanner`

```python
class MotionPlanner:
    def __init__(self, num_points: int = 10)
```

#### Constructor Parameters

| Parameter    | Type  | Default | Description                              |
|--------------|-------|---------|------------------------------------------|
| `num_points` | `int` | `10`    | Number of waypoints to generate along the path (including start and end) |

---

### Methods

#### `plan_joint_motion(start, goal) -> List[List[float]]`

Generates a linearly interpolated path in **joint space**.

```python
def plan_joint_motion(self, start: JointAngles, goal: JointAngles) -> List[List[float]]
```

| Parameter | Type          | Description                  |
|-----------|---------------|------------------------------|
| `start`   | `JointAngles` | Starting joint configuration |
| `goal`    | `JointAngles` | Target joint configuration   |

**Returns:** `List[List[float]]` — list of `num_points` waypoints, each a list of 6 joint angles in radians.

**Example:**
```python
from ur5lib import MotionPlanner, JointAngles

planner = MotionPlanner(num_points=20)

start = JointAngles([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
goal  = JointAngles([0.1, -0.5, 0.3, -1.2, 1.5, 0.0])

path = planner.plan_joint_motion(start, goal)
# path is a list of 20 joint configurations
```

---

#### `plan_cartesian_motion(start, goal) -> List[Pose]`

Generates a linearly interpolated path in **Cartesian space**.

```python
def plan_cartesian_motion(self, start: Pose, goal: Pose) -> List[Pose]
```

| Parameter | Type   | Description       |
|-----------|--------|-------------------|
| `start`   | `Pose` | Starting TCP pose |
| `goal`    | `Pose` | Target TCP pose   |

**Returns:** `List[Pose]` — list of `num_points` intermediate poses.

**Example:**
```python
from ur5lib import MotionPlanner, Pose

planner = MotionPlanner(num_points=15)

start = Pose(0.1, 0.2, 0.3, 0.0, 0.0, 0.0)
goal  = Pose(0.5, 0.1, 0.4, 0.0, 3.14, 0.0)

path = planner.plan_cartesian_motion(start, goal)
# path is a list of 15 Pose objects
```

---

---

## MotionExecutor

**Module:** `ur5lib.motion.executor`

High-level interface that combines planning and execution. Given a target configuration, it reads the robot's current state, computes an interpolated path, and commands the robot to follow it.

### Class: `MotionExecutor`

```python
class MotionExecutor:
    def __init__(self, robot: UR5Base, planner: MotionPlanner = None)
```

#### Constructor Parameters

| Parameter | Type             | Default               | Description                        |
|-----------|------------------|-----------------------|------------------------------------|
| `robot`   | `UR5Base`        | (required)            | Connected robot instance           |
| `planner` | `MotionPlanner`  | `MotionPlanner()`     | Planner to use (default: 10 pts)   |

---

### Methods

#### `move_to_joint_position(target) -> None`

Plans and executes a joint-space move to the given target.

```python
def move_to_joint_position(self, target: JointAngles) -> None
```

| Parameter | Type          | Description                |
|-----------|---------------|----------------------------|
| `target`  | `JointAngles` | Target joint configuration |

**Steps performed internally:**
1. `robot.get_joint_angles()` — reads current state
2. `planner.plan_joint_motion(current, target)` — builds path
3. `robot.run_motion(path)` — executes path

**Example:**
```python
from ur5lib import UR5Sim, MotionExecutor, JointAngles

robot = UR5Sim()
robot.connect_()

executor = MotionExecutor(robot)
executor.move_to_joint_position(JointAngles([0.1, -0.5, 0.3, -1.2, 1.5, 0.0]))
```

---

#### `move_to_pose(target) -> None`

Plans and executes a Cartesian-space move to the given TCP pose.

```python
def move_to_pose(self, target: Pose) -> None
```

| Parameter | Type   | Description          |
|-----------|--------|----------------------|
| `target`  | `Pose` | Target TCP pose      |

**Steps performed internally:**
1. `robot.get_current_pose()` — reads current state
2. `planner.plan_cartesian_motion(current, target)` — builds path
3. `robot.run_motion(path)` — executes path

**Example:**
```python
from ur5lib import UR5Sim, MotionExecutor, Pose

robot = UR5Sim()
robot.connect_()

executor = MotionExecutor(robot)
executor.move_to_pose(Pose(0.5, 0.1, 0.4, 0.0, 3.14, 0.0))
```

---

### Using a Custom Planner

```python
from ur5lib import UR5Sim, MotionExecutor, MotionPlanner, JointAngles

robot = UR5Sim()
robot.connect_()

planner = MotionPlanner(num_points=50)   # finer interpolation
executor = MotionExecutor(robot, planner=planner)

executor.move_to_joint_position(JointAngles([0.1, -0.5, 0.3, -1.2, 1.5, 0.0]))
```
