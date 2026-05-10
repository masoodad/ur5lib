# IO / Backends

This module provides two concrete implementations of [`UR5Base`](core.md): a software simulator and a real-robot RTDE client.

---

## UR5Sim

**Module:** `ur5lib.io.simulator`

A software-only simulator. No hardware is required. Simulated sensor readings include small random noise/drift to mimic real behavior.

### Class: `UR5Sim(UR5Base)`

```python
class UR5Sim(UR5Base):
    def __init__(self, config: Dict[str, Any] = None)
```

#### Constructor Parameters

| Parameter | Type             | Default | Description              |
|-----------|------------------|---------|--------------------------|
| `config`  | `Dict[str, Any]` | `{}`    | Configuration dictionary |

#### Internal State

| Attribute     | Type          | Initial Value              | Description                     |
|---------------|---------------|----------------------------|---------------------------------|
| `fake_pose`   | `Pose`        | `(0.1, 0.2, 0.3, 0, 0, 0)` | Simulated TCP pose              |
| `fake_joints` | `JointAngles` | `[0, 0, 0, 0, 0, 0]`       | Simulated joint angles          |

---

### Methods

#### `connect_rtde() -> None`

Logs `"Simulator connected (no real robot)."`. No hardware interaction.

---

#### `get_joint_angles() -> JointAngles`

Returns `fake_joints` with small random jitter (±0.01 rad) on each joint to simulate measurement noise.

```python
angles = robot.get_joint_angles()
print(angles.joints)  # [~0.0, ~0.0, ...]
```

---

#### `get_current_pose() -> Pose`

Returns `fake_pose` with small random drift (±0.001 m) on x, y, z.

```python
pose = robot.get_current_pose()
print(pose.x, pose.y, pose.z)
```

---

#### `run_motion(motion_plan) -> None`

Simulates motion by sleeping 0.1 s per waypoint and logging progress.

| Parameter     | Type   | Description            |
|---------------|--------|------------------------|
| `motion_plan` | `list` | List of waypoints      |

---

### Example

```python
from ur5lib import UR5Sim, MotionExecutor, JointAngles

robot = UR5Sim()
robot.connect_()

print(robot.get_joint_angles())
print(robot.get_current_pose())

executor = MotionExecutor(robot)
executor.move_to_joint_position(JointAngles([0.1, -0.5, 0.3, -1.2, 1.5, 0.0]))
```

---

---

## UR5RTDE

**Module:** `ur5lib.io.ur_rtde`

Controls a real UR5 robot over Ethernet using the RTDE protocol. Requires the `ur_rtde` Python package.

**Dependency:** `pip install ur_rtde`

### Class: `UR5RTDE(UR5Base)`

```python
class UR5RTDE(UR5Base):
    def __init__(self, config: Dict[str, Any] = None)
```

#### Constructor Parameters

| Parameter | Type             | Default | Description              |
|-----------|------------------|---------|--------------------------|
| `config`  | `Dict[str, Any]` | `{}`    | Configuration dictionary |

**Recognized config keys:**

| Key        | Type  | Default          | Description              |
|------------|-------|------------------|--------------------------|
| `robot_ip` | `str` | `"172.22.22.2"`  | IP address of the robot  |

#### Internal State

| Attribute | Type                        | Description                    |
|-----------|-----------------------------|--------------------------------|
| `rtde_c`  | `RTDEControlInterface`      | RTDE control interface         |
| `rtde_r`  | `RTDEReceiveInterface`      | RTDE receive interface         |
| `robot_ip`| `str`                       | Robot IP address               |

---

### Methods

#### `connect_rtde() -> None`

Initializes both RTDE interfaces (`rtde_c` and `rtde_r`) using `robot_ip`.

**Raises:**
- `ImportError` — if `ur_rtde` package is not installed.

```python
robot = UR5RTDE(config={"robot_ip": "192.168.0.100"})
robot.connect_()
```

---

#### `get_joint_angles() -> JointAngles`

Reads current joint angles from the robot via `rtde_r.getActualQ()`.

**Returns:** [`JointAngles`](types.md#jointangles)  
**Raises:** [`NotConnectedError`](exceptions.md#notconnectederror)

```python
angles = robot.get_joint_angles()
```

---

#### `get_current_pose() -> Pose`

Reads current TCP pose from the robot via `rtde_r.getActualTCPPose()`.

**Returns:** [`Pose`](types.md#pose)  
**Raises:** [`NotConnectedError`](exceptions.md#notconnectederror)

```python
pose = robot.get_current_pose()
```

---

#### `run_motion(motion_plan) -> None`

Executes a sequence of joint configurations using `moveJ`.

| Parameter     | Type   | Description                                           |
|---------------|--------|-------------------------------------------------------|
| `motion_plan` | `list` | List of joint configs (each: list/tuple of 6 floats)  |

- Speed: `0.5` rad/s (fixed)
- **Raises:** [`NotConnectedError`](exceptions.md#notconnectederror)

---

#### `moveL(pose, speed=0.25, acceleration=0.5, blend_radius=0.0, asynchronous=False) -> None`

Moves the TCP in a straight line in Cartesian space to `pose`.

| Parameter      | Type    | Default | Description                                          |
|----------------|---------|---------|------------------------------------------------------|
| `pose`         | `Pose`  |         | Target TCP pose                                      |
| `speed`        | `float` | `0.25`  | Linear speed in m/s                                  |
| `acceleration` | `float` | `0.5`   | Linear acceleration in m/s²                          |
| `blend_radius` | `float` | `0.0`   | Blending radius in meters (0 = no blending)          |
| `asynchronous` | `bool`  | `False` | If `True`, returns immediately without waiting       |

**Raises:** [`NotConnectedError`](exceptions.md#notconnectederror)

```python
from ur5lib import Pose

robot.moveL(Pose(0.5, 0.1, 0.4, 0.0, 3.14, 0.0), speed=0.1)
```

---

#### `servoJ(joint_angles, speed=1.0, acceleration=1.0, time=0.008, lookahead_time=0.1, gain=300) -> None`

Real-time servo control in joint space. Suitable for streaming high-frequency joint targets.

| Parameter       | Type          | Default | Description                                             |
|-----------------|---------------|---------|---------------------------------------------------------|
| `joint_angles`  | `JointAngles` |         | Target joint configuration                              |
| `speed`         | `float`       | `1.0`   | Speed in rad/s                                          |
| `acceleration`  | `float`       | `1.0`   | Acceleration in rad/s²                                  |
| `time`          | `float`       | `0.008` | Duration of move in seconds                             |
| `lookahead_time`| `float`       | `0.1`   | Path smoothing lookahead in seconds                     |
| `gain`          | `int`         | `300`   | Servo gain — higher value = stiffer response            |

**Raises:** [`NotConnectedError`](exceptions.md#notconnectederror)

```python
from ur5lib import JointAngles

robot.servoJ(JointAngles([0.0, -1.57, 1.57, -1.57, -1.57, 0.0]))
```

---

#### `servoL(pose, speed=0.25, acceleration=0.5, time=0.008, lookahead_time=0.1, gain=300) -> None`

Real-time servo control in Cartesian space. Suitable for streaming high-frequency TCP targets.

| Parameter        | Type    | Default | Description                                             |
|------------------|---------|---------|----------------------------------------------------------| 
| `pose`           | `Pose`  |         | Target TCP pose                                         |
| `speed`          | `float` | `0.25`  | Speed in m/s                                            |
| `acceleration`   | `float` | `0.5`   | Acceleration in m/s²                                    |
| `time`           | `float` | `0.008` | Duration of move in seconds                             |
| `lookahead_time` | `float` | `0.1`   | Path smoothing lookahead in seconds                     |
| `gain`           | `int`   | `300`   | Servo gain — higher value = stiffer response            |

**Raises:** [`NotConnectedError`](exceptions.md#notconnectederror)

```python
robot.servoL(Pose(0.5, 0.1, 0.4, 0.0, 3.14, 0.0))
```

---

### Comparison: moveL vs servoL

| Feature           | `moveL`                        | `servoL`                              |
|-------------------|--------------------------------|---------------------------------------|
| Control mode      | Trajectory following           | Real-time streaming                   |
| Blocking          | Configurable (`asynchronous`)  | Non-blocking                          |
| Latency           | Higher                         | Low (≈ 8 ms cycle)                   |
| Use case          | Point-to-point moves           | Sensor-guided or teleoperation        |

### Comparison: servoJ vs servoL

| Feature    | `servoJ`           | `servoL`                |
|------------|--------------------|-------------------------|
| Space      | Joint space        | Cartesian space         |
| Input      | `JointAngles`      | `Pose`                  |
| Use case   | Joint-level control| TCP-level control       |
