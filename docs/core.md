# Core — UR5Base

**Module:** `ur5lib.core`

`UR5Base` is the abstract base class that defines the interface all UR5 robot backends must implement. Both `UR5Sim` (simulator) and `UR5RTDE` (real robot) inherit from this class.

---

## Class: `UR5Base`

```python
class UR5Base(ABC):
    def __init__(self, config: Dict[str, Any] = None)
```

### Constructor Parameters

| Parameter | Type              | Default | Description              |
|-----------|-------------------|---------|--------------------------|
| `config`  | `Dict[str, Any]`  | `{}`    | Configuration dictionary |

**Recognized config keys:**

| Key    | Type  | Default | Description              |
|--------|-------|---------|--------------------------|
| `mode` | `str` | `"sim"` | Operating mode: `"sim"` or `"real"` |

---

## Properties

| Property    | Type               | Description                        |
|-------------|--------------------|------------------------------------|
| `config`    | `Dict[str, Any]`   | Configuration dictionary           |
| `connected` | `bool`             | Whether the robot is connected     |
| `mode`      | `str`              | Operating mode (`"sim"` or `"real"`) |
| `logger`    | `logging.Logger`   | Logger for this instance           |

---

## Abstract Methods

These methods **must** be implemented by every concrete subclass.

### `connect_rtde() -> None`

Establish the connection to the robot or simulator backend.

```python
@abstractmethod
def connect_rtde(self) -> None: ...
```

---

### `get_joint_angles() -> JointAngles`

Return the current joint angles of the robot.

```python
@abstractmethod
def get_joint_angles(self) -> JointAngles: ...
```

**Returns:** [`JointAngles`](types.md#jointangles) — six joint angles in radians.

---

### `get_current_pose() -> Pose`

Return the current TCP pose in Cartesian space.

```python
@abstractmethod
def get_current_pose(self) -> Pose: ...
```

**Returns:** [`Pose`](types.md#pose) — position (meters) and orientation (radians, axis-angle).

---

### `run_motion(motion_plan: Any) -> None`

Execute a pre-planned motion sequence.

```python
@abstractmethod
def run_motion(self, motion_plan: Any) -> None: ...
```

| Parameter     | Type  | Description                                      |
|---------------|-------|--------------------------------------------------|
| `motion_plan` | `Any` | List of waypoints (joint configs or Pose objects) |

---

## Concrete Methods

### `connect_() -> None`

Calls `connect_rtde()` and sets `connected = True`. This is the method callers should use to connect.

```python
def connect_(self) -> None: ...
```

**Example:**
```python
robot = UR5Sim()
robot.connect_()
```

---

### `validate_connection() -> None`

Raises `NotConnectedError` if `connected` is `False`. Called internally by methods that require an active connection.

```python
def validate_connection(self) -> None: ...
```

**Raises:** [`NotConnectedError`](exceptions.md#notconnectederror)

---

### `log(msg: str, level: str = "info") -> None`

Log a message via the instance logger.

```python
def log(self, msg: str, level: str = "info") -> None: ...
```

| Parameter | Type  | Default  | Description                              |
|-----------|-------|----------|------------------------------------------|
| `msg`     | `str` |          | Message to log                           |
| `level`   | `str` | `"info"` | Log level: `"info"`, `"warning"`, `"error"`, etc. |

---

## Implementing a Custom Backend

```python
from ur5lib.core import UR5Base
from ur5lib import JointAngles, Pose

class MyRobot(UR5Base):
    def connect_rtde(self):
        # establish connection
        pass

    def get_joint_angles(self) -> JointAngles:
        return JointAngles([0.0] * 6)

    def get_current_pose(self) -> Pose:
        return Pose(0, 0, 0, 0, 0, 0)

    def run_motion(self, motion_plan):
        for waypoint in motion_plan:
            # send waypoint to hardware
            pass
```
