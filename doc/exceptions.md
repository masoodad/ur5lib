# Exceptions

**Module:** `ur5lib.exceptions`

All library-specific exceptions inherit from `UR5Error`, making it easy to catch any ur5lib error with a single `except` clause.

---

## Exception Hierarchy

```
Exception
└── UR5Error
    ├── NotConnectedError
    └── InvalidConfigurationError
```

---

## `UR5Error`

Base class for all ur5lib exceptions.

```python
class UR5Error(Exception): ...
```

Catch this to handle any error raised by the library:

```python
from ur5lib.exceptions import UR5Error

try:
    robot.get_joint_angles()
except UR5Error as e:
    print(f"Robot error: {e}")
```

---

## `NotConnectedError`

Raised when a robot method is called before `connect_()` has been called.

```python
class NotConnectedError(UR5Error): ...
```

**Raised by:** `get_joint_angles`, `get_current_pose`, `run_motion`, `moveL`, `servoJ`, `servoL` — any method that calls `validate_connection()`.

```python
from ur5lib import UR5RTDE
from ur5lib.exceptions import NotConnectedError

robot = UR5RTDE()
# forgot to call robot.connect_()

try:
    robot.get_joint_angles()
except NotConnectedError:
    print("Connect first with robot.connect_()")
```

---

## `InvalidConfigurationError`

Raised when the provided configuration dictionary is missing required keys or contains malformed values.

```python
class InvalidConfigurationError(UR5Error): ...
```

```python
from ur5lib.exceptions import InvalidConfigurationError

try:
    robot = UR5RTDE(config={"robot_ip": None})
    robot.connect_()
except InvalidConfigurationError as e:
    print(f"Bad config: {e}")
```
