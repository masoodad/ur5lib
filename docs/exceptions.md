# Exceptions

**Module:** `ur5lib.exceptions`

All library-specific exceptions inherit from `UR5Error`, making it easy to catch any ur5lib error with a single `except` clause.

---

## Exception Hierarchy

```
Exception
└── UR5Error
    ├── NotConnectedError
    ├── InvalidConfigurationError
    ├── JointLimitError          ★ NEW
    ├── SafetyViolationError     ★ NEW
    ├── KinematicsError          ★ NEW
    └── ControlError             ★ NEW
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

---

## `JointLimitError` ★ NEW

Raised by `SafetyChecker.validate_configuration()` when a joint angle
exceeds the UR5 hardware position limits (±360° / ±2π rad per joint).

```python
class JointLimitError(UR5Error): ...
```

```python
from ur5lib.control import SafetyChecker
from ur5lib.exceptions import JointLimitError

safety = SafetyChecker()
try:
    safety.validate_configuration([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
except JointLimitError as e:
    print(e)  # Joint 1: 572.96° outside [-360°, 360°]
```

---

## `SafetyViolationError` ★ NEW

Raised by `SafetyChecker.validate_trajectory()` when a trajectory exceeds
per-joint velocity or acceleration limits.

```python
class SafetyViolationError(UR5Error): ...
```

```python
from ur5lib.control import SafetyChecker
from ur5lib.exceptions import SafetyViolationError

safety = SafetyChecker()
try:
    safety.validate_trajectory(my_traj, dt=0.008)
except SafetyViolationError as e:
    print(e)
```

---

## `KinematicsError` ★ NEW

Raised by `ik_numerical()` when the iterative IK solver does not converge
within the specified tolerance and iteration limit.

```python
class KinematicsError(UR5Error): ...
```

```python
from ur5lib.control import ik_numerical
from ur5lib.exceptions import KinematicsError

try:
    q = ik_numerical(unreachable_pose, q_init=[0]*6, max_iter=50)
except KinematicsError as e:
    print(e)  # IK did not converge after 50 iterations ...
```

---

## `ControlError` ★ NEW

Base class for general control-loop failures not covered by the more
specific exceptions above.

```python
class ControlError(UR5Error): ...
```
