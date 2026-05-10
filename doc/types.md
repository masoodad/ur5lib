# Types

**Module:** `ur5lib.types.common_types`

This module defines the core data types used throughout the library.

---

## `JointAngles`

A named tuple representing the angular configuration of all six UR5 joints.

```python
JointAngles(joints: List[float])
```

| Field    | Type          | Description                                  |
|----------|---------------|----------------------------------------------|
| `joints` | `List[float]` | Six joint angles in radians, one per joint   |

### Example

```python
from ur5lib import JointAngles

# Home position (all zeros)
home = JointAngles([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# Custom configuration
config = JointAngles([0.1, -0.5, 0.3, -1.2, 1.5, 0.0])

# Access individual joints
print(config.joints[0])  # base joint angle
```

---

## `Pose`

A named tuple representing the position and orientation of the robot's TCP (Tool Center Point) in Cartesian space.

```python
Pose(x, y, z, rx, ry, rz)
```

| Field | Type    | Unit    | Description                        |
|-------|---------|---------|------------------------------------|
| `x`   | `float` | meters  | X position                         |
| `y`   | `float` | meters  | Y position                         |
| `z`   | `float` | meters  | Z position                         |
| `rx`  | `float` | radians | Rotation around X (axis-angle)     |
| `ry`  | `float` | radians | Rotation around Y (axis-angle)     |
| `rz`  | `float` | radians | Rotation around Z (axis-angle)     |

### Example

```python
from ur5lib import Pose

# Define a target pose
target = Pose(x=0.5, y=0.1, z=0.4, rx=0.0, ry=3.14, rz=0.0)

# Access individual fields
print(target.x, target.y, target.z)
```
