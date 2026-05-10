# Examples

---

## 1. Simulator — Joint Motion

Move the simulated robot to a joint configuration.

```python
from ur5lib import UR5Sim, MotionExecutor, JointAngles

robot = UR5Sim()
robot.connect_()

executor = MotionExecutor(robot)
executor.move_to_joint_position(JointAngles([0.1, -0.5, 0.3, -1.2, 1.5, 0.0]))
```

Source: `examples/demo_joint_motion.py`

---

## 2. Simulator — Cartesian Motion

Move the simulated robot's TCP to a target pose.

```python
from ur5lib import UR5Sim, MotionExecutor, Pose

robot = UR5Sim()
robot.connect_()

executor = MotionExecutor(robot)
executor.move_to_pose(Pose(x=0.5, y=0.1, z=0.4, rx=0.0, ry=3.14, rz=0.0))
```

---

## 3. Real Robot — Connect and Read State

```python
from ur5lib import UR5RTDE

robot = UR5RTDE(config={"robot_ip": "192.168.0.100"})
robot.connect_()

print("Joint angles:", robot.get_joint_angles())
print("TCP pose:    ", robot.get_current_pose())
```

---

## 4. Real Robot — Linear Move (moveL)

Move the TCP in a straight line in Cartesian space.

```python
from ur5lib import UR5RTDE, Pose

robot = UR5RTDE(config={"robot_ip": "192.168.0.100"})
robot.connect_()

target = Pose(x=0.5, y=0.1, z=0.4, rx=0.0, ry=3.14, rz=0.0)
robot.moveL(target, speed=0.1, acceleration=0.3)
```

---

## 5. Real Robot — Real-Time Servo Control (servoJ)

Stream joint targets at high frequency (e.g., for teleoperation or sensor-guided motion).

```python
import time
from ur5lib import UR5RTDE, JointAngles

robot = UR5RTDE(config={"robot_ip": "192.168.0.100"})
robot.connect_()

# Send 100 servo commands at ~125 Hz
for i in range(100):
    target = JointAngles([0.001 * i, -1.57, 1.57, -1.57, -1.57, 0.0])
    robot.servoJ(target, time=0.008)
    time.sleep(0.008)
```

---

## 6. Real Robot — Real-Time Servo Control (servoL)

Stream TCP pose targets at high frequency.

```python
import time
from ur5lib import UR5RTDE, Pose

robot = UR5RTDE(config={"robot_ip": "192.168.0.100"})
robot.connect_()

for i in range(100):
    target = Pose(0.4 + 0.001 * i, 0.1, 0.4, 0.0, 3.14, 0.0)
    robot.servoL(target, time=0.008)
    time.sleep(0.008)
```

---

## 7. Custom Planner Resolution

Use more interpolation points for smoother motion.

```python
from ur5lib import UR5Sim, MotionExecutor, MotionPlanner, JointAngles

robot = UR5Sim()
robot.connect_()

planner  = MotionPlanner(num_points=100)
executor = MotionExecutor(robot, planner=planner)

executor.move_to_joint_position(JointAngles([0.1, -0.5, 0.3, -1.2, 1.5, 0.0]))
```

---

## 8. Error Handling

```python
from ur5lib import UR5RTDE
from ur5lib.exceptions import NotConnectedError, UR5Error

robot = UR5RTDE(config={"robot_ip": "192.168.0.100"})

try:
    robot.connect_()
    angles = robot.get_joint_angles()
    print(angles)
except NotConnectedError:
    print("Robot is not connected.")
except UR5Error as e:
    print(f"Robot error: {e}")
```
