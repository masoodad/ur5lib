# CLI — ur5lib-cli

**Module:** `ur5lib.cli`  
**Entry point:** `ur5lib-cli`

A command-line interface for basic robot control without writing Python code.

---

## Usage

```
ur5lib-cli [--sim] [--ip IP] [--move-joints J1 J2 J3 J4 J5 J6]
```

---

## Arguments

| Argument                          | Type    | Default           | Description                                      |
|-----------------------------------|---------|-------------------|--------------------------------------------------|
| `--sim`                           | flag    | `False`           | Use the simulator instead of a real robot        |
| `--ip IP`                         | `str`   | `"192.168.0.100"` | IP address of the real robot (ignored with `--sim`) |
| `--move-joints J1 J2 J3 J4 J5 J6`| 6×`float` |               | Target joint angles in radians                   |

---

## Examples

**Move the simulator to a joint configuration:**
```bash
ur5lib-cli --sim --move-joints 0.1 -0.5 0.3 -1.2 1.5 0.0
```

**Move a real robot at a custom IP:**
```bash
ur5lib-cli --ip 192.168.1.42 --move-joints 0.0 -1.57 1.57 -1.57 -1.57 0.0
```

---

## Behavior

1. Parses arguments.
2. Creates a robot instance:
   - `--sim` → `UR5Sim()`
   - otherwise → `UR5RTDE(config={"robot_ip": <ip>})`
3. Calls `robot.connect_()`.
4. Creates a `MotionExecutor`.
5. If `--move-joints` is provided, calls `executor.move_to_joint_position(...)`.
