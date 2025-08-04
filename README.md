# 🤖 UR5Lib

**UR5Lib** is a modular Python library designed to interface with and control a **Universal Robots UR5** robotic arm. It supports both **real hardware** via `ur_rtde` and **simulation** via custom simulators.

---

## 📦 Features

- ✅ Clean modular architecture
- ⚙️ RTDE support for real robot interaction
- 🧪 Simulator backend for testing without hardware
- 📚 Built-in motion planning and execution interfaces
- 💡 Examples and Jupyter notebooks to get started quickly
- 🔧 Configurable via YAML files
- 🧪 Test suite using `unittest`

---

## 📁 Project Structure

```bash
.
├── core.py                       # Base UR5 interface logic
├── cli.py                        # Command-line interface
├── config/
│   └── default_config.yaml       # Default robot config
├── exceptions.py                 # Custom exception classes
├── motion/                       # Motion planning and execution
│   ├── executor.py
│   ├── planner.py
│   └── __init__.py
├── io/                           # Interfaces for I/O (real/sim)
│   ├── ur_rtde.py
│   ├── simulator.py
│   └── __init__.py
├── tools/                        # Helper functions and math
│   ├── transforms.py
│   ├── utils.py
│   └── __init__.py
├── types/                        # Common data structures
│   ├── common_types.py
│   └── __init__.py
├── examples/                     # Example scripts and notebooks
│   ├── demo_joint_motion.py
│   ├── ur5lib_test.ipynb
│   └── test.py
├── tests/                        # Unit tests
│   ├── test_core.py
│   └── __init__.py
├── pyproject.toml                # PEP 517 build config
├── setup.py                      # setuptools build script
├── README.md                     # Project documentation





🚀 Quick Start
1. Clone the Repository
bash
Copy
Edit
git clone https://github.com/yourusername/ur5lib.git
cd ur5lib
2. Install (Editable Mode)
bash
Copy
Edit
pip install -e .
Ensure you have Python 3.7+ and ur_rtde if working with real robots.

3. Try an Example
Run a demo with simulated motion:

bash
Copy
Edit
python examples/demo_joint_motion.py
Or launch the CLI:

bash
Copy
Edit
ur5lib-cli --help
🛠️ Configuration
Robot parameters are managed via YAML:

yaml
Copy
Edit
# config/default_config.yaml
robot:
  ip: "192.168.0.10"
  port: 30004
  use_sim: true
You can override this in your scripts or by passing CLI arguments.

🧪 Running Tests
This project uses Python's built-in unittest framework.

bash
Copy
Edit
python -m unittest discover tests
Or use pytest if installed:

bash
Copy
Edit
pytest tests/
🧰 CLI Usage
The command-line interface is registered as:

bash
Copy
Edit
ur5lib-cli
You can extend it in cli.py to support features like:

bash
Copy
Edit
ur5lib-cli plan --target pose.yaml
ur5lib-cli execute --path planned_path.json
🧠 Modules Overview
Module	Description
core.py	Base class combining planner + executor
motion/	Planning & execution logic
io/	Communication layers (simulator/RTDE)
tools/	Utilities, math, transforms
types/	Typed definitions (poses, joints, etc.)
exceptions.py	Custom exception hierarchy

📓 Examples
See the examples/ directory for usage:

demo_joint_motion.py — run simulated motion

ur5lib_test.ipynb — notebook for interactive experiments

📦 Packaging
Build wheel and source distribution:

bash
Copy
Edit
python setup.py sdist bdist_wheel
Generated files will appear in dist/:

pgsql
Copy
Edit
dist/
├── ur5lib-0.1.0-py3-none-any.whl
├── ur5lib-0.1.0.tar.gz
📄 License
MIT License. See LICENSE for details.

👤 Author
Masood Ahmad
Robotics & Automation Engineer

🙋‍♂️ Contributing
Pull requests, issues, and suggestions welcome!
Fork the repo → create a branch → submit a PR ✅

🧭 Future Ideas
Support for UR10 / UR3

GUI for live visualization

ROS integration

Trajectory optimization

Live feedback from force sensors
```
