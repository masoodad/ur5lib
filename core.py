# ur5lib/core.py

import abc
import logging
from typing import Dict, Any

from .exceptions import NotConnectedError
from .ur5_types.common_types import Pose, JointAngles


class UR5Base(abc.ABC):
    def __init__(self, config: Dict[str, Any] = None):
        self.config = config or {}
        self.connected = False
        self.mode = self.config.get("mode", "sim")  # 'sim' or 'real'

        # Setup logger
        self.logger = logging.getLogger("UR5")
        logging.basicConfig(level=logging.INFO)
        self.logger.info(f"UR5Base initialized in {self.mode.upper()} mode")

    # =============================
    # Connectors
    # =============================
    @abc.abstractmethod
    def connect_rtde(self):
        """Connect to RTDE interface"""
        pass

    def connect_(self):
        self.logger.info("Connecting...")
        self.connect_rtde()
        self.connected = True
        self.logger.info("Connection successful.")

    # =============================
    # Getters
    # =============================
    @abc.abstractmethod
    def get_joint_angles(self) -> JointAngles:
        """Get current joint angles"""
        pass

    @abc.abstractmethod
    def get_current_pose(self) -> Pose:
        """Get current end-effector pose"""
        pass

    # =============================
    # Runners
    # =============================
    @abc.abstractmethod
    def run_motion(self, motion_plan: Any):
        """Execute a motion plan"""
        pass

    # =============================
    # Utilities
    # =============================
    def log(self, msg: str, level: str = "info"):
        getattr(self.logger, level)(msg)

    def validate_connection(self):
        if not self.connected:
            raise NotConnectedError("UR5 is not connected. Use connect_() first.")
