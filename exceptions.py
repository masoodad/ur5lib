# ur5lib/exceptions.py

class UR5Error(Exception):
    """Base class for all UR5-related exceptions."""
    pass


class NotConnectedError(UR5Error):
    """Raised when attempting to use UR5 before connecting."""
    pass


class InvalidConfigurationError(UR5Error):
    """Raised when configuration is missing or malformed."""
    pass


class JointLimitError(UR5Error):
    """Raised when a joint configuration violates hardware position limits."""
    pass


class SafetyViolationError(UR5Error):
    """Raised when a trajectory violates velocity or acceleration limits."""
    pass


class KinematicsError(UR5Error):
    """Raised when forward or inverse kinematics fails (e.g. IK non-convergence)."""
    pass


class ControlError(UR5Error):
    """Raised for general control-loop failures."""
    pass
