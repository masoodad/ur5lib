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
