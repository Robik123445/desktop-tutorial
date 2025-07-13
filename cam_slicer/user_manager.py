import logging
from dataclasses import dataclass
from typing import Dict

from cam_slicer.logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)


@dataclass
class User:
    """Simple user model."""

    username: str
    role: str  # admin, operator, viewer


class UserManager:
    """Manage users and permissions."""

    ROLE_PERMS = {
        "admin": {"add_machine", "remove_machine", "run_job", "configure"},
        "operator": {"run_job"},
        "viewer": set(),
    }

    def __init__(self) -> None:
        self.users: Dict[str, User] = {}

    def add_user(self, username: str, role: str) -> None:
        self.users[username] = User(username, role)
        logger.info("Added user %s with role %s", username, role)

    def remove_user(self, username: str) -> None:
        if username in self.users:
            del self.users[username]
            logger.info("Removed user %s", username)

    def check_permission(self, username: str, action: str) -> bool:
        user = self.users.get(username)
        if not user:
            logger.warning("Unknown user %s", username)
            return False
        allowed = action in self.ROLE_PERMS.get(user.role, set())
        if not allowed:
            logger.warning("User %s lacks permission for %s", username, action)
        return allowed
