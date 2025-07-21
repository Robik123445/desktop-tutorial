"""Core package setup and thin convenience imports."""
from __future__ import annotations

from .logging_config import setup_logging, setup_night_logging

setup_logging()

# lightweight re-exports used in tests
try:
    from .api_server import create_app  # type: ignore
except Exception:  # pragma: no cover - optional FastAPI
    create_app = None  # type: ignore
from . import plugin_manager  # noqa: F401
from . import plugin_marketplace  # noqa: F401
from . import workflow_manager  # noqa: F401

__all__ = [
    "setup_logging",
    "setup_night_logging",
    "create_app",
    "plugin_manager",
    "plugin_marketplace",
    "workflow_manager",
]
