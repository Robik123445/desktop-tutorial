"""Core package setup with lazy imports for optional modules."""
from __future__ import annotations

from importlib import import_module
from typing import Any
import logging

from .logging_config import setup_logging, setup_night_logging

setup_logging()

# Try importing plugin helpers. If unavailable, create no-op fallbacks that
# only log the attempted use. The logger writes to ``log.txt`` so failures are
# still visible to developers.
try:  # pragma: no cover - exercised via tests
    from .plugin_manager import (
        load_plugins,
        get_plugin,
        get_all_plugins,
        reload_plugins,
        execute_plugin,
    )
except Exception:  # pragma: no cover - missing optional deps
    _pm_logger = logging.getLogger(__name__)
    _pm_logger.addHandler(logging.FileHandler("log.txt"))
    _pm_logger.setLevel(logging.WARNING)

    def load_plugins(*_args, **_kwargs) -> None:
        """No-op when plugin manager is unavailable."""
        _pm_logger.warning("load_plugins called but plugin_manager missing")

    def get_plugin(*_args, **_kwargs) -> None:
        """Return ``None`` when plugin manager is unavailable."""
        _pm_logger.warning("get_plugin called but plugin_manager missing")
        return None

    def get_all_plugins(*_args, **_kwargs) -> list:
        """Return empty list when plugin manager is unavailable."""
        _pm_logger.warning("get_all_plugins called but plugin_manager missing")
        return []

    def reload_plugins(*_args, **_kwargs) -> None:
        """No-op when plugin manager is unavailable."""
        _pm_logger.warning("reload_plugins called but plugin_manager missing")

    def execute_plugin(*_args, **_kwargs) -> None:
        """Return ``None`` when plugin manager is unavailable."""
        _pm_logger.warning("execute_plugin called but plugin_manager missing")
        return None


_LAZY_MODULES = {"plugin_manager", "plugin_marketplace", "workflow_manager"}

# mapping of lazily loaded functions to their modules
_LAZY_FUNCTIONS = {
    "run_live_detection": "cam_slicer.vision.debris_detector",
    "suggest_changes": "cam_slicer.ai.feedback_advisor",
    "move_to_pose": "cam_slicer.robotics.arm_control",
}


def __getattr__(name: str) -> Any:
    """Lazily import optional submodules or objects when accessed."""
    if name == "create_app":
        try:
            from .api_server import create_app as app
        except Exception as exc:  # pragma: no cover - optional FastAPI
            raise ImportError(
                "FastAPI components are not available. Install optional dependencies."
            ) from exc
        globals()[name] = app
        return app
    if name in _LAZY_MODULES:
        module = import_module(f".{name}", __name__)
        globals()[name] = module
        return module
    if name in _LAZY_FUNCTIONS:
        mod_name = _LAZY_FUNCTIONS[name]
        try:
            module = import_module(mod_name)
            obj = getattr(module, name)
        except Exception as exc:
            raise ImportError(f"Failed to import {name} from {mod_name}: {exc}") from exc
        globals()[name] = obj
        return obj
    raise AttributeError(f"module {__name__} has no attribute {name}")


def __dir__() -> list[str]:
    return sorted(
        list(globals().keys())
        + list(_LAZY_MODULES)
        + list(_LAZY_FUNCTIONS)
        + ["create_app"]
    )


__all__ = [
    "setup_logging",
    "setup_night_logging",
    "create_app",
    "load_plugins",
    "get_plugin",
    "get_all_plugins",
    "reload_plugins",
    "execute_plugin",
    *_LAZY_MODULES,
    *_LAZY_FUNCTIONS.keys(),
]
