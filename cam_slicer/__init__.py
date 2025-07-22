"""Core package setup with lazy imports for optional modules."""
from __future__ import annotations

from importlib import import_module
from typing import Any

from .logging_config import setup_logging, setup_night_logging

setup_logging()

_LAZY_MODULES = {"plugin_manager", "plugin_marketplace", "workflow_manager"}


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
    raise AttributeError(f"module {__name__} has no attribute {name}")


def __dir__() -> list[str]:
    return sorted(list(globals().keys()) + list(_LAZY_MODULES) + ["create_app"])


__all__ = ["setup_logging", "setup_night_logging", "create_app", *_LAZY_MODULES]
