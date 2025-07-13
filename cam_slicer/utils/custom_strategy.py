"""Load and execute user-defined toolpath strategies."""

from __future__ import annotations

import importlib.util
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, List, Any, Dict

from cam_slicer.logging_config import setup_logging

setup_logging()


@dataclass
class Strategy:
    """Container for a custom toolpath strategy."""

    name: str
    description: str
    apply: Callable[[List[Any], Dict[str, Any]], List[Any]]
    path: str | None = None


def load_strategy(script_path: str) -> Strategy:
    """Load a strategy from a Python file.

    The file must define a ``register()`` function returning a :class:`Strategy`.
    Any errors are logged and raised to the caller.
    """
    path = Path(script_path)
    if not path.exists():
        raise FileNotFoundError(path)

    spec = importlib.util.spec_from_file_location(path.stem, path)
    if not spec or not spec.loader:
        raise ImportError(f"Cannot load spec from {path}")
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)  # type: ignore
    except Exception as exc:  # pragma: no cover - import errors
        logging.exception("Failed importing strategy %s", path)
        raise

    if not hasattr(module, "register"):
        raise AttributeError(f"{path} missing register()")

    info = module.register()  # type: ignore
    if not isinstance(info, Strategy):
        raise TypeError("register() must return Strategy")
    info.path = str(path)
    logging.info("Loaded strategy %s", info.name)
    return info


def run_strategy(toolpath: List[Any], strategy: Strategy, **params: Any) -> List[Any]:
    """Execute ``strategy.apply`` on the given toolpath."""
    logging.info("Running strategy %s", strategy.name)
    return strategy.apply(toolpath, params)


__all__ = ["Strategy", "load_strategy", "run_strategy"]
