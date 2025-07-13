"""Structured plugin loader for CAM Slicer.

This module discovers and loads Python plugins from :mod:`cam_slicer.plugins`.
Every plugin must define a :func:`register` function returning a
:class:`Plugin` instance.  Loading continues even if a plugin fails to import
so a broken plugin will not prevent others from working.  All events are logged
to ``logs/central.log`` by the shared logger.

Example
-------
>>> from cam_slicer.plugin_manager import load_plugins, get_all_plugins
>>> load_plugins()  # doctest: +SKIP
>>> get_all_plugins()  # doctest: +SKIP
[{"name": "reverse_path", "description": "Reverse toolpath", "category": None}]
"""

from __future__ import annotations

import importlib.util
import logging
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from typing import Callable, Dict, List, Optional
import multiprocessing as mp
import os



from cam_slicer.logging_config import setup_logging
setup_logging()
@dataclass
class Plugin:
    """Information container for a plugin.

    Attributes
    ----------
    name : str
        Human readable plugin identifier.
    description : str
        Short summary displayed in the plugin list.
    apply : Callable
        Callable executed when the plugin runs.  The signature depends on the
        plugin implementation.
    version : str, optional
        Version string or semantic version of the plugin.
    category : str, optional
        Optional plugin grouping used by UIs.

    Examples
    --------
    >>> def register():
    ...     def run(args):
    ...         return 'done'
    ...     return Plugin(name='demo', description='demo', apply=run)
    """

    name: str
    description: str
    apply: Callable
    version: Optional[str] = None
    category: Optional[str] = None
    module_path: Optional[str] = None


_PLUGINS: Dict[str, Plugin] = {}
_PLUGIN_DIR: Optional[Path] = None
_PREVIOUS_PLUGINS: Dict[str, Plugin] = {}


def _parse_version(v: Optional[str]) -> tuple:
    """Return version tuple for comparison."""
    if not v:
        return (0,)
    try:
        return tuple(int(x) for x in v.split('.'))
    except Exception:
        return (0,)


def load_plugins(plugin_dir: Optional[str | Path] = None) -> None:
    """Load all plugins found in ``plugin_dir``.

    Parameters
    ----------
    plugin_dir : str or Path, optional
        Directory containing plugin modules. When ``None`` the default
        ``cam_slicer/plugins`` directory is used.

    Notes
    -----
    A plugin is any Python file with a :func:`register` function returning a
    :class:`Plugin`.  Errors during import or registration are logged and
    skipped.  Successfully loaded plugins are stored globally and can be
    accessed via :func:`get_plugin`.

    Examples
    --------
    >>> from cam_slicer.plugin_manager import load_plugins
    >>> load_plugins('/path/to/plugins')  # doctest: +SKIP
    """
    global _PLUGINS, _PLUGIN_DIR, _PREVIOUS_PLUGINS

    previous = _PLUGINS.copy()
    _PLUGINS.clear()
    directory = Path(plugin_dir or Path(__file__).parent / "plugins")
    _PLUGIN_DIR = directory

    if not directory.exists():
        logging.warning("Plugin directory %s not found", directory)
        return

    loaded = 0
    failed = 0

    for path in directory.glob("*.py"):
        if path.name == "__init__.py":
            continue

        try:
            spec = importlib.util.spec_from_file_location(path.stem, path)
            if not spec or not spec.loader:
                raise ImportError(f"No spec for {path}")
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)  # type: ignore
        except Exception as exc:  # pragma: no cover - log only
            logging.exception("Failed importing %s", path.name)
            failed += 1
            continue

        if not hasattr(module, "register"):
            logging.error("%s missing register()", path.name)
            failed += 1
            continue

        try:
            info = module.register()  # type: ignore
        except Exception:  # pragma: no cover - log only
            logging.exception("%s register() failed", path.name)
            failed += 1
            continue

        if not isinstance(info, Plugin):
            logging.error("%s register() did not return Plugin", path.name)
            failed += 1
            continue

        if not info.name or not info.description or not callable(info.apply):
            logging.error("%s returned invalid Plugin", path.name)
            failed += 1
            continue

        info.module_path = str(path)

        existing = previous.get(info.name)
        if existing:
            if _parse_version(info.version) < _parse_version(existing.version):
                logging.warning(
                    "Ignoring older version %s of plugin %s",
                    info.version,
                    info.name,
                )
                _PLUGINS[info.name] = existing
                continue
            _PREVIOUS_PLUGINS[info.name] = existing

        _PLUGINS[info.name] = info
        loaded += 1
        logging.info("Loaded plugin %s", info.name)

    logging.info("Plugin loading complete: %d loaded, %d failed", loaded, failed)
    if failed:
        logging.warning("Restoring previous plugins due to %d failures", failed)
        for name, plug in previous.items():
            if name not in _PLUGINS:
                _PLUGINS[name] = plug


def get_plugin(name: str) -> Optional[Plugin]:
    """Return a previously loaded plugin.

    Parameters
    ----------
    name : str
        Name of the plugin as registered in :func:`load_plugins`.

    Returns
    -------
    Plugin or None
        The plugin object if found, otherwise ``None``.
    """
    return _PLUGINS.get(name)


def get_all_plugins() -> List[Dict[str, Optional[str]]]:
    """Return metadata for all loaded plugins.

    Returns
    -------
    list of dict
        Each dict contains ``name``, ``description`` and ``category`` keys.
    """
    return [
        {"name": p.name, "description": p.description, "category": p.category}
        for p in _PLUGINS.values()
    ]


def reload_plugins() -> None:
    """Reload plugins from the last directory passed to :func:`load_plugins`.

    Calling this function is useful during development when plugin files change
    and you want to refresh the registry without restarting the application.

    Examples
    --------
    >>> from cam_slicer.plugin_manager import load_plugins, reload_plugins
    >>> load_plugins()  # doctest: +SKIP
    >>> reload_plugins()  # doctest: +SKIP
    """
    logging.info("Reloading plugins from %s", _PLUGIN_DIR)
    load_plugins(_PLUGIN_DIR)


def execute_plugin(name: str, args: list | None = None, timeout: int = 30):
    """Run plugin in an isolated process and return its result."""
    plugin = get_plugin(name)
    if not plugin:
        raise ValueError(f"Plugin '{name}' not found")
    if not plugin.module_path:
        # fallback to direct call for backward compatibility
        return plugin.apply(args or [])

    def _runner(path: str, queue: mp.Queue, a: list):
        try:
            spec = importlib.util.spec_from_file_location(name, path)
            mod = importlib.util.module_from_spec(spec)
            assert spec and spec.loader
            spec.loader.exec_module(mod)  # type: ignore
            info = mod.register()  # type: ignore
            res = info.apply(a)
            queue.put(("ok", res))
        except Exception as exc:
            queue.put(("err", str(exc)))

    q: mp.Queue = mp.Queue()
    p = mp.Process(target=_runner, args=(plugin.module_path, q, args or []))
    p.start()
    p.join(timeout)
    if p.is_alive():
        p.terminate()
        raise RuntimeError("Plugin execution timeout")
    status, payload = q.get()
    if status == "err":
        raise RuntimeError(payload)
    return payload
