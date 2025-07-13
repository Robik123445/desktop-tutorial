"""Simple plugin marketplace utilities."""

from __future__ import annotations

import json
import logging
import shutil
from pathlib import Path
from typing import List, Dict

from .logging_config import setup_logging

setup_logging()

MARKET_DIR = Path(__file__).parent / "plugins_market"
PLUGIN_DIR = Path(__file__).parent / "plugins"


def list_available_plugins(market_dir: Path | str = MARKET_DIR) -> List[Dict]:
    """Return metadata for all marketplace plugins."""
    directory = Path(market_dir)
    plugins = []
    if not directory.exists():
        logging.warning("Marketplace %s not found", directory)
        return plugins
    for meta in directory.glob("*.json"):
        try:
            data = json.loads(meta.read_text())
            plugins.append(data)
        except Exception as exc:  # pragma: no cover - log and continue
            logging.exception("Failed reading %s", meta)
    return plugins


def _plugin_path(name: str, market_dir: Path) -> Path:
    return market_dir / f"{name}.py"


def install_plugin(name: str, market_dir: Path | str = MARKET_DIR, plugin_dir: Path | str = PLUGIN_DIR) -> bool:
    """Install plugin from marketplace."""
    market = Path(market_dir)
    dest_dir = Path(plugin_dir)
    src = _plugin_path(name, market)
    dest = dest_dir / f"{name}.py"
    if not src.exists():
        logging.error("Plugin %s not found in marketplace", name)
        return False
    dest_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dest)
    logging.info("Installed plugin %s", name)
    return True


def update_plugin(name: str, market_dir: Path | str = MARKET_DIR, plugin_dir: Path | str = PLUGIN_DIR) -> bool:
    """Update plugin by reinstalling from marketplace."""
    return install_plugin(name, market_dir, plugin_dir)


def remove_plugin(name: str, plugin_dir: Path | str = PLUGIN_DIR) -> bool:
    """Remove installed plugin."""
    dest = Path(plugin_dir) / f"{name}.py"
    if dest.exists():
        dest.unlink()
        logging.info("Removed plugin %s", name)
        return True
    logging.warning("Plugin %s not installed", name)
    return False

