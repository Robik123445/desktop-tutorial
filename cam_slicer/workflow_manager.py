"""Workflow sharing utilities."""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Dict, Any

from .logging_config import setup_logging

setup_logging()

WORKFLOW_DIR = Path(__file__).parent / "workflows"


def save_workflow(data: Dict[str, Any], name: str) -> Path:
    """Save workflow as JSON in the workflows directory."""
    WORKFLOW_DIR.mkdir(exist_ok=True)
    path = WORKFLOW_DIR / f"{name}.json"
    path.write_text(json.dumps(data, indent=2))
    logging.info("Saved workflow %s", name)
    return path


def load_workflow(path: str | Path) -> Dict[str, Any]:
    """Load workflow JSON from file."""
    p = Path(path)
    try:
        data = json.loads(p.read_text())
        logging.info("Loaded workflow %s", p.name)
        return data
    except Exception as exc:  # pragma: no cover - log and raise
        logging.exception("Failed loading workflow %s", p)
        raise


def list_workflows(dir_path: str | Path = WORKFLOW_DIR) -> list[str]:
    """List available workflow names."""
    directory = Path(dir_path)
    if not directory.exists():
        return []
    return [p.stem for p in directory.glob("*.json")]

