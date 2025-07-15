"""Utilities for analyzing basic properties of G-code."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Sequence, Dict

from .logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)

# Ensure extra log file for this module
_log_path = Path("logs/log.txt")
if not any(
    isinstance(h, logging.FileHandler) and getattr(h, "baseFilename", "") == str(_log_path)
    for h in logging.getLogger().handlers
):
    _log_path.parent.mkdir(exist_ok=True)
    _h = logging.FileHandler(_log_path)
    _h.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logging.getLogger().addHandler(_h)


def parse_gcode_text(text: str) -> list[str]:
    """Return cleaned list of G-code commands."""
    return [ln.strip() for ln in text.splitlines() if ln.strip() and not ln.strip().startswith(";")]


def compute_gcode_stats(lines: Sequence[str]) -> Dict[str, object]:
    """Compute simple statistics from G-code lines."""
    moves = 0
    max_feed = 0.0
    min_x = min_y = min_z = float("inf")
    max_x = max_y = max_z = float("-inf")
    for line in lines:
        tokens = line.upper().split()
        if not tokens:
            continue
        cmd = tokens[0]
        if cmd in {"G0", "G1", "G2", "G3"}:
            moves += 1
        for t in tokens[1:]:
            if len(t) < 2:
                continue
            prefix, val_str = t[0], t[1:]
            try:
                val = float(val_str)
            except ValueError:
                continue
            if prefix == "X":
                min_x = min(min_x, val)
                max_x = max(max_x, val)
            elif prefix == "Y":
                min_y = min(min_y, val)
                max_y = max(max_y, val)
            elif prefix == "Z":
                min_z = min(min_z, val)
                max_z = max(max_z, val)
            elif prefix == "F":
                max_feed = max(max_feed, val)
    bounds = {
        "min_x": 0.0 if min_x == float("inf") else min_x,
        "max_x": 0.0 if max_x == float("-inf") else max_x,
        "min_y": 0.0 if min_y == float("inf") else min_y,
        "max_y": 0.0 if max_y == float("-inf") else max_y,
        "min_z": 0.0 if min_z == float("inf") else min_z,
        "max_z": 0.0 if max_z == float("-inf") else max_z,
    }
    stats = {
        "line_count": len(lines),
        "move_count": moves,
        "max_feed_rate": round(max_feed, 4),
        "bounds": bounds,
    }
    logger.info("Computed gcode stats: %s", stats)
    return stats
