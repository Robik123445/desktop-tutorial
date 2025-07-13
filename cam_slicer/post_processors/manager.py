from __future__ import annotations

import importlib.util
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional

from cam_slicer.logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)


@dataclass
class PostProcessor:
    """Metadata and apply callable for a post-processor plugin."""

    name: str
    description: str
    firmware_patterns: List[str]
    apply: Callable[[List[str], dict], List[str]]
    version: Optional[str] = None
    category: Optional[str] = None
    module_path: Optional[str] = None


_POST_PROCESSORS: Dict[str, PostProcessor] = {}
_DIR: Optional[Path] = None


def load_post_processors(directory: Optional[str | Path] = None) -> None:
    """Load post-processor plugins from directory."""
    global _DIR, _POST_PROCESSORS
    _POST_PROCESSORS.clear()
    _DIR = Path(directory or Path(__file__).parent)
    if not _DIR.exists():
        logger.warning("Post-processor directory %s not found", _DIR)
        return
    for path in _DIR.glob("*.py"):
        if path.name == "__init__.py" or path.name == "manager.py":
            continue
        try:
            spec = importlib.util.spec_from_file_location(path.stem, path)
            assert spec and spec.loader
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)  # type: ignore
        except Exception as exc:
            logger.error("Failed to import %s: %s", path.name, exc)
            continue
        if not hasattr(module, "register"):
            logger.error("%s missing register()", path.name)
            continue
        try:
            info = module.register()  # type: ignore
        except Exception as exc:
            logger.error("%s register() failed: %s", path.name, exc)
            continue
        if not isinstance(info, PostProcessor):
            logger.error("%s register() did not return PostProcessor", path.name)
            continue
        info.module_path = str(path)
        _POST_PROCESSORS[info.name] = info
        logger.info("Loaded post-processor %s", info.name)


def reload_post_processors() -> None:
    """Reload post-processors from last directory."""
    load_post_processors(_DIR)


def get_post_processor(name: str) -> Optional[PostProcessor]:
    return _POST_PROCESSORS.get(name)


def get_all_post_processors() -> List[Dict[str, Optional[str]]]:
    return [
        {
            "name": p.name,
            "description": p.description,
            "category": p.category,
            "version": p.version,
        }
        for p in _POST_PROCESSORS.values()
    ]


def match_post_processor(firmware: str) -> Optional[PostProcessor]:
    fw = firmware.lower()
    for pp in _POST_PROCESSORS.values():
        if any(p.lower() in fw for p in pp.firmware_patterns):
            return pp
    return None


# --- Detection and automatic processing ---


def detect_machine_firmware(
    port: str,
    baud: int = 115200,
    timeout: float = 2.0,
    test_response: str | None = None,
) -> str:
    """Return firmware info string from the connected machine."""
    if test_response is not None:
        logger.info("Using test firmware response: %s", test_response)
        return test_response
    try:
        import serial  # type: ignore
    except Exception as exc:  # pragma: no cover - optional dependency
        logger.error("pyserial missing: %s", exc)
        raise
    import time

    with serial.Serial(port, baud, timeout=timeout) as ser:
        ser.write(b"\r\n")
        time.sleep(0.1)
        ser.flushInput()
        ser.write(b"$I\n")
        time.sleep(0.1)
        lines: List[str] = []
        end = time.time() + timeout
        while time.time() < end:
            line = ser.readline()
            if not line:
                break
            lines.append(line.decode("utf-8", errors="ignore").strip())
    info = " ".join(lines)
    logger.info("Firmware detection: %s", info)
    return info


def auto_post_process(
    gcode_lines: List[str],
    port: str,
    baud: int = 115200,
    **kwargs,
) -> List[str]:
    """Detect firmware on ``port`` and apply matching post-processor."""
    firmware = detect_machine_firmware(port, baud, kwargs.pop("timeout", 2.0))
    pp = match_post_processor(firmware)
    if not pp:
        logger.warning("No post-processor for firmware '%s'", firmware)
        return gcode_lines
    logger.info("Using post-processor %s", pp.name)
    return pp.apply(gcode_lines, {"firmware": firmware, **kwargs})
