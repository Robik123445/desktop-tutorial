import json
import logging
import os
from pathlib import Path
from typing import Dict

CONFIG_PATH = Path(os.path.expanduser("~/.camslicer/camera.json"))
logger = logging.getLogger(__name__)

def ensure_dir() -> None:
    """Vytvor adresár pre config ak neexistuje."""
    CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)

def load_config() -> Dict[str, int]:
    """Načíta config JSON, ak chýba vráti prázdny slovník."""
    try:
        data = CONFIG_PATH.read_text(encoding="utf-8")
        return json.loads(data)
    except FileNotFoundError:
        return {}
    except Exception as exc:  # pragma: no cover
        logger.warning("Neplatný config: %s", exc)
        return {}

def save_config(cfg: Dict[str, int]) -> None:
    """Uloží config atomicky."""
    ensure_dir()
    tmp = CONFIG_PATH.with_suffix(".tmp")
    tmp.write_text(json.dumps(cfg), encoding="utf-8")
    tmp.replace(CONFIG_PATH)
    logger.info("Config uložený do %s", CONFIG_PATH)

def get_current_index(default: int = 0) -> int:
    """Vráti zvolený index kamery alebo default."""
    cfg = load_config()
    return int(cfg.get("index", default))

def set_current_index(idx: int) -> None:
    """Uloží aktuálny index kamery."""
    save_config({"index": int(idx)})
