import json
import logging
from pathlib import Path

from cam_slicer.logging_config import setup_logging
setup_logging()
# Ensure logging writes to the central log file

MACROS_FILE = Path(__file__).with_name('macros.json')


def get_macro(name: str) -> list[str]:
    """Return macro lines by name."""
    try:
        with MACROS_FILE.open('r', encoding='utf-8') as fh:
            macros = json.load(fh)
    except FileNotFoundError:
        logging.error('Macro file not found: %s', MACROS_FILE)
        raise
    if name not in macros:
        logging.error('Macro %s not defined', name)
        raise KeyError(f'Macro {name} not defined')
    logging.info('Loaded macro: %s', name)
    return macros[name]
