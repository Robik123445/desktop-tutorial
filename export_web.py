import json
import logging
import re
from pathlib import Path
from typing import List, Dict

LOG_FILE = Path("log.txt")

def setup_logger() -> logging.Logger:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        handlers=[logging.FileHandler(LOG_FILE, encoding="utf-8")],
    )
    return logging.getLogger(__name__)

LOGGER = setup_logger()

def should_skip(path: Path) -> bool:
    for part in path.parts:
        if part.startswith('.') or part in {"node_modules", "dist", ".git"}:
            return True
    return False

def collect_files(root: Path) -> List[Path]:
    return [p for p in root.rglob('*') if p.is_file() and not should_skip(p.relative_to(root))]

def clean_content(text: str) -> str:
    # HTML komentáre
    text = re.sub(r'<!--.*?-->', '', text, flags=re.DOTALL)
    # JS/TS viacriadkové komentáre
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    # JS/TS jednoriadkové komentáre
    text = re.sub(r'//.*', '', text)
    # Odstráni prázdne riadky a nadbytočné medzery
    lines = [ln.strip() for ln in text.splitlines()]
    lines = [ln for ln in lines if ln]
    return '\n'.join(lines)

def export_web(root: Path = Path('web'), output: Path = Path('export_web.json')) -> List[Dict[str, str]]:
    LOGGER.info("Scanning %s", root)
    data = []
    for f in collect_files(root):
        rel = f.relative_to(root).as_posix()
        try:
            content = f.read_text(encoding='utf-8')
            cleaned = clean_content(content)
        except Exception as exc:
            LOGGER.error("Failed to read %s: %s", f, exc)
            continue
        data.append({'filename': rel, 'full_content': cleaned})
    with output.open('w', encoding='utf-8') as fh:
        json.dump(data, fh, ensure_ascii=False, indent=2)
    LOGGER.info("Exported %d cleaned files to %s", len(data), output)
    return data

if __name__ == '__main__':
    export_web()
