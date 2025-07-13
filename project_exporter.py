"""Export project files with metadata for Codex.

The exporter scans the repository and stores file contents along with
basic metadata such as section and status. A ``build_info`` block is
added for quick reference about dependencies and CLI entrypoints.
"""

import os
import json
import argparse
from datetime import datetime
from pathlib import Path
from typing import List, Dict
import logging

from cam_slicer.logging_config import setup_logging
import hashlib

SUPPORTED_EXTS = {
    '.py', '.json', '.yml', '.yaml', '.md', '.js', '.ts', '.tsx', '.jsx',
    '.txt', '.cfg', '.ini'
}
EXCLUDE_DIRS = {'.git', 'node_modules', '__pycache__'}


def categorize_file(path: Path) -> str:
    """Return top-level project section for *path*."""
    parts = path.as_posix().split('/')
    if not parts:
        return 'root'
    if parts[0] == 'cam_slicer' and len(parts) > 1:
        return parts[1]
    return parts[0]


def _status_for(path: Path) -> str:
    """Return stability status based on path."""
    parts = {p.lower() for p in path.parts}
    if 'experimental' in parts or 'vision' in parts:
        return 'experimental'
    if 'plugins' in parts:
        return 'beta'
    return 'stable'


def collect_files(root: Path) -> List[Dict[str, str]]:
    """Return a list of dictionaries with file metadata and content."""
    files = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in EXCLUDE_DIRS]
        for fname in filenames:
            if fname in {'export_codex.json', 'world-maple_codex.json'}:
                continue
            ext = Path(fname).suffix.lower()
            if ext not in SUPPORTED_EXTS:
                continue
            path = Path(dirpath) / fname
            try:
                with open(path, 'r', encoding='utf-8', errors='ignore') as f:
                    content = f.read()
            except Exception as exc:  # pragma: no cover - skip unreadable
                logging.warning('Failed to read %s: %s', path, exc)
                continue
            checksum = hashlib.sha256(content.encode('utf-8')).hexdigest()
            files.append({
                'filename': str(path.as_posix()),
                'full_content': content,
                'section': categorize_file(path.relative_to(root)),
                'description': '',
                'status': _status_for(path.relative_to(root)),
                'checksum': checksum,
            })
    return sorted(files, key=lambda x: x['filename'])


def generate_build_info(root: Path) -> Dict[str, object]:
    """Collect basic build information for the export."""
    req_file = root / 'requirements.txt'
    deps = []
    if req_file.exists():
        deps = [line.strip() for line in req_file.read_text().splitlines() if line.strip() and not line.startswith('#')]

    entrypoints = []
    cli_dir = root / 'cam_slicer'
    if cli_dir.exists():
        for path in cli_dir.glob('cli_*.py'):
            entrypoints.append(path.as_posix())

    project_name = 'CAM Slicer'
    readme = root / 'README.md'
    if readme.exists():
        first_line = readme.read_text().splitlines()[0].strip('#').strip()
        if first_line:
            project_name = first_line

    return {
        'version': '1.0.0',
        'timestamp': datetime.utcnow().isoformat(),
        'dependencies': deps,
        'entrypoints': sorted(entrypoints),
        'project': project_name,
    }


def save_export(files: List[Dict[str, str]], build_info: Dict[str, str], out_file: Path) -> None:
    """Save export list and build info to JSON file."""
    entries = files + [{'build_info': build_info}]
    with open(out_file, 'w', encoding='utf-8') as f:
        json.dump(entries, f, indent=2, ensure_ascii=False)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Export project files to JSON')
    parser.add_argument('--root', default='.', help='Project root directory')
    parser.add_argument('--output', default='export_codex.json', help='Output JSON file')
    return parser.parse_args(argv)


def main(argv=None) -> None:
    args = parse_args(argv)
    setup_logging()
    root = Path(args.root)
    files = collect_files(root)
    build_info = generate_build_info(root)
    save_export(files, build_info, Path(args.output))
    print(f'Saved {len(files)} files to {args.output}')


if __name__ == '__main__':
    main()
