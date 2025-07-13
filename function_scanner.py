"""Utility to list all top-level Python functions as JSON."""

import os
import ast
import json
import logging
import argparse
from typing import List, Dict

from cam_slicer.logging_config import setup_logging


def collect_functions(root: str = '.') -> List[Dict[str, str]]:
    """Recursively collect top-level functions.

    Args:
        root: Directory to scan.

    Returns:
        List of dicts with file, function, and first docstring line.
    """
    functions = []
    for dirpath, _, filenames in os.walk(root):
        for filename in filenames:
            if not filename.endswith('.py'):
                continue
            path = os.path.join(dirpath, filename)
            try:
                with open(path, 'r', encoding='utf-8', errors='ignore') as f:
                    source = f.read()
                module = ast.parse(source)
            except Exception as exc:  # pragma: no cover - logging only
                logging.warning('Failed to parse %s: %s', path, exc)
                continue
            for node in module.body:
                if isinstance(node, ast.FunctionDef):
                    doc = ast.get_docstring(node)
                    first = doc.splitlines()[0].strip() if doc else ''
                    functions.append({
                        'file': os.path.relpath(path, root),
                        'function': node.name,
                        'doc': first
                    })
    return functions


def print_functions_list(functions: List[Dict[str, str]]) -> str:
    """Print collected functions as JSON and return the string."""
    result = json.dumps(functions, indent=2)
    print(result)
    logging.info("Found %d functions", len(functions))
    return result


def parse_args(argv=None) -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="Scan Python files for functions")
    parser.add_argument(
        "root",
        nargs="?",
        default=".",
        help="Root directory to scan (default: current directory)",
    )
    return parser.parse_args(argv)


def main(argv=None) -> None:
    """Entry point for command-line use."""
    args = parse_args(argv)
    setup_logging()
    funcs = collect_functions(args.root)
    print_functions_list(funcs)


if __name__ == '__main__':
    main()
