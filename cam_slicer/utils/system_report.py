import ast
import logging
from pathlib import Path
from typing import Dict, List

from cam_slicer.logging_config import setup_logging

setup_logging()  # Configure shared logger


MODULES: Dict[str, Dict[str, List[str] | str]] = {
    "core": {"file": "engine.py", "functions": ["toolpath_to_gcode", "process_toolpath"]},
    "modules": {"file": "modules.py", "functions": []},
    "visualizer": {"file": "preview.py", "functions": ["preview_gcode", "render_surface_and_toolpath"]},
    "sensors": {"file": "probe_generator.py", "functions": ["generate_heightmap"]},
    "motion": {"file": "kinematics.py", "functions": ["apply_pressure_advance", "transform_point"]},
    "config": {"file": "machine_config.py", "functions": ["get_machine_config"]},
}


def _has_tests(project_root: Path, module_file: str) -> bool:
    """Return ``True`` if a test file exists for ``module_file``."""

    pattern = f"test_{Path(module_file).stem}.py"
    test_dir = project_root / "tests"
    return any(p.name == pattern for p in test_dir.glob("test_*.py"))


def _has_readme(project_root: Path) -> bool:
    """Return ``True`` if the project root contains ``README.md``."""

    return (project_root / "README.md").exists()


def _extract_functions(path: Path) -> Dict[str, bool]:
    """Return mapping of function names to docstring presence.

    Parameters
    ----------
    path : Path
        File to inspect.

    Returns
    -------
    dict
        Keys are function names, values ``True`` if a docstring exists.
    """
    if not path.exists():
        return {}
    try:
        source = path.read_text(encoding="utf-8")
        tree = ast.parse(source)
    except Exception as exc:  # pragma: no cover - parse errors
        logging.warning("Failed to parse %s: %s", path, exc)
        return {}

    return {
        node.name: ast.get_docstring(node) is not None
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
    }


def generate_system_report(project_root: str, markdown: bool = True) -> str:
    """Scan project directories and produce a summary report.

    Parameters
    ----------
    project_root : str
        Path to the project root.
    markdown : bool, optional
        When ``True`` the result is formatted as Markdown.

    Returns
    -------
    str
        The generated report text.

    Examples
    --------
    >>> generate_system_report('.', markdown=False)  # doctest: +SKIP
    'Module: cam_slicer.core\n...'
    """

    root = Path(project_root)
    lines: List[str] = ["# System Report"] if markdown else []
    missing_docs = 0
    missing_tests = 0

    for mod, info in MODULES.items():
        section_header = f"## cam_slicer.{mod}" if markdown else f"Module: cam_slicer.{mod}"
        lines.append(section_header)

        mod_dir = root / "cam_slicer" / mod
        if not mod_dir.exists():
            lines.append("* â Missing directory" if markdown else "  - MISSING directory")
            continue

        main_file = mod_dir / info["file"]
        if not main_file.exists():
            msg = f"* â Missing file `{info['file']}`" if markdown else f"  - MISSING main file: {info['file']}"
            lines.append(msg)
            continue

        funcs = _extract_functions(main_file)
        for fname in info["functions"]:
            if fname in funcs:
                if funcs[fname]:
                    msg = f"* `{fname}` â â docstring" if markdown else f"  - {fname}: OK"
                else:
                    msg = f"* `{fname}` â â missing docstring" if markdown else f"  - {fname}: missing docstring"
                    missing_docs += 1
            else:
                msg = f"* `{fname}` â â missing" if markdown else f"  - {fname}: MISSING"
                missing_docs += 1
            lines.append(msg)

        test_exists = _has_tests(root, info["file"])
        if not test_exists:
            missing_tests += 1
        if markdown:
            test_msg = "* tests: â" if test_exists else "* tests: â"
        else:
            test_msg = f"  - tests: {'yes' if test_exists else 'no'}"
        lines.append(test_msg)
        lines.append("")

    readme_msg = "README.md present" if _has_readme(root) else "README.md MISSING"
    lines.append(readme_msg)

    summary_header = "## Summary" if markdown else "Summary"
    lines.append(summary_header)
    lines.append(
        f"* modules without tests: {missing_tests}" if markdown else f"modules without tests: {missing_tests}"
    )
    lines.append(
        f"* functions lacking docs: {missing_docs}" if markdown else f"functions lacking docs: {missing_docs}"
    )

    report = "\n".join(lines)
    (root / "report.txt").write_text(report, encoding="utf-8")
    logging.info("System report generated at %s", root / 'report.txt')
    return report
