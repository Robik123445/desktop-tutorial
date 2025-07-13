from cam_slicer.utils.system_report import generate_system_report


def test_generate_system_report(tmp_path):
    """Test generation of a system status report."""
    repo_root = tmp_path
    # create minimal structure
    (repo_root / "cam_slicer/core").mkdir(parents=True)
    (repo_root / "tests").mkdir()
    # create dummy engine.py with docstring
    (repo_root / "cam_slicer/core/engine.py").write_text("""def toolpath_to_gcode():\n    \"\"\"doc\"\"\"\n\n""")
    (repo_root / "README.md").write_text("readme")
    report = generate_system_report(str(repo_root))
    assert report.startswith("# System Report")
    assert "## cam_slicer.core" in report
    assert "`toolpath_to_gcode`" in report
    assert "## Summary" in report
    assert "modules without tests" in report
    assert (repo_root / "report.txt").exists()
