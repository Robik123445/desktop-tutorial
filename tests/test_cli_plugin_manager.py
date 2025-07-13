import os, sys
sys.path.insert(0, os.path.abspath('.'))
from cam_slicer.cli_plugin_manager import main


def test_cli_list(capsys):
    """Test listing available plugins via the CLI."""
    main(['list'])
    out = capsys.readouterr().out
    assert 'reverse_path' in out and 'adaptive_path' in out


def test_cli_run_with_args(capsys):
    """Test running a plugin with extra arguments."""
    # Pass additional arguments and expect them reversed by the plugin
    main(['run', 'reverse_path', 'A', 'B'])
    out = capsys.readouterr().out
    assert "['B', 'A']" in out
