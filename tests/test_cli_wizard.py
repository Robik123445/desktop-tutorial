import os, sys
sys.path.insert(0, os.path.abspath('.'))

from click.testing import CliRunner
from cam_slicer.cli_wizard import cli


def test_wizard_basic(tmp_path):
    """Test non-interactive wizard run."""
    runner = CliRunner()
    proj = tmp_path / 'proj'
    result = runner.invoke(cli, ['--lang', 'en', '--project', str(proj), '--profile', 'MACHINE_GRBL_SMALL'], input='\n')
    assert result.exit_code == 0
    assert 'Setup complete' in result.output
    assert proj.exists()
