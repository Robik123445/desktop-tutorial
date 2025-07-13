import os, sys
sys.path.insert(0, os.path.abspath('.'))
from cam_slicer.cli_diagnostics import main
from pathlib import Path


def test_export_logs(tmp_path, capsys):
    """Export log file to given path."""
    log_file = Path('logs/central.log')
    log_file.parent.mkdir(exist_ok=True)
    log_file.write_text('demo log')
    out_path = tmp_path / 'out.log'
    main(['export-logs', str(out_path)])
    assert out_path.read_text() == 'demo log'


def test_system_report(tmp_path):
    """Generate system report via CLI."""
    out = tmp_path / 'report.txt'
    main(['system-report', str(out)])
    assert out.exists()
    assert 'Module:' in out.read_text()


def test_export_critical(tmp_path):
    """Export critical log lines to file."""
    log_file = Path('logs/central.log')
    log_file.parent.mkdir(exist_ok=True)
    log_file.write_text('INFO ok\nWARNING bad\nERROR worse')
    out = tmp_path / 'crit.log'
    main(['export-critical', str(out), '--level', 'WARNING'])
    text = out.read_text()
    assert 'WARNING bad' in text and 'ERROR worse' in text
