import os, sys
import logging
from pathlib import Path

sys.path.insert(0, os.path.abspath('.'))
from cam_slicer.logging_config import setup_logging, setup_night_logging, export_critical_logs


def test_export_critical_logs(tmp_path):
    """Export warnings and errors from log file."""
    log_file = tmp_path / 'central.log'
    for h in logging.getLogger().handlers:
        logging.getLogger().removeHandler(h)
    setup_logging(level=logging.INFO, log_file=log_file)
    logger = logging.getLogger('test')
    logger.warning('warning message')
    logger.error('error message')
    logger.info('info message')
    report = export_critical_logs(log_file)
    assert 'warning message' in report and 'error message' in report
    assert 'info message' not in report


def test_setup_night_logging(tmp_path):
    """Night log rotates at midnight and file is created."""
    log_file = tmp_path / 'night.log'
    setup_night_logging(level=logging.INFO, log_file=log_file)
    logging.getLogger('night').info('test entry')
    assert log_file.exists()
