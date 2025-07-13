"""Central logging utilities for the entire CAM Slicer project."""

from __future__ import annotations

import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Optional

try:  # requests is optional for Slack/email export
    import requests  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    requests = None
from email.message import EmailMessage
import smtplib

LOG_DIR = Path("logs")
LOG_DIR.mkdir(exist_ok=True)

# default log path
LOG_PATH = LOG_DIR / "central.log"


def setup_logging(
    level: int = logging.INFO,
    log_file: Path | str = LOG_PATH,
    max_bytes: int = 1_000_000,
    backup_count: int = 3,
) -> None:
    """Configure rotating logging for the entire application.

    Parameters
    ----------
    level : int, optional
        Minimum log level. Defaults to ``logging.INFO``.
    log_file : str or Path, optional
        Destination log file. Defaults to ``logs/central.log``.
    max_bytes : int, optional
        Size threshold before rotation. Defaults to 1 MB.
    backup_count : int, optional
        Number of rotated files to keep. Defaults to 3.
    """
    log_path = Path(log_file)
    log_path.touch(exist_ok=True)

    handler = RotatingFileHandler(
        log_path, maxBytes=max_bytes, backupCount=backup_count
    )
    formatter = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    handler.setFormatter(formatter)

    console = logging.StreamHandler()
    console.setFormatter(formatter)

    logging.basicConfig(
        level=level,
        handlers=[handler, console],
        force=True,
    )


def export_critical_logs(
    log_file: Path | str = LOG_PATH,
    level: int = logging.WARNING,
    email: Optional[str] = None,
    slack_webhook: Optional[str] = None,
) -> str:
    """Return and optionally send warnings and errors from log file.

    Parameters
    ----------
    log_file : str or Path, optional
        Path to the log file to parse.
    level : int, optional
        Minimal level to export. Defaults to ``logging.WARNING``.
    email : str, optional
        Address to send the report via SMTP. Uses localhost mail server.
    slack_webhook : str, optional
        Slack webhook URL for posting the report.

    Returns
    -------
    str
        Combined log lines matching the given level.
    """

    threshold_names = [
        name
        for lvl, name in logging._levelToName.items()
        if isinstance(lvl, int) and lvl >= level
    ]
    report_lines: list[str] = []
    path = Path(log_file)
    if path.exists():
        for line in path.read_text().splitlines():
            if any(name in line for name in threshold_names):
                report_lines.append(line)

    report = "\n".join(report_lines)

    if email and report:
        msg = EmailMessage()
        msg["Subject"] = "CAM Slicer critical logs"
        msg["From"] = "noreply@example.com"
        msg["To"] = email
        msg.set_content(report)
        try:
            with smtplib.SMTP("localhost") as s:
                s.send_message(msg)
        except Exception as exc:  # pragma: no cover - network errors
            logging.error("Failed to send email: %s", exc)

    if slack_webhook and report:
        if requests is None:  # pragma: no cover - optional dependency missing
            logging.error("Cannot send Slack notification, requests not instal" "led")
        else:
            try:
                requests.post(slack_webhook, json={"text": report}, timeout=5)
            except Exception as exc:  # pragma: no cover - network errors
                logging.error("Failed to send Slack notification: %s", exc)

    return report


def setup_night_logging(level: int = logging.INFO, log_file: Path | str = LOG_DIR / "night.log") -> None:
    """Configure logging rotated at midnight.

    Parameters
    ----------
    level : int, optional
        Minimum log level.
    log_file : Path or str, optional
        Destination log file. Defaults to ``logs/night.log``.
    """
    log_path = Path(log_file)
    log_path.touch(exist_ok=True)

    from logging.handlers import TimedRotatingFileHandler

    handler = TimedRotatingFileHandler(str(log_path), when="midnight", backupCount=7)
    formatter = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    handler.setFormatter(formatter)

    console = logging.StreamHandler()
    console.setFormatter(formatter)

    logging.basicConfig(level=level, handlers=[handler, console], force=True)

