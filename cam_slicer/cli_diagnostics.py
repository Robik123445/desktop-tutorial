import argparse
from pathlib import Path
import shutil
import logging

from cam_slicer.logging_config import setup_logging, export_critical_logs
from cam_slicer.utils.system_report import generate_system_report

setup_logging()

LOG_FILE = Path('logs') / 'central.log'


def export_logs(destination: str) -> None:
    """Copy central log file to ``destination``."""
    src = LOG_FILE
    dst = Path(destination)
    dst.write_text(src.read_text() if src.exists() else '')
    print(f'Logs exported to {dst}')


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description='Diagnostics utilities (docs: https://example.github.io/cam_slicer/)',
        epilog='Full documentation: https://example.github.io/cam_slicer/'
    )
    sub = parser.add_subparsers(dest='cmd', required=True)
    exp = sub.add_parser('export-logs', help='Export full log file')
    exp.add_argument('path', nargs='?', default='central.log', help='output file')

    crit = sub.add_parser('export-critical', help='Export warnings and errors')
    crit.add_argument('path', nargs='?', default='critical.log', help='output file')
    crit.add_argument('--level', default='WARNING', help='minimum level')
    crit.add_argument('--email', help='send report via email')
    crit.add_argument('--slack', help='Slack webhook URL')

    rep = sub.add_parser('system-report', help='Generate system report')
    rep.add_argument('path', nargs='?', default='report.txt', help='output file')
    args = parser.parse_args(argv)

    if args.cmd == 'export-logs':
        export_logs(args.path)
    elif args.cmd == 'export-critical':
        level = getattr(logging, args.level.upper(), logging.WARNING)
        report = export_critical_logs(
            log_file=LOG_FILE, level=level, email=args.email, slack_webhook=args.slack
        )
        Path(args.path).write_text(report, encoding='utf-8')
        print(f'Critical logs saved to {args.path}')
    elif args.cmd == 'system-report':
        text = generate_system_report('.', markdown=False)
        Path(args.path).write_text(text, encoding='utf-8')
        print(f'System report saved to {args.path}')


if __name__ == '__main__':
    main()
