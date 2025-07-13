import argparse
import logging
from pathlib import Path

from cam_slicer.logging_config import setup_logging
from cam_slicer.core.header_footer import ControllerConfig
from cam_slicer.utils.batch_processor import batch_process_toolpaths, run_macro_sequence
from cam_slicer.visualizer.preview import parse_gcode

setup_logging()
logger = logging.getLogger(__name__)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Batch process toolpaths")
    parser.add_argument("files", nargs="+", help="Toolpath files (G-code)")
    parser.add_argument("--controller", default="grbl", help="Controller type")
    parser.add_argument("--start", help="Macro at start")
    parser.add_argument("--between", help="Macro between jobs")
    parser.add_argument("--end", help="Macro at end")
    args = parser.parse_args(argv)

    cfg = ControllerConfig(CONTROLLER_TYPE=args.controller)
    toolpaths = []
    for f in args.files:
        lines = Path(f).read_text().splitlines()
        toolpaths.append(parse_gcode(lines))
    gcode = batch_process_toolpaths(
        toolpaths,
        cfg,
        start_macro=args.start,
        between_macro=args.between,
        end_macro=args.end,
    )
    for line in gcode:
        print(line)


if __name__ == "__main__":  # pragma: no cover
    main()
