"""CLI utilities for managing multiple CNC machines."""

import argparse
import logging

from cam_slicer.logging_config import setup_logging
from cam_slicer.machines import MachineManager, Machine

setup_logging()


CONFIG_PATH = "machines.json"


def load_manager() -> MachineManager:
    try:
        return MachineManager.load_config(CONFIG_PATH)
    except FileNotFoundError:
        return MachineManager()


def save_manager(manager: MachineManager) -> None:
    manager.save_config(CONFIG_PATH)


def list_machines(manager: MachineManager) -> None:
    for m in manager.machines.values():
        print(f"{m.name}: {m.port} @ {m.baud}")


def add_machine(manager: MachineManager, name: str, port: str, baud: int) -> None:
    manager.add_machine(Machine(name, port, baud))
    save_manager(manager)


def start_machine(manager: MachineManager, name: str, gcode: str) -> None:
    m = manager.machines.get(name)
    if not m:
        print(f"Machine {name} not found")
        return
    m.add_job(gcode)
    m.start_next()
    m.join()


def start_all(manager: MachineManager, gcode: str) -> None:
    for m in manager.machines.values():
        m.add_job(gcode)
    manager.start_all_parallel()


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Manage multiple CNC machines")
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", help="List machines")
    add_p = sub.add_parser("add", help="Add machine")
    add_p.add_argument("name")
    add_p.add_argument("port")
    add_p.add_argument("--baud", type=int, default=115200)
    start_p = sub.add_parser("start", help="Start job on machine")
    start_p.add_argument("name")
    start_p.add_argument("gcode")
    all_p = sub.add_parser("start-all", help="Start same job on all machines")
    all_p.add_argument("gcode")
    args = parser.parse_args(argv)

    manager = load_manager()

    if args.cmd == "list":
        list_machines(manager)
    elif args.cmd == "add":
        add_machine(manager, args.name, args.port, args.baud)
    elif args.cmd == "start":
        start_machine(manager, args.name, args.gcode)
    elif args.cmd == "start-all":
        start_all(manager, args.gcode)


if __name__ == "__main__":  # pragma: no cover - CLI entry
    main()
