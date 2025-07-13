from __future__ import annotations

import json
import logging
import threading
from pathlib import Path
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

from cam_slicer.logging_config import setup_logging
from cam_slicer.sender.serial_streamer import stream_gcode_to_grbl
from cam_slicer.digital_twin import DigitalTwin

_ROSBridge = None  # loaded lazily

setup_logging()
logger = logging.getLogger(__name__)


class Machine:
    """Single machine with optional multi-head support."""

    def __init__(
        self,
        name: str,
        port: str,
        baud: int = 115200,
        connection: Optional[object] = None,
        ros_bridge: Optional[object] = None,
        machine_type: str = "cnc",
        heads: Optional[List[str]] = None,
    ) -> None:
        self.name = name
        self.port = port
        self.baud = baud
        self.machine_type = machine_type
        self.heads = heads or ["default"]
        self.active_head = self.heads[0]
        self.jobs: List[str] = []
        self.thread: Optional[threading.Thread] = None
        self.state = "idle"
        self.ros_bridge = ros_bridge
        self.twin: Optional[DigitalTwin] = (
            DigitalTwin(connection) if connection else None
        )

    def enable_ros(self) -> None:
        """Attach a ROS bridge if the dependency is available."""
        global _ROSBridge
        if self.ros_bridge:
            return
        if _ROSBridge is None:
            try:
                from cam_slicer.ros_bridge import ROSBridge as RB  # type: ignore
                _ROSBridge = RB
            except Exception as exc:  # pragma: no cover - log only
                logger.error("ROS unavailable: %s", exc)
                return
        try:
            self.ros_bridge = _ROSBridge()
        except Exception as exc:  # pragma: no cover - log only
            logger.error("Failed to init ROS bridge: %s", exc)

    def attach_twin(self, connection: object) -> None:
        """Assign a :class:`DigitalTwin` using given connection."""
        self.twin = DigitalTwin(connection)

    def change_head(self, head: str) -> bool:
        """Switch active head if available."""
        if head in self.heads:
            self.active_head = head
            logger.info("%s changed head to %s", self.name, head)
            return True
        logger.warning("%s missing head %s", self.name, head)
        return False

    def supports(self, machine_type: str, head: str | None = None) -> bool:
        """Return True if machine and head match requirements."""
        if self.machine_type != machine_type:
            return False
        if head and head not in self.heads:
            return False
        return True

    def add_job(self, gcode_path: str) -> None:
        """Queue a G-code file for execution."""
        self.jobs.append(gcode_path)
        logger.info("%s added job %s", self.name, gcode_path)

    def _run_job(self, gcode: str) -> None:
        logger.info("%s starting job %s", self.name, gcode)
        try:
            stream_gcode_to_grbl(gcode, self.port, self.baud)
            logger.info("%s finished job %s", self.name, gcode)
        except Exception as exc:
            logger.error("%s error: %s", self.name, exc)
        self.state = "idle"

    def start_next(self) -> None:
        """Start the next job in the queue in a background thread."""
        if self.state != "idle" or not self.jobs:
            return
        gcode = self.jobs.pop(0)
        self.state = "running"
        self.thread = threading.Thread(target=self._run_job, args=(gcode,), daemon=True)
        self.thread.start()

    def join(self) -> None:
        """Wait for the current job to finish."""
        if self.thread:
            self.thread.join()


@dataclass
class Job:
    """Job specification for job assignment."""

    path: str
    machine_type: str = "cnc"
    head: Optional[str] = None
    priority: int = 0


class MachineManager:
    """Manage multiple :class:`Machine` instances."""

    def __init__(self) -> None:
        self.machines: Dict[str, Machine] = {}

    def add_machine(self, machine: Machine) -> None:
        self.machines[machine.name] = machine
        logger.info("Added machine %s", machine.name)

    def assign_jobs(self, jobs: List[Job]) -> None:
        """Assign jobs by priority and capability."""
        jobs_sorted = sorted(jobs, key=lambda j: j.priority, reverse=True)
        for job in jobs_sorted:
            candidates = [m for m in self.machines.values() if m.supports(job.machine_type, job.head)]
            if not candidates:
                logger.error("No machine found for %s", job.path)
                continue
            target = min(candidates, key=lambda m: len(m.jobs))
            if job.head:
                target.change_head(job.head)
            target.add_job(job.path)
        logger.info("Assigned %d jobs", len(jobs))

    def assign_jobs_optimized(
        self, jobs: List[str], estimate_time: Callable[[str], float]
    ) -> None:
        """Distribute jobs across machines to minimize total time."""
        machine_time = {name: 0.0 for name in self.machines}
        for job in jobs:
            target = min(machine_time, key=machine_time.get)
            self.machines[target].add_job(job)
            machine_time[target] += estimate_time(job)
        logger.info(
            "Assigned %d jobs across %d machines", len(jobs), len(self.machines)
        )

    def remove_machine(self, name: str) -> None:
        if name in self.machines:
            del self.machines[name]
            logger.info("Removed machine %s", name)

    def start_all_parallel(self) -> None:
        """Start queued jobs on all machines simultaneously."""
        for m in self.machines.values():
            m.start_next()
        for m in self.machines.values():
            m.join()

    def start_serial(self) -> None:
        """Run queued jobs one machine after another."""
        for m in self.machines.values():
            m.start_next()
            m.join()

    @staticmethod
    def load_config(path: str) -> "MachineManager":
        """Create manager from JSON configuration."""
        manager = MachineManager()
        data = json.loads(Path(path).read_text())
        for item in data.get("machines", []):
            rb = ROSBridge() if ROSBridge else None
            machine = Machine(item["name"], item["port"], item.get("baud", 115200), rb)
            manager.add_machine(machine)
        return manager

    def save_config(self, path: str) -> None:
        """Save manager machine list to JSON."""
        data = {"machines": [
            {"name": m.name, "port": m.port, "baud": m.baud} for m in self.machines.values()
        ]}
        Path(path).write_text(json.dumps(data, indent=2))
        logger.info("Saved machine config to %s", path)
