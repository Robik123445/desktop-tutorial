import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List
from cam_slicer.machines.machine import Machine

try:
    from cam_slicer.ros.ros_bridge import ROSBridge as _ROSBridge
except ImportError:
    _ROSBridge = None

logger = logging.getLogger(__name__)


@dataclass
class Job:
    """Simple job representation with status tracking."""

    id: str
    toolpath: list[tuple[float, float, float]]
    status: str = "pending"


class MachineManager:
    def __init__(self) -> None:
        self.machines: Dict[str, Machine] = {}

    def add_machine(self, machine: Machine) -> None:
        self.machines[machine.name] = machine
        logger.info("Added machine %s", machine.name)

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
            try:
                m.start_next()
            except Exception as exc:
                logger.error("Streaming failed on %s: %s", m.name, exc)
            m.join()

    def assign_jobs_optimized(self, jobs: List[str], estimator) -> None:
        """Distribute jobs among machines to balance estimated time."""
        totals = {name: 0.0 for name in self.machines}
        for job in jobs:
            target = min(self.machines.values(), key=lambda m: totals[m.name])
            target.add_job(job)
            totals[target.name] += estimator(job)

    @staticmethod
    def load_config(path: str) -> "MachineManager":
        """Create manager from JSON configuration."""
        manager = MachineManager()
        data = json.loads(Path(path).read_text())
        for item in data.get("machines", []):
            # Use loaded ROS bridge class if available
            rb = _ROSBridge() if _ROSBridge else None
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


__all__ = ["Job", "MachineManager"]
