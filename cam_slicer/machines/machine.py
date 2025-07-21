class Machine:
    """Placeholder representation of a machine."""

    def __init__(self, name: str, port: str, baud: int = 115200, ros_bridge=None, machine_type: str = "", heads=None) -> None:
        self.name = name
        self.port = port
        self.baud = baud
        self.ros_bridge = ros_bridge
        self.machine_type = machine_type
        self.heads = heads or []
        self.jobs = []
        self.active_job = None

    def add_job(self, path: str) -> None:
        """Queue a job for this machine."""
        self.jobs.append(path)

    def start_next(self) -> None:
        if self.jobs:
            self.active_job = self.jobs.pop(0)
            from cam_slicer.sender.serial_streamer import stream_gcode_to_grbl
            stream_gcode_to_grbl(self.active_job, self.port, self.baud)

    def join(self) -> None:
        pass


class Job:
    """Placeholder representation of a job."""

    def __init__(self, path: str, machine_type: str = "", head: str = "", priority: int = 0) -> None:
        self.path = path
        self.machine_type = machine_type
        self.head = head
        self.priority = priority
        self.state = "queued"

    def run(self) -> None:
        self.state = "running"

    def complete(self) -> None:
        self.state = "done"


class MachineManager:
    """Placeholder manager handling multiple machines."""

    def __init__(self) -> None:
        self.machines = {}

    def add_machine(self, machine: Machine) -> None:
        self.machines[machine.name] = machine

    def remove_machine(self, name: str) -> None:
        self.machines.pop(name, None)

    def start_all(self) -> None:
        for machine in self.machines.values():
            machine.start_next()

    def start_all_parallel(self) -> None:
        for machine in self.machines.values():
            machine.start_next()
        for machine in self.machines.values():
            machine.join()

    def start_serial(self) -> None:
        for machine in self.machines.values():
            machine.start_next()
            machine.join()

    def assign_jobs_optimized(self, jobs: list[str], estimator) -> None:
        """Distribute jobs to machines based on estimated time."""
        totals = {m.name: 0 for m in self.machines.values()}
        machines = list(self.machines.values())
        for job in jobs:
            target = min(machines, key=lambda m: totals[m.name])
            target.add_job(job)
            totals[target.name] += estimator(job)
