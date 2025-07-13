from cam_slicer.machines.machine_manager import MachineManager, Machine, Job
from cam_slicer.user_manager import UserManager


def test_assign_jobs_with_head():
    """Jobs are assigned to machines supporting required heads."""
    mgr = MachineManager()
    cnc = Machine("cnc", "P1", machine_type="cnc", heads=["router", "laser"])
    laser = Machine("laser", "P2", machine_type="laser", heads=["laser"])
    mgr.add_machine(cnc)
    mgr.add_machine(laser)
    jobs = [Job(path="a.gcode", machine_type="laser", head="laser", priority=1),
            Job(path="b.gcode", machine_type="cnc", head="router", priority=0)]
    mgr.assign_jobs(jobs)
    assert laser.jobs == ["a.gcode"]
    assert cnc.jobs == ["b.gcode"]


def test_user_permissions():
    """Operators can run jobs but not remove machines."""
    um = UserManager()
    um.add_user("bob", "operator")
    assert um.check_permission("bob", "run_job")
    assert not um.check_permission("bob", "remove_machine")
