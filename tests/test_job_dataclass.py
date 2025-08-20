from cam_slicer.machines.machine_manager import Job


def test_job_defaults():
    """Job dataclass initializes with pending status."""
    toolpath = [(0.0, 0.0, 0.0)]
    job = Job(id="j1", toolpath=toolpath)
    assert job.id == "j1"
    assert job.toolpath == toolpath
    assert job.status == "pending"
