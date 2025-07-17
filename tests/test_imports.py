import importlib
import pytest

MODULES = [
    'cam_slicer',
    'cam_slicer.api_server',
    'cam_slicer.core.gcode_export',
    'cam_slicer.utils.zmap',
    'cam_slicer.digital_twin',
    'cam_slicer.machines.machine_manager',
    'cam_slicer.tool_database',
    'cam_slicer.user_manager',
]

def test_import_all_modules():
    mods = MODULES.copy()
    try:
        import fastapi  # noqa: F401
    except Exception:
        mods.remove('cam_slicer.api_server')
    for name in mods:
        mod = importlib.import_module(name)
        assert mod is not None

def test_instantiate_classes():
    pytest.importorskip("fastapi")
    from cam_slicer.digital_twin import DigitalTwin, WorkshopTwin
    from cam_slicer.machines.machine_manager import Machine, MachineManager, Job
    from cam_slicer.tool_database import ToolDatabase
    from cam_slicer.user_manager import UserManager
    from cam_slicer import create_app

    DigitalTwin(None)
    WorkshopTwin()
    Machine('m', 'COM1')
    MachineManager()
    Job('path.gcode')
    ToolDatabase()
    UserManager()
    if create_app is not None:
        app = create_app()
        assert hasattr(app, 'router')
