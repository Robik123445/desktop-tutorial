import importlib
import pytest

MODULES = [
    "cam_slicer",
    "cam_slicer.api_server",
    "cam_slicer.core.gcode_export",
    "cam_slicer.utils.zmap",
    "cam_slicer.digital_twin",
    "cam_slicer.machines.machine_manager",
    "cam_slicer.tool_database",
    "cam_slicer.user_manager",
]


def test_import_all_modules():
    mods = MODULES.copy()
    try:
        import fastapi  # noqa: F401
    except ImportError:
        mods.remove("cam_slicer.api_server")

    for name in mods:
        try:
            mod = importlib.import_module(name)
        except ImportError:
            if name == "cam_slicer.api_server":
                pytest.skip("API server optional")
            raise
        assert mod is not None


def test_instantiate_classes():
    pytest.importorskip("fastapi")
    try:
        from cam_slicer import create_app
    except ImportError:
        pytest.skip("API server optional")
    from cam_slicer.digital_twin import DigitalTwin, WorkshopTwin
    from cam_slicer.machines.machine_manager import Machine, MachineManager, Job
    from cam_slicer.tool_database import ToolDatabase
    from cam_slicer.user_manager import UserManager

    DigitalTwin(None)
    WorkshopTwin()
    Machine("m", "COM1")
    MachineManager()
    Job("path.gcode")
    ToolDatabase()
    UserManager()

    app = create_app()
    assert hasattr(app, "router")


def test_plugin_functions_exposed():
    """Plugin helpers are available from the package root."""
    from cam_slicer import (
        load_plugins,
        get_plugin,
        get_all_plugins,
        reload_plugins,
        execute_plugin,
    )

    assert callable(load_plugins)
    assert callable(get_plugin)
    assert callable(get_all_plugins)
    assert callable(reload_plugins)
    assert callable(execute_plugin)
