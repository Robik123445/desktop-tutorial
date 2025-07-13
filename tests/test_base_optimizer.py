from cam_slicer.robotics.base_optimizer import (
    optimize_base_position,
    optimize_base_position_ai,
    plan_base_zones,
    plan_workspace_zones,
)
from cam_slicer.robotics.interface import ArmKinematicProfile


def test_optimize_base_position_simple():
    """Finds valid base for entire path."""
    profile = ArmKinematicProfile(
        name="planar",
        link_lengths=[1, 1],
        joint_types=["revolute", "revolute"],
        joint_limits=[(-180, 180), (-180, 180)],
        workspace=[(0, 1), (0, 1), (0, 1)],
    )
    toolpath = [(0.2, 0.2, 0), (0.8, 0.2, 0)]
    base = optimize_base_position(toolpath, profile, (0, 1), (0, 0), step=1)
    assert base == (0, 0)


def test_plan_base_zones_split():
    """Splits path into two zones when base cannot reach all points."""
    profile = ArmKinematicProfile(
        name="planar",
        link_lengths=[1, 1],
        joint_types=["revolute", "revolute"],
        joint_limits=[(-180, 180), (-180, 180)],
        workspace=[(0, 1), (0, 1), (0, 1)],
    )
    toolpath = [(0.2, 0.2, 0), (1.2, 0.2, 0)]
    zones = plan_base_zones(toolpath, profile, (0, 1), (0, 0), step=1)
    assert len(zones) == 2
    assert zones[0].toolpath == [toolpath[0]]
    assert zones[1].toolpath == [toolpath[1]]


def test_plan_workspace_zones_order_and_moves():
    """Zones are ordered by nearest base and include move commands."""
    profile = ArmKinematicProfile(
        name="planar",
        link_lengths=[1, 1],
        joint_types=["revolute", "revolute"],
        joint_limits=[(-180, 180), (-180, 180)],
        workspace=[(0, 1), (0, 1), (0, 1)],
    )
    toolpath = [
        (0.2, 0.2, 0),
        (1.2, 0.2, 0),
        (0.2, 0.8, 0),
    ]
    zones = plan_workspace_zones(toolpath, profile, (0, 1.5), (0, 1), step=1, safe_z=1)
    assert len(zones) >= 2
    assert zones[0].move_cmds
    assert zones[1].move_cmds[0].startswith("G1 Z")
    assert zones[0].warnings == []


def test_plan_workspace_zones_warning_on_obstacle():
    """Warn when base move crosses a forbidden zone."""
    profile = ArmKinematicProfile(
        name="planar",
        link_lengths=[1, 1],
        joint_types=["revolute", "revolute"],
        joint_limits=[(-180, 180), (-180, 180)],
        workspace=[(0, 2), (0, 2), (0, 1)],
    )
    toolpath = [(0.2, 0.2, 0), (2.5, 0.2, 0)]
    from cam_slicer.utils.geofence import GeoFence

    fence = GeoFence(forbidden_zones=[(0.9, 0, 1.1, 1)])
    zones = plan_workspace_zones(
        toolpath,
        profile,
        (0, 2),
        (0, 0),
        step=0.5,
        safe_z=1,
        obstacles=fence,
    )
    assert any(z.warnings for z in zones)


def test_optimize_base_position_ai_obstacle():
    """Avoids base inside forbidden zone when optimizing."""
    profile = ArmKinematicProfile(
        name="planar",
        link_lengths=[1, 1],
        joint_types=["revolute", "revolute"],
        joint_limits=[(-180, 180), (-180, 180)],
        workspace=[(0, 2), (0, 1), (0, 1)],
    )
    toolpath = [(0.5, 0.5, 0), (1.5, 0.5, 0)]
    from cam_slicer.utils.geofence import GeoFence

    fence = GeoFence(forbidden_zones=[(0, 0, 1, 1)])
    base = optimize_base_position_ai(
        toolpath,
        profile,
        (0, 1.5),
        (0, 0),
        step=0.5,
        obstacles=fence,
    )
    assert base != (0, 0)


def test_plan_workspace_zones_inserts_spindle_stop():
    """Spindle stop command added between zones."""
    profile = ArmKinematicProfile(
        name="p",
        link_lengths=[1, 1],
        joint_types=["revolute", "revolute"],
        joint_limits=[(-180, 180), (-180, 180)],
        workspace=[(0, 1), (0, 1), (0, 1)],
    )
    toolpath = [(0.2, 0.2, 0), (1.2, 0.2, 0)]
    zones = plan_workspace_zones(toolpath, profile, (0, 1.5), (0, 1), step=1)
    cmds = [cmd for z in zones for cmd in z.move_cmds]
    assert any(cmd == "M5" for cmd in cmds)

