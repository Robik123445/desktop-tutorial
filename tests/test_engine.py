import os, sys
sys.path.insert(0, os.path.abspath("."))

import pytest
import math

from cam_slicer.core.header_footer import _get_header_footer, ControllerConfig
from cam_slicer.core.gcode_export import toolpath_to_gcode
from cam_slicer.core.engine import process_toolpath
from cam_slicer.robotics.kinematics import (
    TransformConfig,
    transform_point,
    axis_limits,
    apply_pressure_advance,
    apply_input_shaping,
    calculate_junction_velocity,
    plan_feedrate_with_lookahead,
)


def test_get_header_footer_grbl():
    """Test GRBL controller header and footer generation."""
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    header, footer = _get_header_footer(cfg)
    assert header == "grbl-header"
    assert footer == "grbl-footer"


def test_get_header_footer_unknown():
    """Ensure unsupported controller types raise ValueError."""
    cfg = ControllerConfig(CONTROLLER_TYPE="unknown")
    with pytest.raises(ValueError):
        _get_header_footer(cfg)


def test_process_toolpath_climb():
    """Test reversing toolpath for climb milling."""
    toolpath = [(0, 0, -1), (1, 0, -1)]
    processed = process_toolpath(toolpath, cutting_strategy="climb")
    assert processed == list(reversed(toolpath))


def test_process_toolpath_adaptive():
    """Test adaptive strategy scaling of depths."""
    toolpath = [(0, 0, -1), (1, 0, -2)]
    processed = process_toolpath(toolpath, cutting_strategy="adaptive", radius=5)
    factor = max(0.1, min(1.0, 5 / 10))
    expected = [(0, 0, -1 * factor), (1, 0, -2 * factor)]
    assert processed == expected


def test_transform_point():
    """Test Cartesian rotation and offset of a point."""
    cfg = TransformConfig(rotation_deg=90, scale=1.0, offset=(1, 0, 0))
    x, y, z = transform_point(1, 0, 0, cfg)
    assert round(x, 3) == 1
    assert round(y, 3) == 1
    assert z == 0


def test_transform_point_polar():
    """Test polar to Cartesian conversion in transform_point."""
    cfg = TransformConfig(rotation_deg=0, scale=1.0, offset=(0, 0, 0))
    x, y, z = transform_point(10, 90, -2, cfg, mode="polar")
    assert round(x, 3) == 0
    assert round(y, 3) == 10
    assert z == -2


def test_toolpath_to_gcode_transformation():
    """Test basic toolpath transformation into G-code."""
    toolpath = [(1, 0, -1)]
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    transform = TransformConfig(rotation_deg=90, scale=1.0, offset=(0, 0, 0))
    gcode = toolpath_to_gcode(toolpath, cfg, transform)
    assert gcode[0] == "grbl-header"
    assert "X0.000 Y1.000 Z-1.000" in gcode[1]
    assert gcode[-1] == "grbl-footer"


def test_toolpath_to_gcode_g0():
    """Test generating rapid moves with transformation."""
    toolpath = [(0, 0, -1)]
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    transform = TransformConfig(scale=2.0, offset=(1, 0, 0))
    gcode = toolpath_to_gcode(toolpath, cfg, transform, move_command="G0")
    assert gcode[1].startswith("G0")
    assert "X1.000" in gcode[1]


def test_toolpath_to_gcode_arc():
    """Test arc command generation for circular paths."""
    circle_path = [(1, 0, -1), (0, 0, -1), (-1, 0, -1)]
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    transform = TransformConfig()
    gcode = toolpath_to_gcode(circle_path, cfg, transform, shape="circle", arc_support=True)
    assert any(line.startswith("G2") or line.startswith("G3") for line in gcode)


def test_toolpath_to_gcode_offsets():
    """Test applying work and tool offsets in G-code."""
    toolpath = [(0, 0, -1)]
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    transform = TransformConfig(work_offset=(1, 2, 0), tool_offset=(0, 0, -0.5))
    gcode = toolpath_to_gcode(toolpath, cfg, transform)
    assert "X1.000 Y2.000 Z-1.500" in gcode[1]


def test_apply_pressure_advance():
    """Test pressure advance feedrate adjustment."""
    fr = 1000
    accel = 200
    factor = 0.05
    expected = fr + accel * factor
    assert apply_pressure_advance(fr, accel, factor) == expected


def test_toolpath_to_gcode_pressure_advance():
    """Test G-code output with pressure advance enabled."""
    toolpath = [(0, 0, -1), (1, 0, -1)]
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    transform = TransformConfig()
    fr = 1000
    gcode = toolpath_to_gcode(toolpath, cfg, transform, feedrate=fr, advance_factor=0.02)
    adj = apply_pressure_advance(fr, 500.0, 0.02)
    assert f"F{adj:.3f}" in gcode[1]


def test_apply_input_shaping_zv():
    """Test zero-vibration input shaping output."""
    points = [(0, 0, 0), (1, 0, 0), (0, 0, 0)]
    shaped = apply_input_shaping(points, frequency=1, damping=0.0)
    assert shaped == [(0.0, 0.0, 0.0), (0.5, 0.0, 0.0), (0.5, 0.0, 0.0)]


def test_calculate_junction_velocity():
    """Test junction velocity calculation for a corner."""
    v = calculate_junction_velocity((1, 0, 0), (0, 1, 0), 1000, 0.05)
    assert pytest.approx(v, rel=1e-3) == 10.9868411346


def test_calculate_junction_velocity_straight():
    """Test junction velocity with no angle change."""
    v = calculate_junction_velocity((1, 0, 0), (1, 0, 0), 1000, 0.05)
    assert math.isinf(v)


def test_plan_feedrate_with_lookahead():
    """Test feedrate planning with a lookahead queue."""
    path = [(0, 0, 0), (1, 0, 0), (1, 1, 0)]
    max_fr = 1000
    jv = calculate_junction_velocity((1, 0, 0), (0, 1, 0), max_fr, 0.05)
    speeds = plan_feedrate_with_lookahead(path, max_fr, max_fr, 0.05, queue_size=2)
    assert len(speeds) == len(path)
    assert pytest.approx(speeds[1], rel=1e-3) == jv


def test_transform_point_axis_limits():
    """Test velocity clamping to axis limits."""
    cfg = TransformConfig()
    prev = (0, 0, 0)
    prev_v = (0, 0, 0)
    pos, vel = transform_point(
        1000,
        0,
        0,
        cfg,
        prev_point=prev,
        prev_velocity=prev_v,
        dt=1.0,
        axis_cfg=axis_limits,
        return_velocity=True,
    )
    assert pos[0] == axis_limits["X"]["max_velocity"]


def test_toolpath_to_gcode_lookahead():
    """Test lookahead-based feedrate in G-code generation."""
    path = [(0, 0, 0), (1, 0, 0), (1, 1, 0)]
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    transform = TransformConfig()
    fr = 1000
    jv = calculate_junction_velocity((1, 0, 0), (0, 1, 0), 1000, 0.05)
    gcode = toolpath_to_gcode(
        path,
        cfg,
        transform,
        feedrate=fr,
        advance_factor=0.0,
        lookahead=2,
    )
    assert f"F{jv:.3f}" in gcode[1]


def test_get_machine_config():
    """Test retrieving predefined machine configuration."""
    from cam_slicer.config.machine_config import get_machine_config

    cfg = get_machine_config("MACHINE_GRBL_SMALL")
    assert cfg["controller"].lower() == "grbl"


def test_get_machine_config_unknown():
    """Test error raised for unknown machine preset."""
    from cam_slicer.config.machine_config import get_machine_config

    with pytest.raises(KeyError):
        get_machine_config("unknown")


def test_toolpath_to_gcode_zmap():
    """Test Z-map height compensation in G-code."""
    from cam_slicer.utils import ZMap

    zmap = ZMap(points=[(0, 0, 0.1), (1, 0, 0.2)])
    toolpath = [(0, 0, -1), (1, 0, -1)]
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    transform = TransformConfig()
    gcode = toolpath_to_gcode(toolpath, cfg, transform, z_map=zmap)
    assert "Z-0.900" in gcode[1]


def test_toolpath_to_gcode_adaptive_mode():
    """Adaptive mode scales feedrate based on z-map."""
    from cam_slicer.utils import ZMap

    zmap = ZMap(points=[(0, 0, 0.5), (1, 0, 0.0)])
    toolpath = [(0, 0, -1), (1, 0, -1)]
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    transform = TransformConfig()
    g_normal = toolpath_to_gcode(toolpath, cfg, transform, feedrate=1000, z_map=zmap)
    g_adapt = toolpath_to_gcode(
        toolpath, cfg, transform, feedrate=1000, z_map=zmap, adaptive_mode=True
    )
    assert g_adapt[1] != g_normal[1]


def test_toolpath_to_gcode_macros():
    """Test macro insertion at start, middle and end."""
    toolpath = [(0, 0, 0), (1, 0, 0)]
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    transform = TransformConfig()
    gcode = toolpath_to_gcode(
        toolpath,
        cfg,
        transform,
        start_macro="START_SPINDLE",
        mid_macro="PAUSE",
        end_macro="END_SPINDLE",
    )
    # start macro lines appear after header
    assert "M3 S1000" in gcode[1]
    # mid macro inserted between moves
    assert gcode[-3] == "M0"
    # end macro before footer
    assert gcode[-2] == "M5"


def test_toolpath_to_gcode_parallel():
    """Ensure parallel point transform produces same result."""
    toolpath = [(0,0,-1),(1,0,-1)] * 10
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    transform = TransformConfig(scale=1.0)
    g1 = toolpath_to_gcode(toolpath, cfg, transform)
    g2 = toolpath_to_gcode(toolpath, cfg, transform, parallel=True, workers=2)
    assert g1 == g2


def test_get_header_footer_multi_axis():
    """Fanuc controller headers should be returned."""
    cfg = ControllerConfig(CONTROLLER_TYPE="fanuc")
    header, footer = _get_header_footer(cfg)
    assert header == "fanuc-header"
    assert footer == "fanuc-footer"


def test_toolpath_to_gcode_multi_axis():
    """Extended G-code should include A/B/C axes."""
    tp = [(0, 0, 0, 10, 20, 30)]
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    transform = TransformConfig()
    gcode = toolpath_to_gcode(tp, cfg, transform)
    assert "A10.000" in gcode[1]
    assert "B20.000" in gcode[1]
    assert "C30.000" in gcode[1]
