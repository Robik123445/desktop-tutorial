import os
import sys

sys.path.insert(0, os.path.abspath("."))

from cam_slicer.ai.analyzers import (
    feedrate_advisor,
    trajectory_cleaner,
    ml_speed_optimizer,
    plugin_optimizer,
)
from cam_slicer.tool_material_db import ToolMaterialDB


def test_feedrate_advisor_with_db(tmp_path):
    db = ToolMaterialDB()
    db.add_tool("T1", 3.0, "flat", 10000)
    db.add_material("Aluminum")
    db.set_params("T1", "Aluminum", rpm=8000, feedrate=500)
    tp = [(0, 0, 0), (1, 0, 0)]
    res = feedrate_advisor(tp, tool="T1", material="Aluminum", db=db)
    assert res["recommended_feedrate"] == 500
    assert res["rpm"] == 8000


def test_trajectory_cleaner_zigzag():
    tp = [
        (0, 0, 0),
        (0.1, 0, 0),
        (0, 0, 0),
        (1, 0, 0),
        (1, 0.05, 0),
        (1, 0, 0),
        (2, 0, 0),
    ]
    out = trajectory_cleaner(tp, distance_threshold=0.1, angle_threshold=160)
    assert out["points"] == [(0, 0, 0), (1, 0, 0), (2, 0, 0)]
    assert out["removed"] >= 3


def test_ml_speed_optimizer():
    tp = [(0, 0, 0), (1, 2, 0), (2, -2, 0), (3, 0, 0)]
    out = ml_speed_optimizer(tp)["points"]
    assert abs(out[1][1]) < 2 and abs(out[2][1]) < 2


def test_plugin_optimizer_reverse():
    from cam_slicer.plugin_manager import load_plugins

    load_plugins()
    tp = [(0, 0, 0), (1, 0, 0)]
    out = plugin_optimizer(tp, "reverse_path")
    assert out["points"][0] == (1, 0, 0)
