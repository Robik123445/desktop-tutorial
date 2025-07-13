from cam_slicer.tool_material_db import ToolMaterialDB


def test_add_and_get(tmp_path):
    """Add tool/material and retrieve parameters."""
    db = ToolMaterialDB()
    db.add_tool("endmill", 3.0, "flat", 12000)
    db.add_material("plywood")
    db.set_params("endmill", "plywood", 8000, 500.0)
    param = db.get_params("endmill", "plywood")
    assert param and param.rpm == 8000


def test_save_load(tmp_path):
    """Save database to JSON and load again."""
    path = tmp_path / "lib.json"
    db = ToolMaterialDB()
    db.add_tool("vbit", 1.0, "engraver", 15000)
    db.add_material("mdf")
    db.set_params("vbit", "mdf", 10000, 300.0)
    db.save(path)

    new_db = ToolMaterialDB()
    new_db.load(path)
    assert new_db.get_params("vbit", "mdf")
