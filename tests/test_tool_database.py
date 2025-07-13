from cam_slicer.tool_database import ToolDatabase


def test_add_list_remove(tmp_path):
    """Test adding, listing and removing tools."""
    db = ToolDatabase()
    db.add_tool('endmill', 3.0, 'flat', 12000)
    assert db.list_tools() == [{"name": 'endmill', "diameter": 3.0, "type": 'flat', "max_rpm": 12000}]
    assert db.remove_tool('endmill')
    assert db.list_tools() == []
    assert not db.remove_tool('missing')


def test_save_load(tmp_path):
    """Test saving and loading tools from JSON."""
    path = tmp_path / 'tools.json'
    db = ToolDatabase()
    db.add_tool('ball', 2.0, 'ball', 15000)
    db.save(path)

    new_db = ToolDatabase()
    new_db.load(path)
    assert new_db.list_tools() == db.list_tools()
