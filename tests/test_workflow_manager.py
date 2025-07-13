from cam_slicer import workflow_manager


def test_save_and_load_workflow(tmp_path, monkeypatch):
    """Save workflow JSON and load it back."""
    monkeypatch.setattr(workflow_manager, "WORKFLOW_DIR", tmp_path)
    data = {"toolpaths": [[0, 0, 0]], "material": "plywood"}
    path = workflow_manager.save_workflow(data, "test_wf")
    loaded = workflow_manager.load_workflow(path)
    assert loaded == data
