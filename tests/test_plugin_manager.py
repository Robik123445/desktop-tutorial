from cam_slicer import plugin_manager
import logging


def test_plugin_manager_load(tmp_path):
    """Test loading a plugin from a custom directory."""
    plugin_dir = tmp_path / "plugins"
    plugin_dir.mkdir()
    (plugin_dir / "sample.py").write_text(
        """
from cam_slicer.plugin_manager import Plugin

def register():
    def apply(tp):
        return tp[::-1]
    return Plugin(name='sample', description='demo', apply=apply)
"""
    )
    plugin_manager.load_plugins(plugin_dir)
    plugin = plugin_manager.get_plugin("sample")
    assert plugin is not None
    assert plugin.apply([1, 2, 3]) == [3, 2, 1]
    all_plugins = {p["name"]: p["description"] for p in plugin_manager.get_all_plugins()}
    assert all_plugins == {"sample": "demo"}


def test_default_plugins_loaded():
    """Test built-in plugins load and execute correctly."""
    plugin_manager.load_plugins()
    plugins = {p["name"] for p in plugin_manager.get_all_plugins()}
    assert "reverse_path" in plugins
    plugin = plugin_manager.get_plugin("reverse_path")
    assert plugin.apply([1, 2, 3]) == [3, 2, 1]


def test_plugin_error_handling(tmp_path):
    """Test that faulty plugins are skipped without crashing."""
    plugin_dir = tmp_path / "plugins"
    plugin_dir.mkdir()
    (plugin_dir / "good.py").write_text(
        """
from cam_slicer.plugin_manager import Plugin

def register():
    def apply(tp):
        return tp
    return Plugin(name='good', description='ok', apply=apply)
"""
    )
    (plugin_dir / "bad.py").write_text(
        """
def register():
    raise RuntimeError('boom')
"""
    )
    plugin_manager.load_plugins(plugin_dir)
    plugins = {p["name"] for p in plugin_manager.get_all_plugins()}
    assert "good" in plugins
    assert "bad" not in plugins


def test_load_summary_logged(tmp_path, caplog):
    """Plugin loader logs count of loaded and failed plugins."""
    plugin_dir = tmp_path / "plugins"
    plugin_dir.mkdir()
    (plugin_dir / "ok.py").write_text(
        """
from cam_slicer.plugin_manager import Plugin

def register():
    def apply(tp):
        return tp
    return Plugin(name='ok', description='good', apply=apply)
"""
    )
    (plugin_dir / "broken.py").write_text(
        """
def register():
    raise RuntimeError('fail')
"""
    )
    caplog.set_level(logging.INFO)
    plugin_manager.load_plugins(plugin_dir)
    summary = "Plugin loading complete: 1 loaded, 1 failed"
    assert any(summary in rec.message for rec in caplog.records)


def test_reload_plugins(tmp_path):
    """Test plugin reload picks up new modules."""
    plugin_dir = tmp_path / "plugins"
    plugin_dir.mkdir()
    (plugin_dir / "one.py").write_text(
        """
from cam_slicer.plugin_manager import Plugin

def register():
    def apply(tp):
        return tp
    return Plugin(name='one', description='first', apply=apply)
"""
    )
    plugin_manager.load_plugins(plugin_dir)
    assert any(p["name"] == "one" for p in plugin_manager.get_all_plugins())

    (plugin_dir / "two.py").write_text(
        """
from cam_slicer.plugin_manager import Plugin

def register():
    def apply(tp):
        return tp
    return Plugin(name='two', description='second', apply=apply)
"""
    )
    plugin_manager.reload_plugins()
    plugins = {p["name"] for p in plugin_manager.get_all_plugins()}
    assert "one" in plugins and "two" in plugins


def test_sandbox_execution(tmp_path):
    """Plugins run in sandboxed process."""
    plugin_dir = tmp_path / "plugins"
    plugin_dir.mkdir()
    (plugin_dir / "s.py").write_text(
        """
from cam_slicer.plugin_manager import Plugin

def register():
    def apply(args):
        return list(reversed(args))
    return Plugin(name='s', description='demo', apply=apply)
"""
    )
    plugin_manager.load_plugins(plugin_dir)
    result = plugin_manager.execute_plugin("s", [1, 2, 3])
    assert result == [3, 2, 1]


def test_plugin_rollback(tmp_path):
    """Old plugin remains when new version fails."""
    plugin_dir = tmp_path / "plugins"
    plugin_dir.mkdir()
    good = plugin_dir / "p.py"
    good.write_text(
        """
from cam_slicer.plugin_manager import Plugin

def register():
    def apply(args):
        return 'ok'
    return Plugin(name='p', description='v1', apply=apply, version='1.0')
"""
    )
    plugin_manager.load_plugins(plugin_dir)
    assert plugin_manager.get_plugin("p").version == "1.0"

    good.write_text("raise RuntimeError('bad')")
    plugin_manager.reload_plugins()
    assert plugin_manager.get_plugin("p").version == "1.0"
