from cam_slicer import plugin_manager
from cam_slicer import plugin_marketplace
from pathlib import Path

def test_install_and_remove_plugin(tmp_path):
    """Install plugin from marketplace and remove it."""
    market_dir = Path('cam_slicer/plugins_market')
    plugin_dir = tmp_path / 'plugins'
    plugin_manager.load_plugins(plugin_dir)
    assert plugin_marketplace.install_plugin('example', market_dir, plugin_dir)
    plugin_manager.load_plugins(plugin_dir)
    assert plugin_manager.get_plugin('example')
    assert plugin_marketplace.remove_plugin('example', plugin_dir)
