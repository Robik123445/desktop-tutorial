from cam_slicer.plugin_manager import Plugin

def register():
    def apply(tp):
        return tp
    return Plugin(name='example', description='Example plugin', apply=apply, version='1.0', category='demo')
