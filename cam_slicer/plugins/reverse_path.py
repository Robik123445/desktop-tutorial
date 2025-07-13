from typing import List, Tuple
from cam_slicer.plugin_manager import Plugin


def register():
    """Register the reverse path plugin.

    Example
    -------
    >>> plugin = register()
    >>> plugin.apply([(0,0,0), (1,0,0)])  # doctest: +SKIP
    [(1, 0, 0), (0, 0, 0)]
    """

    def apply(toolpath: List[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
        return list(reversed(toolpath))

    return Plugin(
        name="reverse_path",
        description="Reverse the order of toolpath points",
        apply=apply,
        version="1.0",
        category="toolpath",
    )

