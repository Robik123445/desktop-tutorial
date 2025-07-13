from typing import List, Tuple
from cam_slicer.shapes import generate_adaptive_path
from cam_slicer.plugin_manager import Plugin


def register():
    """Register the adaptive path plugin.

    Example
    -------
    >>> plugin = register()
    >>> plugin.apply([(0,0), (1,0), (1,1)])  # doctest: +SKIP
    """

    def apply(
        boundary: List[Tuple[float, float]],
        depth: float = -1.0,
        stepdown: float = 0.5,
        stepover: float = 0.5,
        mode: str = "spiral",
        adaptive_mode: bool = False,
        **kwargs,
    ):
        """Generate a spiral or zigzag toolpath using ``generate_adaptive_path``."""
        return generate_adaptive_path(
            boundary,
            depth=depth,
            stepdown=stepdown,
            stepover=stepover,
            mode=mode,
            adaptive_mode=adaptive_mode,
            **kwargs,
        )

    return Plugin(
        name="adaptive_path",
        description="Generate spiral adaptive toolpaths",
        apply=apply,
        version="1.0",
        category="strategy",
    )
