import logging
from typing import Sequence, Tuple, List

from cam_slicer.logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)


class ForcePredictor:
    """Predict cutting forces along a toolpath using a simple heuristic."""

    MATERIAL_FACTORS = {
        "plywood": 0.3,
        "mdf": 0.4,
        "aluminum": 0.8,
        "steel": 1.2,
        "acrylic": 0.5,
    }

    def __init__(self, material: str = "aluminum") -> None:
        self.material = material.lower()
        self.factor = self.MATERIAL_FACTORS.get(self.material, 0.7)

    def predict_forces(
        self,
        toolpath: Sequence[Tuple[float, float, float]],
        feedrate: float,
        depth: float = 1.0,
    ) -> List[float]:
        """Return estimated cutting force for each segment."""
        if len(toolpath) < 2:
            raise ValueError("toolpath must contain at least two points")
        forces: List[float] = []
        for i in range(1, len(toolpath)):
            x0, y0, _ = toolpath[i - 1]
            x1, y1, _ = toolpath[i]
            distance = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
            force = self.factor * feedrate * depth * distance
            forces.append(force)
        logger.debug("Predicted forces: %s", forces)
        return forces


def simulate_force_profile(
    toolpath: Sequence[Tuple[float, float, float]],
    feedrate: float,
    depth: float = 1.0,
    material: str = "aluminum",
    output_file: str = "force_sim.png",
    force_limit: float = 1000.0,
) -> Tuple[str, List[int]]:
    """Simulate cutting forces and save an overlay image.

    Returns the output file path and indices of segments exceeding `force_limit`.
    """
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:  # pragma: no cover - optional dep
        logger.warning("matplotlib unavailable: %s", exc)
        with open(output_file, "w", encoding="utf-8") as fh:
            fh.write("force simulation unavailable\n")
        return output_file, []

    predictor = ForcePredictor(material)
    forces = predictor.predict_forces(toolpath, feedrate, depth)
    warnings: List[int] = []

    xs = [p[0] for p in toolpath]
    ys = [p[1] for p in toolpath]

    plt.figure(figsize=(5, 5))
    for i, force in enumerate(forces):
        x0, y0 = xs[i], ys[i]
        x1, y1 = xs[i + 1], ys[i + 1]
        if force > force_limit:
            color = "red"
            warnings.append(i)
        else:
            color = "green"
        plt.plot([x0, x1], [y0, y1], color=color)
    plt.title(f"Force simulation on {material}")
    plt.axis("equal")
    plt.savefig(output_file)
    plt.close()
    if warnings:
        logger.warning("High forces on segments: %s", warnings)
    logger.info("Saved force simulation to %s", output_file)
    return output_file, warnings
