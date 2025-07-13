import logging
from typing import Sequence, Tuple

from cam_slicer.logging_config import setup_logging

setup_logging()
logger = logging.getLogger(__name__)


class MaterialSimulator:
    """Render a simple preview of machining results."""

    def __init__(self, material: str = "MDF") -> None:
        self.material = material

    def simulate(
        self,
        toolpath: Sequence[Tuple[float, float, float]],
        output_file: str = "simulation.png",
        *,
        title: str | None = None,
    ) -> str:
        """Save a 2D preview image of the toolpath."""
        try:
            import matplotlib.pyplot as plt
        except Exception as exc:  # pragma: no cover - optional dep
            logger.warning("matplotlib unavailable: %s", exc)
            with open(output_file, "w", encoding="utf-8") as fh:
                fh.write("simulation unavailable\n")
            return output_file
        if not toolpath:
            raise ValueError("toolpath empty")

        xs = [p[0] for p in toolpath]
        ys = [p[1] for p in toolpath]
        plt.figure(figsize=(5, 5))
        plt.plot(xs, ys, "b-", label="path")
        plt.title(title or f"Simulation on {self.material}")
        plt.axis("equal")
        plt.savefig(output_file)
        plt.close()
        logger.info("Saved simulation preview to %s", output_file)
        return output_file
