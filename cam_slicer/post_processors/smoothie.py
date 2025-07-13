from __future__ import annotations

from typing import List
from cam_slicer.post_processors.manager import PostProcessor


def register() -> PostProcessor:
    """Post processor for Smoothieware firmware."""

    def apply(lines: List[str], ctx: dict) -> List[str]:
        processed = ["; Smoothieware post-processed"]
        processed.extend(lines)
        return processed

    return PostProcessor(
        name="smoothie",
        description="Smoothieware post processor",
        firmware_patterns=["smoothie", "smoothieware"],
        apply=apply,
        version="1.0",
        category="cnc",
    )
