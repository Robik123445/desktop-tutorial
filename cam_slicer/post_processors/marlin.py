from __future__ import annotations

from typing import List
from cam_slicer.post_processors.manager import PostProcessor


def register() -> PostProcessor:
    """Post processor for Marlin firmware."""

    def apply(lines: List[str], ctx: dict) -> List[str]:
        processed = ["; Marlin post-processed"]
        processed.extend(lines)
        return processed

    return PostProcessor(
        name="marlin",
        description="Marlin post processor",
        firmware_patterns=["marlin"],
        apply=apply,
        version="1.0",
        category="cnc",
    )
