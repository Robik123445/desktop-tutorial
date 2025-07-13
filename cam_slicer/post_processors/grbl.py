from __future__ import annotations

from typing import List
from cam_slicer.post_processors.manager import PostProcessor


def register() -> PostProcessor:
    """Post processor for GRBL firmware."""

    def apply(lines: List[str], ctx: dict) -> List[str]:
        # For demo, prepend comment with firmware info
        processed = [f"; GRBL post-processed"]
        processed.extend(lines)
        return processed

    return PostProcessor(
        name="grbl",
        description="GRBL post processor",
        firmware_patterns=["grbl"],
        apply=apply,
        version="1.0",
        category="cnc",
    )
