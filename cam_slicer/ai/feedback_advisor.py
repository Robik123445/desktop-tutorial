"""Lightweight AI feedback utilities for toolpath analysis and monitoring."""

from __future__ import annotations

import logging


logger = logging.getLogger(__name__)


class AIFeedbackAdvisor:
    """Provide simple heuristic advice for CNC toolpaths."""

    def __init__(
        self,
        report_file: str | None = None,
        collision_z: float = -10.0,
        wear_length: float = float("inf"),
    ) -> None:
        """Store configuration for later analysis.

        Parameters
        ----------
        report_file : str, optional
            File where a text summary will be written.
        collision_z : float, optional
            Z threshold below which potential collisions are reported.
        wear_length : float, optional
            Path length after which tool wear is suggested.
        """
        self.report_file = report_file
        self.collision_z = collision_z
        self.wear_length = wear_length

    def analyze_toolpath(
        self, toolpath: list[tuple[float, float, float]], feedrate: int
    ) -> dict:
        """Analyse a toolpath and produce basic recommendations.

        Detects sharp direction changes, deep moves that may collide, and
        repeated cuts that can wear the tool.
        """
        logger.debug("Analyzing toolpath with %d points", len(toolpath))
        recommendations: list[dict[str, float | str]] = []

        # Detect sharp angles
        for i in range(1, len(toolpath) - 1):
            x1, y1, _ = toolpath[i - 1]
            x2, y2, _ = toolpath[i]
            x3, y3, _ = toolpath[i + 1]

            v1 = (x2 - x1, y2 - y1)
            v2 = (x3 - x2, y3 - y2)
            mag1 = (v1[0] ** 2 + v1[1] ** 2) ** 0.5
            mag2 = (v2[0] ** 2 + v2[1] ** 2) ** 0.5
            if mag1 and mag2:
                dot = v1[0] * v2[0] + v1[1] * v2[1]
                cos_angle = dot / (mag1 * mag2)
                if cos_angle < 0.5:  # >60° change
                    recommendations.append(
                        {
                            "x": x2,
                            "y": y2,
                            "reason": "sharp_angle",
                            "suggested_feedrate": feedrate * 0.5,
                        }
                    )

        # Detect potential collisions
        for x, y, z in toolpath:
            if z < self.collision_z:
                recommendations.append(
                    {
                        "x": x,
                        "y": y,
                        "reason": "potential_collision",
                        "suggested_feedrate": feedrate,
                    }
                )

        # Detect repeated cuts (same segment visited twice)
        seen: set[tuple[tuple[float, float, float], tuple[float, float, float]]] = set()
        repeated = False
        for i in range(len(toolpath) - 1):
            seg = (toolpath[i], toolpath[i + 1])
            rev = (toolpath[i + 1], toolpath[i])
            if seg in seen or rev in seen:
                repeated = True
                recommendations.append(
                    {
                        "x": seg[0][0],
                        "y": seg[0][1],
                        "reason": "repeated_cut",
                        "suggested_feedrate": feedrate,
                    }
                )
            else:
                seen.add(seg)

        tool_reco = (
            "Replace tool; signs of wear"
            if repeated or len(toolpath) > self.wear_length
            else "Tool condition acceptable"
        )
        material_reco = "Use standard parameters"

        report = {
            "recommendations": recommendations,
            "tool_recommendation": tool_reco,
            "material_recommendation": material_reco,
        }

        if self.report_file:
            try:
                with open(self.report_file, "w", encoding="utf-8") as fh:
                    for rec in recommendations:
                        fh.write(
                            f"{rec['reason']} at X{rec['x']:.2f} Y{rec['y']:.2f}\n"
                        )
            except OSError as exc:  # pragma: no cover - file errors
                logger.error("Failed to write report: %s", exc)

        return report

    def generate_feedback_report(
        self,
        toolpath: list[tuple[float, float, float]],
        feedrate: int,
        material: str = "MDF",
    ) -> str:
        """Return a formatted textual report for UI display."""
        data = self.analyze_toolpath(toolpath, feedrate)
        lines = [f"Material: {material}", ""]
        for rec in data["recommendations"]:
            pos = f"X{rec['x']:.2f} Y{rec['y']:.2f}"
            if rec["reason"] == "sharp_angle":
                lines.append(f"Slow down near {pos} to F{rec['suggested_feedrate']:.0f}")
            elif rec["reason"] == "potential_collision":
                lines.append(f"Check possible collision at {pos}")
            elif rec["reason"] == "sudden_speed_change":
                lines.append(f"Speed change at {pos}; review program")
            elif rec["reason"] == "repeated_cut":
                lines.append(f"Repeated cut detected near {pos}")
        if not data["recommendations"]:
            lines.append("No issues detected")
        lines.append("")
        lines.append(f"Tool suggestion: {data['tool_recommendation']}")
        lines.append(f"Material suggestion: {data['material_recommendation']}")
        report = "\n".join(lines)
        logger.info(
            "Generated feedback report with %d recommendations",
            len(data["recommendations"]),
        )
        return report

    def start_live_monitoring(self) -> None:
        """Run live vision monitoring for hands or obstacles."""
        logger.info("Starting live monitoring")
        from cam_slicer.vision import run_live_detection

        run_live_detection()


def suggest_changes(
    toolpath: list[tuple[float, float, float]], feedrate: int
) -> dict:
    """Return feedrate suggestions for the given toolpath."""
    logger.debug(
        "Providing change suggestions for %d toolpath points", len(toolpath)
    )
    advisor = AIFeedbackAdvisor()
    return advisor.analyze_toolpath(toolpath, feedrate)

