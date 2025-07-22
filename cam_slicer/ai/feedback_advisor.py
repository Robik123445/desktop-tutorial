class AIFeedbackAdvisor:

    def generate_feedback_report(self, toolpath: List[Tuple[float, float, float]],
                                 feedrate: int, material: str = "MDF") -> str:
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
        if not data["recommendations"]:
            lines.append("No issues detected")
        lines.append("")
        lines.append(f"Tool suggestion: {data['tool_recommendation']}")
        lines.append(f"Material suggestion: {data['material_recommendation']}")
        report = "\n".join(lines)
        return report

    def start_live_monitoring(self) -> None:
        """Run live vision monitoring for hands or obstacles."""
        run_live_detection()


def suggest_changes(toolpath: list[tuple[float, float, float]], feedrate: int) -> dict:
    """Return feedrate suggestions for the given toolpath."""
    advisor = AIFeedbackAdvisor()
    return advisor.analyze_toolpath(toolpath, feedrate)
