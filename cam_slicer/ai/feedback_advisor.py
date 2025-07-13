import logging
import math
from typing import List, Tuple, Dict, Any
from collections import defaultdict
from cam_slicer.robotics.kinematics import calculate_junction_velocity
from cam_slicer.vision.debris_detector import run_live_detection



from cam_slicer.logging_config import setup_logging
setup_logging()
class AIFeedbackAdvisor:
    """Analyse toolpaths and generate feedrate suggestions.

    The advisor also recommends tools, materials and can start a live
    monitoring loop that uses the vision module to detect hands or
    obstacles and trigger an emergency stop.
    """

    def __init__(
        self,
        report_file: str = "advisor_report.txt",
        *,
        collision_z: float = -10.0,
        wear_length: float = 10000.0,
    ) -> None:
        """Create advisor with thresholds for collision and wear."""
        self.report_file = report_file
        self.collision_z = collision_z
        self.wear_length = wear_length

    def _log_bottleneck(self, idx: int, x: float, y: float, reason: str,
                         suggested_fr: float) -> None:
        message = (f"Bottleneck at {idx} X{x:.3f} Y{y:.3f} - {reason}"
                   f" suggest F{suggested_fr:.1f}")
        with open(self.report_file, "a", encoding="utf-8") as fh:
            fh.write(message + "\n")
        logging.info(message)

    def _recommend_tool(self, total_length: float, avg_angle: float) -> str:
        """Suggest tool based on path length and average angle."""
        if total_length > self.wear_length:
            return "Replace tool - high wear"
        if avg_angle < 60:
            return "Ball end mill"
        return "Flat end mill"

    def _recommend_material(self, collision_count: int) -> str:
        """Recommend material adjustment based on collision risk."""
        if collision_count > 0:
            return "Use softer material or reduce depth"
        return "Material OK"

    def analyze_toolpath(self, toolpath: List[Tuple[float, float, float]],
                         feedrate: int) -> Dict[str, Any]:
        """Return feedrate suggestions and detected issues."""
        if len(toolpath) < 2:
            return {"recommendations": []}

        base_fr = feedrate / 60.0  # mm/s
        max_acc = 1000.0
        jdev = 0.05
        suggestions = []
        prev_len = None
        total_length = 0.0
        angle_sum = 0.0
        collision_count = 0
        seg_map: defaultdict[Tuple[int, int, int, int], int] = defaultdict(int)

        for i in range(1, len(toolpath) - 1):
            p_prev = toolpath[i - 1]
            p_curr = toolpath[i]
            p_next = toolpath[i + 1]
            vec1 = (p_curr[0] - p_prev[0], p_curr[1] - p_prev[1], p_curr[2] - p_prev[2])
            vec2 = (p_next[0] - p_curr[0], p_next[1] - p_curr[1], p_next[2] - p_curr[2])

            seg_len = math.sqrt(vec1[0]**2 + vec1[1]**2 + vec1[2]**2)
            total_length += seg_len
            if prev_len is not None:
                ratio = seg_len / prev_len if prev_len else 0
                if ratio > 2 or ratio < 0.5:
                    suggestions.append({
                        "index": i,
                        "x": p_curr[0],
                        "y": p_curr[1],
                        "z": p_curr[2],
                        "reason": "sudden_speed_change",
                        "suggested_feedrate": feedrate,
                    })
                    self._log_bottleneck(i, p_curr[0], p_curr[1], "speed change",
                                         feedrate)
            prev_len = seg_len

            angle = math.degrees(math.acos(
                max(-1.0, min(1.0,
                    (vec1[0]*vec2[0] + vec1[1]*vec2[1] + vec1[2]*vec2[2]) /
                    (math.sqrt(vec1[0]**2 + vec1[1]**2 + vec1[2]**2) *
                     math.sqrt(vec2[0]**2 + vec2[1]**2 + vec2[2]**2) + 1e-6)))
            ))
            angle_sum += angle
            jv = calculate_junction_velocity(vec1, vec2, max_acc, jdev)
            if math.isfinite(jv):
                fr_suggest = min(base_fr, jv) * 60.0
                if fr_suggest < feedrate:
                    suggestions.append({
                        "index": i,
                        "x": p_curr[0],
                        "y": p_curr[1],
                        "z": p_curr[2],
                        "reason": "sharp_angle",
                        "suggested_feedrate": fr_suggest,
                    })
                    self._log_bottleneck(i, p_curr[0], p_curr[1], "sharp angle",
                                         fr_suggest)

            if p_curr[2] < self.collision_z:
                suggestions.append({
                    "index": i,
                    "x": p_curr[0],
                    "y": p_curr[1],
                    "z": p_curr[2],
                    "reason": "potential_collision",
                    "suggested_feedrate": feedrate,
                })
                collision_count += 1
                self._log_bottleneck(i, p_curr[0], p_curr[1], "collision",
                                     feedrate)

            key = (round(p_prev[0], 3), round(p_prev[1], 3),
                   round(p_curr[0], 3), round(p_curr[1], 3))
            seg_map[key] += 1
            if seg_map[key] > 2:
                suggestions.append({
                    "index": i,
                    "x": p_curr[0],
                    "y": p_curr[1],
                    "z": p_curr[2],
                    "reason": "repeated_cut",
                    "suggested_feedrate": feedrate,
                })
                self._log_bottleneck(i, p_curr[0], p_curr[1],
                                     "repeated cut", feedrate)

        avg_angle = angle_sum / max(1, len(toolpath) - 2)
        result = {
            "total_segments": len(toolpath) - 1,
            "recommendations": suggestions,
            "tool_recommendation": self._recommend_tool(total_length, avg_angle),
            "material_recommendation": self._recommend_material(collision_count),
        }
        return result

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
