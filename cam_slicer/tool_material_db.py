import json
import logging
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from .tool_database import Tool
from .logging_config import setup_logging

setup_logging()

@dataclass
class Material:
    """Description of a material."""

    name: str
    notes: str = ""
    hardness: float = 1.0


@dataclass
class FeedSpeed:
    """Recommended parameters for a tool/material pair."""

    tool: str
    material: str
    rpm: int
    feedrate: float


class ToolMaterialDB:
    """Manage tools, materials and feed/speed suggestions."""

    def __init__(self) -> None:
        self.tools: Dict[str, Tool] = {}
        self.materials: Dict[str, Material] = {}
        self.params: Dict[Tuple[str, str], FeedSpeed] = {}

    # Tool handling
    def add_tool(self, name: str, diameter: float, type: str, max_rpm: int) -> None:
        """Store new tool."""
        self.tools[name] = Tool(name, diameter, type, max_rpm)
        logging.info("Added tool %s", name)

    def find_tool(self, query: str) -> List[Tool]:
        """Return tools whose name contains the query."""
        q = query.lower()
        return [t for t in self.tools.values() if q in t.name.lower()]

    # Material handling
    def add_material(self, name: str, notes: str = "", hardness: float = 1.0) -> None:
        """Store material entry."""
        self.materials[name] = Material(name, notes, hardness)
        logging.info("Added material %s", name)

    def find_material(self, query: str) -> List[Material]:
        """Search materials by substring."""
        q = query.lower()
        return [m for m in self.materials.values() if q in m.name.lower()]

    # Feed/speed parameters
    def set_params(self, tool: str, material: str, rpm: int, feedrate: float) -> None:
        """Assign recommended parameters for tool and material."""
        self.params[(tool, material)] = FeedSpeed(tool, material, rpm, feedrate)
        logging.info("Params set for %s on %s", tool, material)

    def get_params(self, tool: str, material: str) -> Optional[FeedSpeed]:
        """Retrieve stored parameters if available."""
        return self.params.get((tool, material))

    # Suggestion logic
    def suggest_for_job(
        self,
        operation: str,
        geometry: str,
        machine_limits: Dict[str, float],
    ) -> Optional[FeedSpeed]:
        """Pick a tool/material combo heuristically."""
        # simplistic: prefer first matching entry
        for (t_name, m_name), param in self.params.items():
            tool = self.tools.get(t_name)
            if not tool:
                continue
            if operation == "cut" and tool.type == "flat":
                return param
            if operation == "engrave" and tool.type in {"v-bit", "engraver"}:
                return param
        return next(iter(self.params.values()), None)

    # Persistence
    def save(self, path: str) -> None:
        """Save databases to JSON."""
        data = {
            "tools": [asdict(t) for t in self.tools.values()],
            "materials": [asdict(m) for m in self.materials.values()],
            "params": [asdict(p) for p in self.params.values()],
        }
        Path(path).write_text(json.dumps(data, indent=2), encoding="utf-8")
        logging.info("Library saved to %s", path)

    def load(self, path: str) -> None:
        """Load databases from JSON if file exists."""
        file = Path(path)
        if not file.exists():
            logging.warning("Library %s missing", path)
            return
        data = json.loads(file.read_text(encoding="utf-8"))
        self.tools = {t["name"]: Tool(**t) for t in data.get("tools", [])}
        self.materials = {
            m["name"]: Material(**m) for m in data.get("materials", [])
        }
        self.params = {
            (p["tool"], p["material"]): FeedSpeed(**p)
            for p in data.get("params", [])
        }
        logging.info("Library loaded from %s", path)

    # Cloud sync (placeholder)
    def sync_to_cloud(self, url: str, token: Optional[str] = None) -> bool:
        """Upload library JSON to a cloud endpoint."""
        try:
            import requests
        except Exception:  # pragma: no cover - optional dependency
            logging.error("requests not installed")
            return False

        headers = {"Authorization": f"Bearer {token}"} if token else {}
        try:
            response = requests.post(url, json=self._export_dict(), headers=headers, timeout=5)
            response.raise_for_status()
        except Exception as exc:  # pragma: no cover - network errors
            logging.error("Cloud sync failed: %s", exc)
            return False
        logging.info("Synced library to %s", url)
        return True

    def load_from_cloud(self, url: str, token: Optional[str] = None) -> bool:
        """Download library JSON from cloud endpoint."""
        try:
            import requests
        except Exception:  # pragma: no cover - optional dependency
            logging.error("requests not installed")
            return False
        headers = {"Authorization": f"Bearer {token}"} if token else {}
        try:
            response = requests.get(url, headers=headers, timeout=5)
            response.raise_for_status()
            data = response.json()
        except Exception as exc:  # pragma: no cover - network errors
            logging.error("Failed to load library: %s", exc)
            return False
        self.tools = {t["name"]: Tool(**t) for t in data.get("tools", [])}
        self.materials = {m["name"]: Material(**m) for m in data.get("materials", [])}
        self.params = {(p["tool"], p["material"]): FeedSpeed(**p) for p in data.get("params", [])}
        logging.info("Loaded library from %s", url)
        return True

    def _export_dict(self) -> Dict[str, list]:
        return {
            "tools": [asdict(t) for t in self.tools.values()],
            "materials": [asdict(m) for m in self.materials.values()],
            "params": [asdict(p) for p in self.params.values()],
        }
