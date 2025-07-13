import json
import logging
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import List

from .logging_config import setup_logging

setup_logging()

@dataclass
class Tool:
    """Definition of a cutting tool."""

    name: str
    diameter: float
    type: str
    max_rpm: int


class ToolDatabase:
    """Manage a collection of tools and persist them to JSON."""

    def __init__(self) -> None:
        self.tools: List[Tool] = []

    def add_tool(self, name: str, diameter: float, type: str, max_rpm: int) -> None:
        """Add a new tool to the database."""
        self.tools.append(Tool(name, diameter, type, max_rpm))
        logging.info("Added tool %s", name)

    def remove_tool(self, name: str) -> bool:
        """Remove a tool by name. Returns True if removed."""
        for i, tool in enumerate(self.tools):
            if tool.name == name:
                del self.tools[i]
                logging.info("Removed tool %s", name)
                return True
        logging.warning("Tool %s not found", name)
        return False

    def list_tools(self) -> List[dict]:
        """Return list of tools as dictionaries."""
        return [asdict(t) for t in self.tools]

    def save(self, path: str) -> None:
        """Save all tools to a JSON file."""
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(self.list_tools(), fh, indent=2)
        logging.info("Saved %d tools to %s", len(self.tools), path)

    def load(self, path: str) -> None:
        """Load tools from a JSON file if it exists."""
        file = Path(path)
        if not file.exists():
            logging.warning("Tool database %s missing", path)
            return
        with file.open("r", encoding="utf-8") as fh:
            data = json.load(fh)
        self.tools = [Tool(**item) for item in data]
        logging.info("Loaded %d tools from %s", len(self.tools), path)
