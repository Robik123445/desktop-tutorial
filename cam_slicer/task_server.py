"""Minimal API exposing high-level assistant tasks."""
from __future__ import annotations

import logging
from typing import Any

from fastapi import FastAPI, HTTPException

from .logging_config import setup_logging

setup_logging(log_file="logs/log.txt")
logger = logging.getLogger(__name__)

app = FastAPI(
    title="CNC Assistant API",
    description="Run common CNC assistant tasks",
    openapi_tags=[{"name": "assistant", "description": "High level actions"}],
)


def create_app() -> FastAPI:
    """Return configured FastAPI app."""
    return app


@app.post(
    "/run/{task}",
    tags=["assistant"],
    responses={
        200: {"description": "Task executed"},
        400: {"description": "Unknown task"},
        500: {"description": "Execution error"},
    },
)
async def run_task(task: str) -> Any:
    """Execute selected task and return JSON result."""
    try:
        if task == "vizualizuj":
            from cam_slicer import run_live_detection

            run_live_detection()
            return {"detail": "ok"}
        if task == "analyzuj":
            from cam_slicer import suggest_changes

            result = suggest_changes([] , 0)  # placeholder params
            return {"result": result}
        if task == "robot":
            from cam_slicer import move_to_pose

            move_to_pose([0, 0, 0])
            return {"detail": "ok"}
        raise HTTPException(status_code=400, detail="Unknown task")
    except ImportError as exc:
        logger.error("Import failed: %s", exc)
        raise HTTPException(status_code=500, detail=str(exc))
    except Exception as exc:  # pragma: no cover - unforeseen errors
        logger.error("Task failed: %s", exc)
        raise HTTPException(status_code=500, detail=str(exc))
