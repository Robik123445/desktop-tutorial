"""Async task server with WebSocket status updates."""
from __future__ import annotations

import asyncio
import logging
import contextlib
from uuid import uuid4
from typing import Any, Awaitable, Callable, Dict, List, Tuple

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect

from .logging_config import setup_logging

# -----------------------------------------------------
# Setup logging to logs/log.txt
setup_logging(log_file="logs/log.txt")
logger = logging.getLogger(__name__)

app = FastAPI(
    title="CNC Assistant API",
    description="Run common CNC assistant tasks",
    openapi_tags=[{"name": "assistant", "description": "High level actions"}],
)

# -----------------------------------------------------
# Global task registry and queue
TaskFunc = Callable[[str], Awaitable[None]]
_registered_tasks: Dict[str, TaskFunc] = {}
task_queue: asyncio.Queue[Tuple[str, TaskFunc]] = asyncio.Queue()
# task_id -> last status string
statuses: Dict[str, str] = {}

# WebSocket clients to broadcast progress
ws_clients: List[WebSocket] = []

# -----------------------------------------------------
# Helper functions

def register_task(name: str, fn: TaskFunc) -> None:
    """Register a task implementation."""
    _registered_tasks[name] = fn

def get_registered_task(name: str) -> TaskFunc | None:
    """Return task callback by name."""
    return _registered_tasks.get(name)

async def broadcast_status(msg: str) -> None:
    """Send status message to all connected WebSocket clients."""
    for ws in ws_clients.copy():
        try:
            await ws.send_text(msg)
        except Exception:
            try:
                ws_clients.remove(ws)
            except ValueError:
                pass

# -----------------------------------------------------
# Background worker

async def task_worker() -> None:
    """Consume tasks from the queue and execute them."""
    while True:
        task_id, func = await task_queue.get()
        statuses[task_id] = "running"
        await broadcast_status(f"{task_id}: running")
        try:
            await func(task_id)
            statuses[task_id] = "done"
            await broadcast_status(f"{task_id}: done")
        except Exception as exc:  # pragma: no cover - execution errors
            statuses[task_id] = f"error: {exc}"
            logger.error("Task %s failed: %s", task_id, exc)
            await broadcast_status(f"{task_id}: error")
        task_queue.task_done()

# -----------------------------------------------------
# FastAPI startup/shutdown hooks

@app.on_event("startup")
async def start_worker() -> None:
    """Launch the background worker."""
    app.state.worker = asyncio.create_task(task_worker())

@app.on_event("shutdown")
async def stop_worker() -> None:
    """Stop the background worker."""
    app.state.worker.cancel()
    with contextlib.suppress(Exception):
        await app.state.worker

# -----------------------------------------------------
# API endpoints

@app.post(
    "/run/{task}",
    tags=["assistant"],
    responses={
        200: {"description": "Task enqueued"},
        400: {"description": "Unknown task"},
        500: {"description": "Execution error"},
    },
)
async def run_task(task: str) -> Any:
    """Queue selected task and return its id."""
    func = get_registered_task(task)
    if func is None:
        raise HTTPException(status_code=400, detail="Unknown task")
    task_id = str(uuid4())
    statuses[task_id] = "queued"
    await broadcast_status(f"{task_id}: queued")
    await task_queue.put((task_id, func))
    return {"task_id": task_id, "status": "queued"}

@app.websocket("/ws/status")
async def status_ws(ws: WebSocket) -> None:
    """Register WebSocket client for status updates."""
    await ws.accept()
    ws_clients.append(ws)
    try:
        while True:
            await ws.receive_text()  # keep connection open
    except WebSocketDisconnect:
        pass
    finally:
        if ws in ws_clients:
            ws_clients.remove(ws)

# -----------------------------------------------------
# Dummy task example

async def dummy_task(task_id: str) -> None:
    """Example long-running task used for testing."""
    await broadcast_status(f"{task_id}: starting")
    await asyncio.sleep(1)
    await broadcast_status(f"{task_id}: working")
    await asyncio.sleep(2)
    await broadcast_status(f"{task_id}: done")

register_task("dummy_task", dummy_task)

def create_app() -> FastAPI:
    """Return configured FastAPI app."""
    return app
