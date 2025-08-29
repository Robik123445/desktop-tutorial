import logging
import time
from typing import List

try:  # pragma: no cover - cv2 may be missing
    import cv2  # type: ignore
except Exception:  # pragma: no cover
    cv2 = None  # type: ignore
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from . import camera_config as cc

logger = logging.getLogger(__name__)
router = APIRouter()

MAX_INDEX = 10
TIMEOUT = 0.5


def try_open(index: int) -> bool:
    """Skúsi otvoriť kameru a načítať jeden frame."""
    if cv2 is None:
        return False
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        cap.release()
        return False
    start = time.time()
    ok = False
    while time.time() - start < TIMEOUT:
        ok, _ = cap.read()
        if ok:
            break
    cap.release()
    return bool(ok)


class CameraInfo(BaseModel):
    index: int
    ok: bool


class CamerasResponse(BaseModel):
    available: List[CameraInfo]
    default: int


class CameraIndex(BaseModel):
    index: int


class CameraSelectResponse(BaseModel):
    index: int
    message: str


@router.get("/vision/cameras", response_model=CamerasResponse)
def list_cameras() -> CamerasResponse:
    """Vracia zoznam dostupných kamier."""
    cams = [CameraInfo(index=i, ok=try_open(i)) for i in range(MAX_INDEX + 1)]
    return CamerasResponse(available=cams, default=cc.get_current_index())


@router.get("/vision/camera/current", response_model=CameraIndex)
def current_camera() -> CameraIndex:
    """Vráti aktuálny index kamery."""
    return CameraIndex(index=cc.get_current_index())


@router.post("/vision/camera/select", response_model=CameraSelectResponse)
def select_camera(req: CameraIndex) -> CameraSelectResponse:
    """Uloží zvolenú kameru po validácii."""
    if not 0 <= req.index <= MAX_INDEX:
        raise HTTPException(status_code=400, detail="Neplatný index")
    if not try_open(req.index):
        raise HTTPException(status_code=400, detail="Kameru sa nepodarilo otvoriť")
    cc.set_current_index(req.index)
    logger.info("Zvolená kamera %d uložená", req.index)
    return CameraSelectResponse(index=req.index, message="Kamera nastavená")
