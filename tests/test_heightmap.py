import os
import sys
from fastapi.testclient import TestClient

sys.path.insert(0, os.path.abspath("."))

from cam_slicer.heightmap import HeightMap, apply_heightmap_to_gcode
from cam_slicer.api_server import create_app


def test_apply_heightmap() -> None:
    """Apply simple heightmap offsets to a short G-code snippet."""
    gcode = "G1 X0 Y0 Z0\nG1 X1 Y0 Z-1"
    hm = HeightMap.from_text("0,0,0.1\n1,0,0.2", fmt="csv")
    out = apply_heightmap_to_gcode(gcode, hm)
    lines = out.splitlines()
    assert lines[0].endswith("Z0.100")
    assert lines[1].endswith("Z-0.800")


def test_heightmap_endpoint() -> None:
    """Verify API endpoint returns adjusted G-code."""
    os.environ["API_TOKEN"] = "testtoken"
    app = create_app()
    client = TestClient(app)
    payload = {"gcode": "G1 X0 Y0 Z0", "heightmap": "0,0,0.5", "format": "csv"}
    resp = client.post(
        "/heightmap",
        json=payload,
        headers={"X-Access-Token": "testtoken"},
    )
    assert resp.status_code == 200
    assert "Z0.500" in resp.json()["gcode"]
