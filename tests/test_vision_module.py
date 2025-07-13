import types
from pathlib import Path

from cam_slicer.vision import debris_detector, board_detector


def test_detect_objects(monkeypatch):
    """Test generic object detection using a stub model."""
    class DummyArray(list):
        def tolist(self):
            return list(self)

    class DummyModel:
        def __call__(self, img):
            boxes = types.SimpleNamespace(xyxy=DummyArray([[0, 0, 10, 10]]))
            res = types.SimpleNamespace(boxes=boxes)
            return [res]

    monkeypatch.setattr(debris_detector, "YOLO", lambda *a, **kw: DummyModel())
    monkeypatch.setattr(debris_detector, "cv2", types.SimpleNamespace(imread=lambda p: [[0]], VideoCapture=lambda x: None))
    monkeypatch.setattr(Path, "is_file", lambda self: True)

    boxes = debris_detector.detect_objects("dummy.jpg")
    assert boxes == [(0.0, 0.0, 10.0, 10.0)]


def test_detect_board_position(monkeypatch):
    """Test detecting board position from an image."""
    class DummyArray(list):
        def tolist(self):
            return list(self)

    class DummyBoxes:
        def __init__(self):
            self.xyxy = DummyArray([[0, 0, 20, 20]])
            self.cls = DummyArray([0])

    class DummyModel:
        def __call__(self, img):
            boxes = DummyBoxes()
            res = types.SimpleNamespace(boxes=boxes, names={0: "board"})
            return [res]

    monkeypatch.setattr(board_detector, "YOLO", lambda *a, **kw: DummyModel())
    monkeypatch.setattr(board_detector, "cv2", types.SimpleNamespace(imread=lambda p: [[0]]))
    monkeypatch.setattr(Path, "is_file", lambda self: True)

    info = board_detector.detect_board_position("dummy.jpg")
    assert info == {"x": 10.0, "y": 10.0, "width": 20.0, "height": 20.0}


def test_get_transform_from_detection():
    """Test conversion from detection dict to TransformConfig."""
    det = {"x": 10, "y": 20}
    cfg = board_detector.get_transform_from_detection(det)
    assert cfg.offset == (1.0, 2.0, 0.0)
    assert cfg.rotation_deg == 0.0
    assert cfg.scale == 1.0


def test_auto_transform_gcode(monkeypatch):
    """Test automatic G-code generation from detection data."""
    calls = {}

    def dummy_toolpath_to_gcode(tp, cfg, transform):
        calls["tp"] = tp
        calls["cfg"] = cfg
        calls["transform"] = transform
        return ["header", "G1 X0", "footer"]

    monkeypatch.setattr(
        "cam_slicer.core.gcode_export.toolpath_to_gcode", dummy_toolpath_to_gcode
    )

    toolpath = [(0, 0, 0)]
    detection = {"x": 10, "y": 20}
    gcode = board_detector.auto_transform_gcode(toolpath, detection)

    assert gcode[0] == "header"
    assert calls["cfg"].CONTROLLER_TYPE == "grbl"
    assert calls["transform"].offset == (1.0, 2.0, 0.0)


def test_auto_transform_gcode_with_scaling(monkeypatch):
    """Toolpath is scaled to match detected board size."""
    calls = {}

    def dummy_toolpath_to_gcode(tp, cfg, transform):
        calls["transform"] = transform
        return []

    monkeypatch.setattr(
        "cam_slicer.core.gcode_export.toolpath_to_gcode", dummy_toolpath_to_gcode
    )

    toolpath = [(0, 0, 0), (10, 5, 0)]
    detection = {"x": 10, "y": 20, "width": 20, "height": 10}

    board_detector.auto_transform_gcode(toolpath, detection)

    assert round(calls["transform"].scale, 2) == 0.2


def test_run_live_detection(monkeypatch):
    """Test live detection loop with warning on person detection."""
    class DummyCap:
        def __init__(self):
            self.released = False

        def isOpened(self):
            return True

        def read(self):
            return True, "frame"

        def release(self):
            self.released = True

    class DummyModel:
        def __call__(self, img):
            class DummyArray(list):
                def tolist(self):
                    return list(self)

            boxes = DummyArray([[0, 0, 10, 20]])
            cls = DummyArray([1])
            return [types.SimpleNamespace(boxes=types.SimpleNamespace(xyxy=boxes, cls=cls), names={1: "person"})]

    cap = DummyCap()
    text_calls = []
    rect_colors = []
    cv2_stub = types.SimpleNamespace(
        VideoCapture=lambda idx: cap,
        rectangle=lambda img, pt1, pt2, color, thickness: rect_colors.append(color),
        putText=lambda img, text, pos, font, scale, color, thickness: text_calls.append(text),
        FONT_HERSHEY_SIMPLEX=1,
        imshow=lambda *a, **kw: None,
        waitKey=lambda x: ord("q"),
        destroyAllWindows=lambda: None,
    )

    monkeypatch.setattr(debris_detector, "YOLO", lambda *a, **kw: DummyModel())
    monkeypatch.setattr(debris_detector, "cv2", cv2_stub)
    printed = []
    monkeypatch.setattr("builtins.print", lambda msg: printed.append(msg))

    from cam_slicer.utils import get_active_geofence, reset_active_geofence
    reset_active_geofence()
    debris_detector.run_live_detection()
    fence = get_active_geofence()
    assert cap.released
    assert "person" in text_calls
    assert "1.0x2.0 mm" in text_calls
    assert rect_colors and rect_colors[0] == (0, 0, 255)
    assert fence.forbidden_zones
    assert any("WARNING" in m for m in printed)


def test_process_camera_to_gcode(monkeypatch, tmp_path):
    """Test full workflow from image to saved G-code."""
    calls = {}

    def dummy_detect_board_position(path):
        calls["image"] = path
        return {"x": 10, "y": 20}

    def dummy_get_transform_from_detection(det):
        calls["det"] = det

        class Dummy:
            offset = (1.0, 2.0, 0.0)

        return Dummy()

    def dummy_toolpath_to_gcode(tp, cfg, transform):
        calls["tp"] = tp
        calls["cfg"] = cfg
        calls["transform"] = transform
        return ["G1 X0"]

    out_file = tmp_path / "out.gcode"
    monkeypatch.setattr(board_detector, "detect_board_position", dummy_detect_board_position)
    monkeypatch.setattr(board_detector, "get_transform_from_detection", dummy_get_transform_from_detection)
    monkeypatch.setattr("cam_slicer.core.gcode_export.toolpath_to_gcode", dummy_toolpath_to_gcode)

    ok = board_detector.process_camera_to_gcode([(0, 0, 0)], "img.jpg", str(out_file))

    assert ok
    assert out_file.read_text().strip() == "G1 X0"
    assert calls["cfg"].CONTROLLER_TYPE == "grbl"
