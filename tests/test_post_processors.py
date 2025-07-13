from cam_slicer.post_processors import (
    load_post_processors,
    match_post_processor,
    auto_post_process,
)


def test_match_post_processor():
    """Match firmware string to built-in processor."""
    load_post_processors()
    pp = match_post_processor("Grbl 1.1h")
    assert pp and pp.name == "grbl"


def test_auto_post_process(tmp_path, monkeypatch):
    """Auto process uses detected firmware."""
    load_post_processors()
    sample = ["G1 X0 Y0"]

    def fake_detect(port, baud=115200, timeout=2.0, test_response=None):
        return "Marlin 2.0"

    monkeypatch.setattr(
        "cam_slicer.post_processors.manager.detect_machine_firmware",
        fake_detect,
    )
    out = auto_post_process(sample, "/dev/null")
    assert out[0].startswith("; Marlin")
