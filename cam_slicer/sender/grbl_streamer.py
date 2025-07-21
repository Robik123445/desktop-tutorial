def stream_gcode_interactive(
    gcode_path: str,
    port: str,
    *,
    baud: int = 115200,
    controller: Optional[StreamController] = None,
) -> None:
    """Stream G-code with pause/resume/stop controls."""

    if serial is None:
        raise ImportError("pyserial is required for streaming")

    path = Path(gcode_path)
    if not path.is_file():
        raise FileNotFoundError(path)

    ctrl = controller or StreamController()

    with serial.Serial(port, baud, timeout=1) as ser:
        lines = path.read_text().splitlines()
        idx = 0
        while idx < len(lines):
            if ctrl._stop:
                break
            if ctrl._paused:
                time.sleep(0.1)
                continue
            line = lines[idx].strip()
            if not line:
                idx += 1
                continue
            ser.write((line + "\n").encode())
            _wait_for_ok(ser)
            idx += 1
