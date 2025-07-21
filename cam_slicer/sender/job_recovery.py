def stream_with_recovery(
    gcode_path: str,
    port: str,
    *,
    baud: int = 115200,
    checkpoint: str | Path,
    state_cb: Optional[Callable[[str], Dict[str, Any]]] = None,
) -> None:
    """Stream G-code while storing progress to ``checkpoint``."""

    if serial is None:
        raise ImportError("pyserial is required for streaming")

    path = Path(gcode_path)
    if not path.is_file():
        raise FileNotFoundError(path)

    ckpt_path = Path(checkpoint)
    with serial.Serial(port, baud, timeout=1) as ser:
        for idx, raw in enumerate(path.read_text().splitlines()):
            line = raw.strip()
            if not line:
                continue
            ser.write((line + "\n").encode())
            _wait_for_ok(ser)
            save_checkpoint(ckpt_path, idx, state_cb(line) if state_cb else None)


def resume_job(
    gcode_path: str,
    port: str,
    *,
    baud: int = 115200,
    checkpoint: str | Path,
) -> None:
    """Resume streaming from the line after the checkpoint."""

    if serial is None:
        raise ImportError("pyserial is required for streaming")

    data = load_checkpoint(checkpoint)
    start = (data["line"] + 1) if data else 0

    path = Path(gcode_path)
    if not path.is_file():
        raise FileNotFoundError(path)

    lines = path.read_text().splitlines()
    if start >= len(lines):
        return

    ckpt_path = Path(checkpoint)
    with serial.Serial(port, baud, timeout=1) as ser:
        for idx, raw in enumerate(lines[start:], start=start):
            line = raw.strip()
            if not line:
                continue
            ser.write((line + "\n").encode())
            _wait_for_ok(ser)
            save_checkpoint(ckpt_path, idx)
