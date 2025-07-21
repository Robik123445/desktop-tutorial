def _parse_xyz(line: str) -> tuple[float, float, float]:
    """Extract X, Y, Z from a G-code line if present."""
    x = y = z = 0.0
    for token in line.split():
        if token.startswith("X"):
            try:
                x = float(token[1:])
            except ValueError:
                pass
        elif token.startswith("Y"):
            try:
                y = float(token[1:])
            except ValueError:
                pass
        elif token.startswith("Z"):
            try:
                z = float(token[1:])
            except ValueError:
                pass
    return x, y, z


def _replace_z(line: str, new_z: float) -> str:
    """Return line with Z value replaced."""
    tokens = []
    replaced = False
    for token in line.split():
        if token.startswith("Z") and not replaced:
            tokens.append(f"Z{new_z:.3f}")
            replaced = True
        else:
            tokens.append(token)
    if not replaced:
        tokens.append(f"Z{new_z:.3f}")
    return " ".join(tokens)


def stream_gcode_with_feedback(
    gcode_path: str,
    port: str,
    measure_fn: Callable[[float, float], float],
    *,
    baud: int = 115200,
    threshold: float = 0.1,
) -> None:
    """Stream G-code adjusting Z based on measurements."""

    if serial is None:
        raise ImportError("pyserial is required for streaming")

    path = Path(gcode_path)
    if not path.is_file():
        raise FileNotFoundError(path)

    lines = path.read_text().splitlines()
    last_error = 0.0

    with serial.Serial(port, baud, timeout=1) as ser:
        for idx, raw in enumerate(lines):
            line = raw.strip()
            if not line:
                continue
            x, y, z = _parse_xyz(line)
            if idx > 0 and abs(last_error) >= threshold:
                line_to_send = _replace_z(line, z - last_error)
            else:
                line_to_send = _replace_z(line, z)
            ser.write((line_to_send + "\n").encode())
            _wait_for_ok(ser)
            measured_z = measure_fn(x, y)
            last_error = measured_z - z
