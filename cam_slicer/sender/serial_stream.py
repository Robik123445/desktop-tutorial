from pathlib import Path

try:
    import serial
except ModuleNotFoundError as exc:  # pragma: no cover - optional dependency
    serial = None  # type: ignore

def _read_until_ok(ser):
    """Read lines from serial until 'ok' is received."""
    line = ""
    while True:
        line = ser.readline().decode().strip()
        if not line:
            continue
        if "ok" in line.lower():
            break
    return line

def stream_gcode_live(gcode_path: str | Path, port: str, baud: int) -> None:
    """
    Stream G-code lines from a file to a serial port.

    Raises
    ------
    RuntimeError
        When controller response does not contain ``ok``.
    """

    if serial is None:
        raise ImportError("pyserial is required for streaming")

    path = Path(gcode_path)
    if not path.is_file():
        raise FileNotFoundError(path)

    with serial.Serial(port, baud, timeout=1) as ser:
        for raw in path.read_text().splitlines():
            line = raw.strip()
            if not line:
                continue
            ser.write((line + "\n").encode())
            resp = _read_until_ok(ser)
            if "error" in resp.lower():
                raise RuntimeError(resp)
