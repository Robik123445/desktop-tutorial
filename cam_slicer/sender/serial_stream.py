import logging
from pathlib import Path

try:
    import serial  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional dependency
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

    with serial.Serial(port, baud, timeout=1) as ser, path.open("r", encoding="utf-8") as fh:
        for line in fh:
            cmd = line.strip()
            if not cmd:
                continue
            ser.write((cmd + "\n").encode())
            logging.info("Sent: %s", cmd)
            resp = _read_until_ok(ser)
            if resp.lower() != "ok":
                logging.error("Controller responded with: %s", resp)
                raise RuntimeError(f"Unexpected response: {resp}")


def send_gcode_over_serial(gcode_text: str, port: str, baud: int = 115200) -> str:
    """Send raw G-code lines over serial and capture controller responses."""
    if serial is None:
        raise ImportError("pyserial is required for streaming")

    lines = [ln.strip() for ln in gcode_text.splitlines() if ln.strip()]
    log_lines = []
    with serial.Serial(port, baud, timeout=1) as ser:
        for cmd in lines:
            ser.write((cmd + "\n").encode())
            logging.info("Sent: %s", cmd)
            resp = ser.readline().decode().strip()
            if resp:
                logging.info("Received: %s", resp)
            log_lines.append(f"sent: {cmd}")
            if resp:
                log_lines.append(f"recv: {resp}")
    return "\n".join(log_lines)

try:
    from serial.tools import list_ports  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional dependency
    list_ports = None  # type: ignore

def list_available_ports() -> list[str]:
    """Return list of available serial port device names."""
    if list_ports is None:
        raise ImportError("pyserial is required for listing ports")
    return [p.device for p in list_ports.comports()]
