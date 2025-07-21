start = time.monotonic()
    while True:
        line = port.readline().decode().strip()
        if not line:
            if time.monotonic() - start > timeout:
                raise TimeoutError("No response from controller")
            continue
        logger.debug("RX: %s", line)
        if line.lower().startswith("error"):
            raise RuntimeError(line)
        if "ok" in line.lower():
            return
        if time.monotonic() - start > timeout:
            raise TimeoutError("Timeout waiting for ok")


def stream_gcode_to_grbl(
    gcode_path: str | Path,
    port: str,
    baud: int = 115200,
    *,
    timeout: float = 1.0,
) -> None:
    """Send a G-code file to a GRBL controller line by line."""

    if serial is None:
        raise ImportError("pyserial is required for streaming")

    path = Path(gcode_path)
    if not path.is_file():
        raise FileNotFoundError(path)

    with serial.Serial(port, baud, timeout=timeout) as ser:
        for raw in path.read_text().splitlines():
            line = raw.strip()
            if not line:
                continue
            logger.debug("TX: %s", line)
            ser.write((line + "\n").encode())
            _wait_for_ok(ser)
