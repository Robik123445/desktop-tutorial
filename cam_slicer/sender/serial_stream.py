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
