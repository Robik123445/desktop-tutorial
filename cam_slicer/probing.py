def probe_heightmap(
    x_range: Tuple[float, float],
    y_range: Tuple[float, float],
    step: float,
    *,
    port: Optional[str] = None,
    baud: int = 115200,
    probe_depth: float = -2.0,
    feed: float = 100.0,
    probe_func: Optional[Callable[[float, float], float]] = None,
    save_path: Optional[str] = None,
) -> ZMap:
    """Probe a grid and return a :class:`ZMap` with measured heights.

    If ``probe_func`` is provided, it will be used to obtain Z values for each
    ``(x, y)`` pair. Otherwise ``port`` must be specified and ``pyserial``
    available so probing commands are sent to the machine.
    """

    if probe_func is None:
        if port is None:
            raise ValueError("port required when probe_func is None")
        if serial is None:
            raise ImportError("pyserial is required for probing")
        ser = serial.Serial(port, baud, timeout=1)

        def probe_func(x: float, y: float) -> float:
            return _probe_using_serial(x, y, ser, probe_depth, feed)

    hm_dict = generate_heightmap(x_range, y_range, step, probe_func)
    points = [(x, y, z) for (x, y), z in hm_dict.items()]
    zmap = ZMap(points)

    if save_path:
        export_heightmap_to_json(hm_dict, save_path)
    logger.info("Probed %d points", len(points))

    if probe_func is not None and port is not None and serial is not None:
        ser.close()

    return zmap
