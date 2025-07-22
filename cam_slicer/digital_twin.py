validate_toolpath(
            toolpath,
            axis_range=axis_range or DEFAULT_AXIS_RANGE,
            max_feedrate=max_feedrate,
            max_acceleration=max_acceleration,
        )

        preview_points = []
        for p in toolpath:
            x, y, z = p
            if heightmap:
                z += heightmap.get_offset(x, y)
            preview_points.append((x, y, z))
            if delay and interval > 0:
                time.sleep(interval)
