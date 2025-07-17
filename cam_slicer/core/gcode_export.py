def toolpath_to_gcode(
    toolpath,
    controller_config,
    move_command="G1",
    feedrate=None,
    transform_cfg=None,
    shape="linear",
    arc_support=True,
    z_map=None,
    lookahead=0,
    max_acceleration=1000.0,
    junction_deviation=0.05,
    advance_factor=0.0,
    laser_power=None,
    adaptive_mode=False,
    start_macro=None,
    mid_macro=None,
    end_macro=None,
    parallel=False,
    workers=None,
):
    """Convert a toolpath sequence into G-code.

    Parameters
    ----------
    laser_power : float, optional
        If provided, enable laser mode and set power via ``M3 S<power>`` / ``M5``
        commands.
    """
    header, footer = _get_header_footer(controller_config)
    gcode_lines = [header]
    if start_macro:
        gcode_lines.extend(get_macro(start_macro))
    if laser_power is not None:
        gcode_lines.append(f"M3 S{laser_power:.1f}")
    cfg = transform_cfg or TransformConfig()
    cmd = move_command.upper()
    prev_z = None
    lookahead_queue = []
    path = list(toolpath)
    segments = []
    flat = []
    if shape == "circle" and arc_support and len(path) >= 3:
        arc = {"start": path[0], "center": path[1], "end": path[2]}
        segments.append(("arc", arc))
        flat.extend([path[0][:3], path[1][:3], path[2][:3]])
    else:
        for item in path:
            if isinstance(item, tuple) and item and item[0] == "arc":
                arc = item[1]
                segments.append(("arc", arc))
                flat.extend([arc["start"], arc["center"], arc["end"]])
            else:
                segments.append(("pt", item))
                flat.append(item[:3])

    mid_index = len(flat) // 2 if mid_macro else None
    if parallel:
        transformed = parallel_map(_transform_pt, [(p, cfg) for p in flat], max_workers=workers)
    else:
        transformed = [transform_point(*p[:3], cfg) for p in flat]

    seg_iter = iter(transformed)
    processed = []
    for typ, data in segments:
        if typ == "arc":
            start = next(seg_iter)
            center = next(seg_iter)
            end = next(seg_iter)
            if z_map is not None:
                start = (start[0], start[1], start[2] + z_map.get_offset(*start[:2]))
                end = (end[0], end[1], end[2] + z_map.get_offset(*end[:2]))
            processed.append(("arc", {"start": start, "center": center, "end": end}))
        else:
            pt = next(seg_iter)
            if z_map is not None:
                pt = (pt[0], pt[1], pt[2] + z_map.get_offset(pt[0], pt[1]))
            angles = tuple(data[3:]) if isinstance(data, tuple) and len(data) > 3 else ()
            processed.append(("pt", (pt, angles)))

    point_count = 0
    n = len(processed)
    for seg in processed:
        if seg[0] == "arc":
            arc = seg[1]
            line = _arc_command(arc["start"], arc["center"], arc["end"])
            if cmd == "G1" and feedrate is not None:
                line += f" F{feedrate:.3f}"
            gcode_lines.append(line)
            prev_z = arc["end"][2]
            point_count += 3
        else:
            pt, angles = seg[1]
            x, y, z = pt
            while lookahead > 0 and len(lookahead_queue) < lookahead + 1 and point_count + len(lookahead_queue) < n:
                next_item = processed[point_count + len(lookahead_queue)]
                if next_item[0] == "arc":
                    lookahead_queue.append(next_item[1]["end"])
                else:
                    lookahead_queue.append(next_item[1][0])
            if cmd == "G1" and feedrate is not None:
                base_fr = feedrate
                if lookahead > 0 and len(lookahead_queue) >= 3:
                    min_jv = feedrate
                    for j in range(len(lookahead_queue) - 2):
                        prev_vec = tuple(
                            lookahead_queue[j + 1][k] - lookahead_queue[j][k]
                            for k in range(3)
                        )
                        next_vec = tuple(
                            lookahead_queue[j + 2][k] - lookahead_queue[j + 1][k]
                            for k in range(3)
                        )
                        jv = calculate_junction_velocity(
                            prev_vec,
                            next_vec,
                            max_acceleration,
                            junction_deviation,
                        )
                        if not math.isinf(jv):
                            min_jv = min(min_jv, jv)
                    base_fr = min(base_fr, min_jv)
                if adaptive_mode and z_map is not None:
                    offset = z_map.get_offset(x, y)
                    scale = max(0.5, 1.0 - abs(offset) * 0.1)
                    logging.info("Adaptive mode: offset %.3f scales feed %.3f", offset, base_fr)
                adj_fr = apply_pressure_advance(base_fr, 500.0, advance_factor)
                logging.info("Pressure advance: %.3f -> %.3f", base_fr, adj_fr)
                line = f"{cmd} X{x:.3f} Y{y:.3f} Z{z:.3f}"
                if angles:
                    if len(angles) > 0:
                        line += f" A{angles[0]:.3f}"
                    if len(angles) > 1:
                        line += f" B{angles[1]:.3f}"
                    if len(angles) > 2:
                        line += f" C{angles[2]:.3f}"
                line += f" F{adj_fr:.3f}"
                gcode_lines.append(line)
                prev_z = z
            else:
                line = f"{cmd} X{x:.3f} Y{y:.3f} Z{z:.3f}"
                if angles:
                    if len(angles) > 0:
                        line += f" A{angles[0]:.3f}"
                    if len(angles) > 1:
                        line += f" B{angles[1]:.3f}"
                    if len(angles) > 2:
                        line += f" C{angles[2]:.3f}"
                gcode_lines.append(line)
                prev_z = z
            point_count += 1
            if lookahead_queue:
                lookahead_queue.pop(0)
        if mid_macro and mid_index is not None and point_count == mid_index + 1:
            gcode_lines.extend(get_macro(mid_macro))
            mid_index = None
    if laser_power is not None:
        gcode_lines.append("M5")
    if end_macro:
        gcode_lines.extend(get_macro(end_macro))
    gcode_lines.append(footer)
    return gcode_lines

# Example usage
if __name__ == "__main__":  # pragma: no cover
    tp = [(0, 0, 0), (10, 0, 0)]
    cfg = ControllerConfig(CONTROLLER_TYPE="grbl")
    lines = toolpath_to_gcode(tp, cfg)
    for line in lines:
        print(line)
