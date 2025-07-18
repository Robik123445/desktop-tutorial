points = parse_gcode(lines)
    if laser_mode:
        points = [(x, y, 0.0) for x, y, _ in points]

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except Exception as exc:  # pragma: no cover - optional dep
        logging.error("Matplotlib not available: %s", exc)
        return points

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]

    flat = laser_mode or all(abs(z) < 1e-6 for z in zs)
    if flat:
        fig, ax = plt.subplots()
        ax.plot(xs, ys, color="gray", alpha=0.7)
        sc = ax.scatter(xs, ys, c=zs, cmap="viridis")
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        fig.colorbar(sc, ax=ax, label="Z")
    else:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(xs, ys, zs, color="gray", alpha=0.7)
        sc = ax.scatter(xs, ys, zs, c=zs, cmap="viridis")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        fig.colorbar(sc, ax=ax, label="Z")

    if show:
        plt.show()
    logging.info("Preview generated with %d points", len(points))
    return fig
