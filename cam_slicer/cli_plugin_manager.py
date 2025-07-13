import argparse
import logging
from cam_slicer.logging_config import setup_logging
from cam_slicer import (
    load_plugins,
    reload_plugins,
    get_plugin,
    get_all_plugins,
    execute_plugin,
)
setup_logging()



def list_plugins() -> None:
    """Print a summary of all loaded plugins to ``stdout``.

    Examples
    --------
    >>> list_plugins()  # doctest: +SKIP
    reverse_path: Reverse toolpath
    """
    for info in get_all_plugins():
        cat = f" ({info['category']})" if info.get("category") else ""
        print(f"{info['name']}: {info['description']}{cat}")


def run_plugin(name: str, plugin_args: list[str] | None = None) -> None:
    """Execute a loaded plugin.

    Parameters
    ----------
    name : str
        Plugin identifier as returned by :func:`list_plugins`.
    plugin_args : list of str, optional
        Raw command line arguments forwarded to the plugin's ``apply``
        callable.

    Examples
    --------
    >>> run_plugin('reverse_path', ['example.gcode'])  # doctest: +SKIP
    """

    plugin = get_plugin(name)
    if not plugin:
        print(f"Plugin '{name}' not found")
        return

    plugin_args = plugin_args or []
    try:
        result = execute_plugin(name, plugin_args)
        print(f"Result: {result}")
    except Exception as exc:  # pragma: no cover - output only
        logging.error("Plugin %s execution failed: %s", name, exc)
        print(f"Execution failed: {exc}")


def main(argv=None) -> None:
    """Command line interface entry point.

    Parameters
    ----------
    argv : list of str, optional
        Arguments to parse instead of ``sys.argv``. Useful for testing.

    Examples
    --------
    >>> main(['list'])  # doctest: +SKIP
    """
    load_plugins()
    parser = argparse.ArgumentParser(
        description="CAM Slicer plugin manager (docs: https://example.github.io/cam_slicer/)",
        epilog="Full documentation: https://example.github.io/cam_slicer/",
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("list", help="List available plugins")
    sub.add_parser("reload", help="Reload plugins")
    run_p = sub.add_parser("run", help="Run plugin with test input")
    run_p.add_argument("name", help="Plugin name")

    args, plugin_args = parser.parse_known_args(argv)

    if args.cmd == "list":
        list_plugins()
    elif args.cmd == "reload":
        reload_plugins()
        print("Plugins reloaded")
    elif args.cmd == "run":
        # Forward remaining CLI arguments to the plugin
        run_plugin(args.name, plugin_args)


if __name__ == "__main__":  # pragma: no cover
    main()
