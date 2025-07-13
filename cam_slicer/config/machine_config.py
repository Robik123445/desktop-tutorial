import logging


from cam_slicer.logging_config import setup_logging
setup_logging()
MACHINE_PRESETS = {
    "MACHINE_GRBL_SMALL": {
        "axis_limits": {
            "X": {"max_velocity": 300.0, "max_acceleration": 600.0},
            "Y": {"max_velocity": 300.0, "max_acceleration": 600.0},
            "Z": {"max_velocity": 50.0, "max_acceleration": 200.0},
        },
        "controller": "GRBL",
        "max_feedrate": 1500.0,
        "acceleration": 800.0,
        "junction_deviation": 0.05,
    },
    "MACHINE_SMOOTHIE_LARGE": {
        "axis_limits": {
            "X": {"max_velocity": 500.0, "max_acceleration": 1000.0},
            "Y": {"max_velocity": 500.0, "max_acceleration": 1000.0},
            "Z": {"max_velocity": 100.0, "max_acceleration": 400.0},
        },
        "controller": "smoothie",
        "max_feedrate": 3000.0,
        "acceleration": 1500.0,
        "junction_deviation": 0.02,
    },
}


def get_machine_config(preset_name: str) -> dict:
    """Return machine configuration for the given preset.

    Parameters
    ----------
    preset_name : str
        Key from ``MACHINE_PRESETS``.

    Returns
    -------
    dict
        Configuration dictionary.

    Raises
    ------
    KeyError
        If the preset name is unknown.
    """

    key = preset_name.upper()
    if key not in MACHINE_PRESETS:
        logging.error("Unknown machine preset: %s", preset_name)
        raise KeyError(f"Unknown machine preset: {preset_name}")

    logging.info("Loaded machine preset: %s", key)
    return MACHINE_PRESETS[key].copy()
