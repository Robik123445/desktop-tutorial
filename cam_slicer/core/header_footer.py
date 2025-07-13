import logging
from dataclasses import dataclass
from cam_slicer.logging_config import setup_logging

setup_logging()

@dataclass
class ControllerConfig:
    """Configuration for CNC controller.

    Attributes
    ----------
    CONTROLLER_TYPE : str
        Identifier such as ``'grbl'`` or ``'smoothie'``. Used to select
        appropriate G-code prologue and epilogue.
    """

    CONTROLLER_TYPE: str


def _get_header_footer(controller_config: ControllerConfig) -> tuple[str, str]:
    """Return header and footer G-code for given controller.

    Parameters
    ----------
    controller_config : ControllerConfig
        Controller description.

    Returns
    -------
    tuple[str, str]
        Header and footer strings.

    Raises
    ------
    ValueError
        If the controller type is unknown.

    Examples
    --------
    >>> cfg = ControllerConfig(CONTROLLER_TYPE='grbl')
    >>> _get_header_footer(cfg)
    ('grbl-header', 'grbl-footer')
    """
    ctype = controller_config.CONTROLLER_TYPE.lower()
    logging.info("Preparing header/footer for %s", ctype)
    if ctype == "grbl":
        return "grbl-header", "grbl-footer"
    if ctype == "smoothie":
        return "smoothie-header", "smoothie-footer"
    if ctype == "fanuc":
        return "fanuc-header", "fanuc-footer"
    if ctype == "siemens":
        return "siemens-header", "siemens-footer"
    if ctype == "haas":
        return "haas-header", "haas-footer"
    if ctype == "linuxcnc":
        return "linuxcnc-header", "linuxcnc-footer"
    logging.error("Unsupported controller type: %s", controller_config.CONTROLLER_TYPE)
    raise ValueError(f"Unsupported controller type: {controller_config.CONTROLLER_TYPE}")
