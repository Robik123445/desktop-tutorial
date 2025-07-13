import os, sys
sys.path.insert(0, os.path.abspath("."))

from cam_slicer.utils import get_active_geofence, reset_active_geofence
from cam_slicer.workspace_scanner import update_global_geofence


def test_update_global_geofence():
    """New scan data replaces zones in the global geofence."""
    reset_active_geofence()
    scan = {"debris": [(0, 0, 1, 1), (2, 2, 3, 3)]}
    fence = update_global_geofence(scan)
    assert fence.is_inside(0.5, 0.5)
    assert fence.is_inside(2.5, 2.5)
