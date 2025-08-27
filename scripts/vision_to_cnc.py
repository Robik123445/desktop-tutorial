"""Compatibility wrapper for the old vision_to_cnc script."""
from cam_slicer.vision_bridge import M_affine, main, px_to_xy, write_move_gcode

__all__ = ["M_affine", "main", "px_to_xy", "write_move_gcode"]

if __name__ == "__main__":
    main()
