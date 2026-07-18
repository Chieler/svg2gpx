"""svg2gpx -- turn an SVG shape into a runnable GPS-art route on real streets."""
from .gen import (
    CONFIG, build_grid, build_route, extract_shape, extract_contour,
    search_placement, feature_ledger, to_lonlat, main,
)
from .gpx import to_gpx
from .shapes import shape_path, bundled_shapes

__version__ = "0.1.0"

__all__ = [
    "CONFIG", "build_grid", "build_route", "extract_shape", "extract_contour",
    "search_placement", "feature_ledger", "to_lonlat", "main",
    "to_gpx", "shape_path", "bundled_shapes", "__version__",
]
