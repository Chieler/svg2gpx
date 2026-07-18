"""svg2gpx -- turn an SVG shape into a runnable GPS-art route on real streets.

Quick start:

    from svg2gpx import get_route
    route = get_route(41.9285, -87.7075, "star")   # lat, lng, shape
    route.to_gpx("star.gpx")
"""
from .gen import (
    CONFIG, get_route, RouteResult, build_grid, build_route, extract_shape,
    extract_contour, search_placement, feature_ledger, to_lonlat, main,
)
from .gpx import to_gpx
from .shapes import shape_path, bundled_shapes

__version__ = "0.2.0"

__all__ = [
    "get_route", "RouteResult", "CONFIG", "build_grid", "build_route",
    "extract_shape", "extract_contour", "search_placement", "feature_ledger",
    "to_lonlat", "main", "to_gpx", "shape_path", "bundled_shapes", "__version__",
]
