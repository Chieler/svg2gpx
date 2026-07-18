"""The `svg2gpx` command: CLI overrides for CONFIG, plus GPX export.

`python -m svg2gpx` and the installed `svg2gpx` console script both land here.
"""
import argparse

from . import gen
from .gpx import to_gpx
from .shapes import shape_path


def _parse(argv=None):
    ap = argparse.ArgumentParser(
        prog="svg2gpx",
        description="Turn an SVG shape into a runnable GPS-art route on real streets.")
    ap.add_argument("--svg", help="a bundled shape stem (e.g. 'star') or a path "
                                  "to your own SVG")
    ap.add_argument("--lat", type=float, help="network center latitude")
    ap.add_argument("--lng", type=float, help="network center longitude")
    ap.add_argument("--radius", type=float, dest="radius_m",
                    help="network radius around the center, metres")
    ap.add_argument("--graphml", help="load this saved GraphML instead of "
                                      "fetching from the Overpass API")
    ap.add_argument("--granularity", type=float,
                    help="0=smooth/simple .. 1=trace every jog")
    ap.add_argument("--seed", type=int, help="placement-search RNG seed")
    ap.add_argument("--save", help="write the plot to this image path")
    ap.add_argument("--gpx", help="also write the best route as a GPX track here "
                                  "(needs --lat/--lng or --graphml: a real "
                                  "network, not the synthetic grid)")
    ap.add_argument("--no-show", action="store_true",
                    help="don't open a window (headless; use with --save)")
    ap.add_argument("--no-inner-features", action="store_true",
                    help="outline only: skip placing/routing/scoring the "
                         "shape's inner features")
    return ap.parse_args(argv)


def main(argv=None):
    args = _parse(argv)

    cfg = dict(gen.CONFIG)
    if args.svg is not None:
        args.svg = shape_path(args.svg)
    if args.gpx:
        # A GPX file is one route, not a panel of options to eyeball.
        cfg["present_options"] = False
    for key, val in [("svg_path", args.svg), ("lat", args.lat), ("lng", args.lng),
                     ("radius_m", args.radius_m), ("graphml_path", args.graphml),
                     ("granularity", args.granularity), ("seed", args.seed),
                     ("save_plot", args.save)]:
        if val is not None:
            cfg[key] = val
    if args.no_show:
        cfg["show_plot"] = False
    if args.no_inner_features:
        cfg["inner_features"] = False

    grid, candidates = gen.main(cfg)

    if args.gpx:
        best = candidates[0]
        latlon = gen.to_lonlat(best.route, grid)
        to_gpx(latlon, args.gpx, name=f"svg2gpx: {cfg['svg_path']}")
        print(f"wrote {args.gpx}")


if __name__ == "__main__":
    main()
