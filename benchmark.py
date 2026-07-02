"""
Shape-fidelity benchmark for the SVG -> street-route pipeline.

Runs the pipeline (gen.py) over a suite of bundled SVG shapes and reports, per
shape, both how faithfully the routed loop reproduces the target outline and how
long each stage took. The goal is a repeatable score you can watch across tuning
changes, instead of eyeballing one matplotlib window at a time.

Two things make this cheap and reproducible:

  * Synthetic grid (default). build_grid() in gen.py fetches live OpenStreetMap
    data over the network, which is slow and non-deterministic. Here we build a
    Grid directly as a regular lattice in [0, 1] space, so every run is identical
    and offline. Pass --real to benchmark against cached real OSM data instead.
  * Shared grid. The street grid is built once and reused for every shape, so the
    per-shape timings isolate placement search + routing (grid build is reported
    separately, once).

Usage:
    python benchmark.py                 # synthetic grid, all shapes/*.svg
    python benchmark.py --grid-size 60  # finer synthetic lattice
    python benchmark.py --real          # real OSM (network; cached on disk)
    python benchmark.py --json          # also write benchmark_results.json
"""

import argparse
import csv
import glob
import hashlib
import json
import os
import pickle
import time

import numpy as np
from scipy.spatial import cKDTree

import gen
from gen import (
    Grid,
    dtw,
    extract_contour,
    format_distance,
    frechet,
    hausdorff,
    iou,
    land_fraction,
    perceptual_cost,
    route_length_m,
    search_placement,
    turning_distance,
)

HERE = os.path.dirname(os.path.abspath(__file__))
SHAPES_DIR = os.path.join(HERE, "shapes")
CACHE_DIR = os.path.join(HERE, ".cache", "grids")


# --------------------------------------------------------------------------- #
# Synthetic street grid                                                        #
# --------------------------------------------------------------------------- #
def synthetic_grid(cfg):
    """Build a Grid as a regular street lattice in [0, 1] space (no network).

    Produces exactly the same fields build_grid() does, so every downstream
    function (snap_waypoints, route_pair, the metrics) works unchanged. The
    lattice is `size` x `size` nodes with 4-neighbour links plus diagonals, which
    gives the router both axis-aligned and ~45 deg moves -- the two angles the
    placement search's orientation term considers "clean".
    """
    size = cfg["grid_size"]
    s = 1.0 / (size - 1)                         # node pitch in [0, 1] units

    def key(i, j):
        return (round(i * s, 6), round(j * s, 6))

    graph, edge_list = {}, []

    def link(a, b):
        d = float(np.hypot(a[0] - b[0], a[1] - b[1]))
        graph.setdefault(a, []).append((b, d))
        graph.setdefault(b, []).append((a, d))
        edge_list.append((a, b))

    for i in range(size):
        for j in range(size):
            graph.setdefault(key(i, j), [])
    for i in range(size):
        for j in range(size):
            a = key(i, j)
            if i + 1 < size:
                link(a, key(i + 1, j))
            if j + 1 < size:
                link(a, key(i, j + 1))
            if cfg["grid_diagonals"]:
                if i + 1 < size and j + 1 < size:
                    link(a, key(i + 1, j + 1))
                if i + 1 < size and j - 1 >= 0:
                    link(a, key(i + 1, j - 1))

    avg_edge = float(np.mean([d for nbrs in graph.values() for _, d in nbrs]))
    node_keys = list(graph.keys())
    nodes_arr = np.array(node_keys, dtype=np.float64)
    span = float(2 * cfg["radius_m"])            # metres spanned by the [0, 1] box
    return Grid(graph, node_keys, nodes_arr, cKDTree(nodes_arr),
                avg_edge, edge_list, span, 0.0, 0.0, 0.0)


# --------------------------------------------------------------------------- #
# Real OSM grid, cached on disk                                                #
# --------------------------------------------------------------------------- #
def real_grid_cached(cfg):
    """gen.build_grid() wrapped in a pickle cache keyed by the fetch params.

    A Grid (including its cKDTree) pickles cleanly, so the first run fetches +
    normalizes the network and later runs load it back in a moment.
    """
    os.makedirs(CACHE_DIR, exist_ok=True)
    sig = (cfg["lat"], cfg["lng"], cfg["radius_m"], cfg["node_spacing"],
           cfg["include_parks"], cfg["densify_streets"])
    h = hashlib.md5(repr(sig).encode()).hexdigest()[:12]
    path = os.path.join(CACHE_DIR, f"grid_{h}.pkl")
    if os.path.exists(path):
        with open(path, "rb") as f:
            print(f"loaded cached grid: {path}")
            return pickle.load(f)
    grid = gen.build_grid(cfg)
    with open(path, "wb") as f:
        pickle.dump(grid, f)
    return grid


# --------------------------------------------------------------------------- #
# Benchmark                                                                    #
# --------------------------------------------------------------------------- #
def discover_cases(shapes_dir):
    """One case per shapes/*.svg, named after the file stem."""
    cases = []
    for svg in sorted(glob.glob(os.path.join(shapes_dir, "*.svg"))):
        cases.append({"name": os.path.splitext(os.path.basename(svg))[0], "svg": svg})
    return cases


def run_case(grid, case, cfg):
    """Time + score one shape. Returns a flat dict of metrics for the table."""
    t0 = time.perf_counter()
    contour = extract_contour(case["svg"], cfg["img_size"])
    t_contour = time.perf_counter() - t0

    t1 = time.perf_counter()
    ranked = search_placement(contour, grid, {**cfg, "svg_path": case["svg"]})
    t_search = time.perf_counter() - t1

    cost, placed, route = ranked[0]
    if len(route) < 2:
        return {"name": case["name"], "nodes": len(route), "cost": cost,
                "frechet": float("nan"), "hausdorff": float("nan"), "iou": 0.0,
                "perceptual": float("nan"), "dtw": float("nan"),
                "turning": float("nan"), "on_land": 0.0, "length_m": 0.0,
                "t_contour": t_contour, "t_search": t_search}

    return {
        "name": case["name"],
        "nodes": len(route),
        "cost": float(cost),
        "frechet": frechet(route, placed),
        "hausdorff": hausdorff(route, placed),
        "iou": iou(route, placed, 0.01),
        "perceptual": perceptual_cost(route, placed),
        "dtw": dtw(route, placed),
        "turning": turning_distance(route, placed),
        "on_land": land_fraction(placed, grid) * 100.0,
        "length_m": route_length_m(route, grid),
        "t_contour": t_contour,
        "t_search": t_search,
    }


# (key, header label, column width, value formatter). Width covers the wider of
# the label and the formatted value so the header and rows line up.
COLUMNS = [
    ("name", "shape", 10, lambda v: f"{v:<10}"),
    ("nodes", "nodes", 6, lambda v: f"{v:>6d}"),
    ("frechet", "Frechet", 9, lambda v: f"{v:>9.4f}"),
    ("hausdorff", "Hausdf", 8, lambda v: f"{v:>8.4f}"),
    ("iou", "IoU", 6, lambda v: f"{v:>6.3f}"),
    ("perceptual", "percept", 8, lambda v: f"{v:>8.4f}"),
    ("dtw", "DTW", 8, lambda v: f"{v:>8.4f}"),
    ("turning", "turning", 8, lambda v: f"{v:>8.4f}"),
    ("cost", "cost", 8, lambda v: f"{v:>8.4f}"),
    ("on_land", "land%", 6, lambda v: f"{v:>6.0f}"),
    ("length_m", "dist", 16, lambda v: f"{v:>16}"),
    ("t_route", "t_route", 8, lambda v: f"{v:>8.2f}"),
]


def print_table(results):
    header = "  ".join(f"{label:>{w}}" for _, label, w, _ in COLUMNS)
    print("\n" + header)
    print("-" * len(header))
    for r in results:
        row = dict(r)
        row["length_m"] = format_distance(r["length_m"])
        row["t_route"] = r["t_search"]
        cells = []
        for key, _, w, fmt in COLUMNS:
            try:
                cells.append(fmt(row[key]))
            except (ValueError, TypeError):
                cells.append(f"{'-':>{w}}")
        print("  ".join(cells))
    print("-" * len(header))
    _print_summary(results)


def _print_summary(results):
    ok = [r for r in results if r["nodes"] >= 2 and np.isfinite(r["frechet"])]
    if not ok:
        print("summary: no routable cases")
        return
    mean = lambda k: float(np.mean([r[k] for r in ok]))
    print(f"mean over {len(ok)} cases:  "
          f"Frechet={mean('frechet'):.4f}  Hausdorff={mean('hausdorff'):.4f}  "
          f"IoU={mean('iou'):.3f}  perceptual={mean('perceptual'):.4f}  "
          f"DTW={mean('dtw'):.4f}  turning={mean('turning'):.4f}  "
          f"t_route={mean('t_search'):.2f}s")


def write_csv(results, path):
    fields = ["name", "nodes", "frechet", "hausdorff", "iou", "perceptual",
              "dtw", "turning", "cost", "on_land", "length_m",
              "t_contour", "t_search"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in results:
            w.writerow({k: r[k] for k in fields})
    print(f"wrote {path}")


def main():
    ap = argparse.ArgumentParser(description="Shape-fidelity benchmark.")
    ap.add_argument("--real", action="store_true",
                    help="benchmark against real OSM data (network; cached) "
                         "instead of the synthetic lattice")
    ap.add_argument("--grid-size", type=int, default=50,
                    help="synthetic lattice resolution (nodes per side)")
    ap.add_argument("--no-diagonals", action="store_true",
                    help="synthetic lattice without 45 deg links")
    ap.add_argument("--shapes", default=SHAPES_DIR, help="directory of *.svg shapes")
    ap.add_argument("--out", default=os.path.join(HERE, "benchmark_results.csv"),
                    help="CSV output path")
    ap.add_argument("--json", action="store_true", help="also write JSON results")
    args = ap.parse_args()

    cfg = dict(gen.CONFIG)
    cfg.update(grid_size=args.grid_size, grid_diagonals=not args.no_diagonals)

    cases = discover_cases(args.shapes)
    if not cases:
        raise SystemExit(f"no SVG shapes found in {args.shapes}")
    print(f"benchmarking {len(cases)} shapes: "
          f"{', '.join(c['name'] for c in cases)}")

    t = time.perf_counter()
    grid = real_grid_cached(cfg) if args.real else synthetic_grid(cfg)
    t_grid = time.perf_counter() - t
    kind = "real OSM" if args.real else f"synthetic {args.grid_size}x{args.grid_size}"
    print(f"grid ({kind}): {len(grid.node_keys)} nodes, avg edge "
          f"{grid.avg_edge:.4f}, built in {t_grid:.2f}s")

    results = []
    for case in cases:
        print(f"\n=== {case['name']} ===")
        results.append(run_case(grid, case, cfg))

    print_table(results)
    write_csv(results, args.out)
    if args.json:
        jpath = os.path.splitext(args.out)[0] + ".json"
        with open(jpath, "w") as f:
            json.dump({"grid": kind, "grid_build_s": t_grid, "results": results},
                      f, indent=2)
        print(f"wrote {jpath}")


if __name__ == "__main__":
    main()
