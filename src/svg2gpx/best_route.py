"""
Pick the best of gen.py's top candidate routes for a shape and log it to result.csv.

For a given shape, `search_placement` (gen.py) evaluates several candidate
placements and returns them ranked best-first by the composite placement cost.
This script routes the top `N_BEST` candidates, selects the single lowest-cost
(best) one, and records its fidelity metrics as one row in `result.csv` -- the
"best route we found for this shape" record. Run it for one shape or all of
them; rows are upserted by shape name so re-running one shape updates just it.

Usage:
    python best_route.py                      # all shapes, synthetic grid
    python best_route.py --shape star         # one shape
    python best_route.py --grid real          # real OSM (network; cached)
"""

import argparse
import csv
import datetime as dt
import glob
import os

import numpy as np

from . import gen
from .gen import (
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
from .benchmark import SHAPES_DIR, real_grid_cached, synthetic_grid

# How many top-ranked candidate placements to route and choose the best from.
N_BEST = 3

FIELDS = ["shape", "grid", "candidates", "chosen_cost", "frechet", "hausdorff",
          "iou", "perceptual", "dtw", "turning", "on_land", "length_m",
          "nodes", "updated"]


def best_for_shape(grid, svg, cfg):
    """Route the top N_BEST candidate placements and return the best result.

    Returns (candidate_costs, best) where best is (cost, placed, route) and
    candidate_costs is the cost of each routed candidate (best-first).
    """
    contour = extract_contour(svg, cfg["img_size"])
    eval_cfg = {**cfg, "svg_path": svg, "n_route_eval": N_BEST, "n_options": N_BEST}
    ranked = search_placement(contour, grid, eval_cfg)   # Candidates, best-first
    candidates = ranked[:N_BEST]
    costs = [float(c.cost) for c in candidates]
    return costs, candidates[0]


def score_row(name, grid_kind, costs, best, grid):
    """Build the result.csv row for the chosen best route."""
    cost, placed, route = best.cost, best.placed, best.route
    if len(route) < 2:
        return {"shape": name, "grid": grid_kind,
                "candidates": " ".join(f"{c:.4f}" for c in costs),
                "chosen_cost": cost, "frechet": float("nan"),
                "hausdorff": float("nan"), "iou": 0.0, "perceptual": float("nan"),
                "dtw": float("nan"), "turning": float("nan"),
                "on_land": 0.0, "length_m": 0.0, "nodes": len(route),
                "updated": _now()}
    return {
        "shape": name,
        "grid": grid_kind,
        "candidates": " ".join(f"{c:.4f}" for c in costs),
        "chosen_cost": round(float(cost), 6),
        "frechet": round(frechet(route, placed), 6),
        "hausdorff": round(hausdorff(route, placed), 6),
        "iou": round(iou(route, placed, 0.01), 4),
        "perceptual": round(perceptual_cost(route, placed), 6),
        "dtw": round(dtw(route, placed), 6),
        "turning": round(turning_distance(route, placed), 6),
        "on_land": round(land_fraction(placed, grid) * 100.0, 1),
        "length_m": round(route_length_m(route, grid), 1),
        "nodes": len(route),
        "updated": _now(),
    }


def _now():
    return dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def upsert_rows(path, new_rows):
    """Merge new rows into result.csv keyed by shape name, then rewrite sorted."""
    rows = {}
    if os.path.exists(path):
        with open(path, newline="") as f:
            for r in csv.DictReader(f):
                rows[r["shape"]] = r
    for r in new_rows:
        rows[r["shape"]] = {k: r[k] for k in FIELDS}
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for name in sorted(rows):
            w.writerow(rows[name])


def main():
    ap = argparse.ArgumentParser(
        description="Select the best of gen.py's top routes per shape -> result.csv.")
    ap.add_argument("--shape", default="all",
                    help="shape file stem in shapes/ (e.g. 'star'), or 'all'")
    ap.add_argument("--grid", choices=["synthetic", "real"], default="synthetic",
                    help="street grid: offline synthetic lattice or real OSM")
    ap.add_argument("--grid-size", type=int, default=50,
                    help="synthetic lattice resolution (nodes per side)")
    ap.add_argument("--shapes", default=SHAPES_DIR, help="directory of *.svg shapes")
    ap.add_argument("--out", default="result.csv",
                    help="result CSV path")
    args = ap.parse_args()

    if args.shape == "all":
        svgs = sorted(glob.glob(os.path.join(args.shapes, "*.svg")))
    else:
        svgs = [os.path.join(args.shapes, f"{args.shape}.svg")]
    svgs = [s for s in svgs if os.path.exists(s)]
    if not svgs:
        raise SystemExit(f"no matching SVG shapes in {args.shapes}")

    cfg = dict(gen.CONFIG)
    cfg.update(grid_size=args.grid_size, grid_diagonals=True)
    grid = real_grid_cached(cfg) if args.grid == "real" else synthetic_grid(cfg)
    grid_kind = "real" if args.grid == "real" else f"synthetic-{args.grid_size}"
    print(f"grid ({grid_kind}): {len(grid.node_keys)} nodes, avg edge {grid.avg_edge:.4f}")

    new_rows = []
    for svg in svgs:
        name = os.path.splitext(os.path.basename(svg))[0]
        print(f"\n=== {name} (best of {N_BEST}) ===")
        costs, best = best_for_shape(grid, svg, cfg)
        row = score_row(name, grid_kind, costs, best, grid)
        new_rows.append(row)
        print(f"  candidates: {row['candidates']}  ->  chosen cost {row['chosen_cost']}  "
              f"(Frechet={row['frechet']}, IoU={row['iou']}, "
              f"dist={format_distance(row['length_m'])})")

    upsert_rows(args.out, new_rows)
    print(f"\nwrote best route(s) for {len(new_rows)} shape(s) -> {args.out}")


if __name__ == "__main__":
    main()
