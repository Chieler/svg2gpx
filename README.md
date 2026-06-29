# Spatial Shape Fidelity Mapping on Street Networks

> Turn an SVG into a runnable city route whose outline traces the shape on real streets.

Given an SVG silhouette and a location, this project lays the shape over a city's
walkable street network and builds a single **closed running route** whose path
resembles the shape — a boar, a star, a heart — drawn in streets you can actually run.

It then **measures how faithfully** the route reproduces the shape, so fidelity is a
number you can track and tune, not just something you eyeball.

---

## How it works

The pipeline (`gen.py`) runs end to end:

| Stage | Function | What it does |
| --- | --- | --- |
| 1. Build grid | `build_grid` | Pull and normalize the walkable street network (and parks) into `[0, 1]` space. |
| 2. Extract contour | `extract_contour` | Render the SVG and trace its outer outline as a closed polyline. |
| 3. Search placement | `search_placement` | Find the scale / rotation / offset / stretch that seats the shape on the streets with the best routed fidelity. |
| 4. Snap waypoints | `snap_waypoints` | Densify the placed outline and snap points to street nodes — dense anchors so each hop barely deviates. |
| 5. Route | `route_contour` | Walk consecutive anchors with a contour-biased Dijkstra so the path hugs the shape. |
| 6. Cleanup + plot | `cleanup`, `plot` | Close the loop, dissolve backtracks / loops / nooks, report fidelity, draw. |

Fidelity comes from **dense waypoints**: spacing anchors well below one block means each
Dijkstra hop is short and has little room to stray from the outline.

### Fidelity metrics

The route is scored against the target outline with complementary measures (all in `gen.py`):

- **Fréchet** — respects order; punishes out-of-sequence detours that pointwise metrics miss.
- **Hausdorff** — worst-case point-to-curve distance.
- **IoU** — area overlap of the two thickened outlines.
- **Perceptual cost** — blur-tolerant render-and-compare (`1 − soft-IoU`); judges the overall gestalt the way the eye does.
- **On-land %** and **distance** — sanity/runnability checks.

---

## Installation

```bash
pip install -r requirements.txt           # core: synthetic-grid runs, offline
pip install -r requirements-osm.txt        # extra: real OSM data + plotting
```

`skia-python` needs system GL libraries on Linux:

```bash
sudo apt-get install -y libegl1 libgl1
```

---

## Usage

### Generate a route

```bash
python gen.py
```

Tune everything from the `CONFIG` dict in `gen.py` — location, search budget,
granularity, and which shape (`svg_path`) to map.

### Benchmark fidelity across shapes

[`benchmark.py`](benchmark.py) runs the pipeline over every shape in [`shapes/`](shapes)
and reports **fidelity + per-stage runtime** for each, plus an aggregate summary.
It uses a fast, deterministic **synthetic street grid** by default (no network), so
runs are reproducible and CI-friendly.

```bash
python benchmark.py                  # synthetic grid, all shapes
python benchmark.py --grid-size 60   # finer lattice
python benchmark.py --real           # real OSM (network; cached on disk)
python benchmark.py --json           # also write benchmark_results.json
```

### Pick the best route per shape

[`best_route.py`](best_route.py) routes the **top 3 candidate placements** for a shape,
selects the lowest-cost (best) one, and upserts its metrics into
[`result.csv`](result.csv) — one "best route" row per shape.

```bash
python best_route.py                 # all shapes, synthetic grid
python best_route.py --shape star     # just one shape
python best_route.py --grid real      # real OSM
```

---

## Continuous fidelity tracking

The **Best Route** GitHub Action ([`.github/workflows/best-route.yml`](.github/workflows/best-route.yml))
runs `best_route.py` on demand and commits the updated `result.csv` back to the repo.

Trigger it from the **Actions** tab (`workflow_dispatch`) with inputs:

| Input | Default | Description |
| --- | --- | --- |
| `shape` | `all` | A shape file stem (e.g. `star`) or `all`. |
| `grid` | `synthetic` | `synthetic` (fast, offline) or `real` (live OSM). |
| `grid_size` | `50` | Synthetic lattice resolution (nodes per side). |

---

## Shapes

Sample SVGs live in [`shapes/`](shapes): `square`, `circle`, `lshape`, `star`.
Drop any `<polygon>` / `<path>` SVG in there and the benchmark and Action pick it up
automatically — no code changes needed.

## Repository layout

```
gen.py                       # the full SVG -> street-route pipeline + metrics
benchmark.py                 # fidelity + runtime benchmark over all shapes
best_route.py                # best-of-3 selection -> result.csv
shapes/                      # sample input SVGs
result.csv                   # latest best-route metrics (written by the Action)
requirements.txt             # core dependencies
requirements-osm.txt         # extras for real OSM + plotting
.github/workflows/           # Best Route GitHub Action
```
