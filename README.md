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

The route is scored against the target outline with complementary measures (all in `gen.py`),
each catching a failure the others miss:

- **Fréchet** — order-aware *worst-case* leash; punishes out-of-sequence detours that pointwise metrics miss.
- **Hausdorff** — worst-case point-to-curve distance (single largest excursion).
- **IoU** — area overlap of the two thickened outlines.
- **Perceptual cost** — blur-tolerant render-and-compare (`1 − soft-IoU`); judges the overall gestalt the way the eye does.
- **DTW** — cyclic Dynamic Time Warping: like Fréchet but *sums* the leash over the best alignment instead of taking its max, so it rewards hugging the outline everywhere rather than only at the worst point. Tries cyclic start offsets and both winding directions, since the loops have no canonical start.
- **Turning distance** — compares the curves' *turning-angle vs. arc-length* signatures: a translation/rotation/scale-invariant measure of **form** (corners, protrusions) that is sharp about real features yet ignores staircase jitter.
- **On-land %** and **distance** — sanity/runnability checks.

Reading them together tells you *how* a result is good or bad: Fréchet/DTW for path order
(worst vs. average), Hausdorff for a single bad excursion, IoU for overall area, and turning
distance for whether the corners and protrusions land in the right places.

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
python gen.py                                        # CONFIG defaults (Manhattan, star)
python gen.py --svg shapes/Crow.svg --granularity 0.8
python gen.py --lat 41.9285 --lng -87.7075 --save route.png --no-show
```

Every common knob is a CLI flag (`--svg`, `--lat/--lng/--radius`, `--granularity`,
`--graphml`, `--seed`, `--save`, `--no-show`); everything else is tuned from the
`CONFIG` dict in `gen.py`. `--graphml` loads a saved `ox.save_graphml` network
instead of fetching from the Overpass API, for offline / reproducible runs.

### Trace the shapes on the real Chicago street map

[`chicago_map.py`](chicago_map.py) runs every bundled shape on the **real Chicago
street network** (an OpenStreetMap citywide snapshot, downloaded once into `data/`)
and renders each route on the actual OSMnx map, plus a gallery image, per-shape
GeoJSON (WGS84) and a metrics CSV in `chicago_maps/`.

```bash
python chicago_map.py                    # all shapes, Logan Square window
python chicago_map.py --shape star       # one shape
python chicago_map.py --live             # fetch fresh OSM data instead
```

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
