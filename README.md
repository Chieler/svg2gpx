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

### Inner features

Shapes are more than their outer silhouette. `extract_shape()` in `gen.py` also
finds the SVG's **inner features**, classified from the raster's ink/paper contour
tree, whether or not they touch the outer contour:

- **holes** — a donut's hole, a shark's eye (closed loops);
- **disconnected inner elements** — a smiley's eyes and mouth, drawn entirely
  separate from the outer ring (closed loops);
- **interior detail strokes** in line art — a wing line, a horse's mane — found as
  the arcs where an interior region's boundary deviates from the outline it shadows
  (open paths, runnable as out-and-back spurs).

`extract_contour()` still returns just the outer outline, so existing callers are
unchanged. Visual check: `python preview_features.py` (writes
`inner_features_preview.png`); correctness checks: `python test_inner_features.py`.

The features are then **placed and routed** with the outer contour: every feature
gets the exact affine chosen for the outline (scale, rotation, aspect, shear —
about the same pivot), closed features route as their own loops and open ones as
out-and-back spurs (`build_feature_route`). And because features are only useful
when the contour is good, the placement search's second stage — the top
`n_route_eval` routed candidates — folds each candidate's **feature fidelity into
its cost** (`feature_cost`): the per-feature routed deviation (1.0 for a feature
that is off-grid or below street resolution), size-weighted and bounded by the
features' share of total drawn length, scaled by `inner_cost_weight`. A placement
whose contour seats nicely but strands the eye now ranks below one that draws both.

Small features get two extra rescues, since an eye drawn at 2% of the shape spans
barely a city block:

- **feature-scaled smoothing** — cleanup's corner-cut slack is sized for the outer
  shape and would legally shortcut a small eye into a triangle; feature routing
  raises the effective granularity until the slack is a small fraction of the
  feature's own span;
- **street-tailored refinement** (`refine_feature`, `inner_refine`) — each feature
  is re-seated on the street fabric around its drawn spot, mirroring the outer
  two-stage search in miniature: candidate positions are the drawn centroid plus
  every street node within `inner_search_radius`× the feature's span, tried at
  small rotations (±`inner_rot_deg`) and a ladder of scales including a rescue
  upscale for features narrower than `inner_min_span_blocks` street edges (capped
  `inner_max_inflate`×). A cheap street-fit proxy — snap closeness, coverage, and
  how many *distinct* nodes the outline resolves to (tiny features die by
  collapsing onto one node) — ranks the variants; the best `inner_route_eval` are
  actually routed (the drawn identity and the best rescue variant are always
  among them), and the winner minimizes routed deviation plus a drift penalty
  that anchors the feature to where the drawing put it. A bigger or moved target
  must pay its own way, so useless inflation and wandering lose.

The stage-1 placement proxy also carries a feature term (`inner_proxy_weight`):
a few sample points per feature are placed with each of the thousands of trial
transforms, and their snap closeness / node resolvability bias the ranking — so
feature-friendly placements survive into stage 2 in the first place.

Toggle everything with `inner_features=False` in CONFIG or `--no-inner-features`
on `gen.py` / `chicago_map.py`.

### Estimating routing quality before routing

A bad placement dooms the route from the start: routing is expensive, so stage 1
ranks thousands of placements with a cheap geometric proxy (`_score`) and only
the top few are actually routed. The catch is that every classic proxy term —
snap distance, coverage, feature reach, grid-angle orientation — is evaluated
**at the outline points**, while routing walks the streets **between** them. A
placement whose every vertex snaps perfectly can still have a long edge spanning
a river, rail cut, highway or superblock with no through-streets, and the route
detours badly around it.

So the proxy carries a **between-anchor clearance** term (`placement_void_weight`):
it samples the *midpoints* of a coarsened outline and rewards placements whose
spans stay near the network. It's near-constant on a void-free grid (can't hurt
where it doesn't apply) but on real OSM it moves the best-of-top-6 routed cost
most of the way to the routed optimum — e.g. on the Chicago window, Shark
0.217 → 0.148 (oracle 0.145), lshape 0.157 → 0.139, with no shape regressing.
This was chosen by measuring rank-correlation of candidate cheap terms against
true routed cost; simpler ideas (nearest-node "snap-jump", local street-direction
availability) flipped sign across shapes and were dropped.

### Routing robustness

Waypoints that fall in a disconnected pocket of the street graph (a clipped
component, a park-mesh island) no longer produce straight-line "teleports": the
router bridges to the reachable node nearest the target (closest approach) and
continues from wherever the route actually ended, so every produced route is a
connected walk on real street edges (`python test_routing.py` verifies this).

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
