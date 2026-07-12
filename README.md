# svg2gpx — SVG to GPS art running route generator

> Turn any **SVG** shape into a runnable **GPS art** route that traces the figure on
> real city streets — the automatic, open-source way to make **Strava art** without
> drawing it by hand.

**svg2gpx** takes an SVG silhouette and a location, lays the shape over a city's
walkable street network, and generates a single **closed running route** whose path
resembles the shape — a boar, a star, a heart — drawn in streets you can actually run
or ride. Unlike the hand-draw GPS art planners, it fits and routes the shape
**automatically**, and it ships a **fidelity engine** that measures how faithfully the
route reproduces your shape — so quality is a number you can track and tune, not just
something you eyeball.

Routes export as GeoJSON (WGS84) plus map images today; a one-hop conversion to **GPX**
drops them straight into **Strava**, **Garmin**, or **Komoot**.

<sub>*Topics: GPS art · Strava art generator · GPS drawing · SVG to GPX · SVG to route ·
running route art · GPX route maker · OpenStreetMap · running · cycling.*</sub>

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


## Future Improvements:

Idea A: Turning-Penalized "Snake Mode" (State-Space Dijkstra)
Value for gen.py: Extremely High. * The Problem in gen.py: Currently, route_pair runs a standard node-based Dijkstra loop where edge costs are computed as edge_length + weight * distance_to_outline(neighbor). Because the state space is tracked strictly by the current node u, the graph expansion is blind to momentum. If a tiny zig-zag (a comb tooth) shaves off a fraction of a geometric unit of deviation, Dijkstra will eagerly take it because it cannot factor in the fact that it is executing two rapid $90^\circ$ turns.Implementation 
Shift: Rewrite route_pair to track the state space as a tuple of (current_node, incoming_edge_id) or (current_node, parent_node). This breaks the strict Bellman criteria constraints by preserving directional history, allowing you to add a massive penalty to the cost accumulator if the angle change between the incoming edge and candidate outgoing edge represents an immediate lateral alternation.
turn penalty should not be absolute. Instead of a raw penalty for any turn, it should be a relative angular deviation penalty: penalize the difference between the change in the street angle and the change in the contour angle. If the contour turns $90^\circ$ and the street turns $90^\circ$, the penalty is $0$. If the contour goes straight and the street turns $90^\circ$, apply the full penalty.

Idea B: Morphological Skeletonization & Component-Based Warping
Value for gen.py: Medium-High (Great for Protrusions).The Problem in gen.py: search_placement applies an excellent global affine transform matrix (handling scale, rotation, aspect ratio, shear, and flipping) to fit the whole shape onto the city grid. However, when a shape has narrow, rigid appendages running diagonally across a grid layout, a global transform cannot satisfy both the main mass and the limb simultaneously. This causes the dense waypoints along that limb to map awkwardly, forcing a combed path.Implementation
Shift: In extract_shape, add a morphological skeletonization pass on the binary ink mask binary before tracing vertices. Segment the shape into a "core body" and "appendage chains." Instead of a uniform place() function, you can allow appendages to dynamically bend or locally align their internal vectors to mirror the dominant local street orientation (grid.grid_angle).
   i.e. affine towards secondary/dominant street angle / affine on the fly
Idea C: Dedicated Post-Processing Combing Filter
Value for gen.py: High (Low-Risk, Immediate Payoff).The Problem in gen.py: gen.py a
dissolve_oscillations(). This filter ignores spatial distance to the shape temporarily and analyzes the discrete turn signature of the route sequence (e.g., looking for a pattern of [+90°, -90°, +90°, -90°] within 4–6 nodes). When detected, it forces a bypass to collapse the tooth into a monotone staircase line, verifying afterwards that it doesn't violate a hard threshold.

New Structural Changes for gen.py (Unexplored So Far)
1: Soft Viterbi/HMM Trellis Routing (Replacing Strict Node-Chaining)The Vulnerability: route_contour currently marches sequentially through a single array of snapped node anchors (waypoints). If snap_waypoints selects even one bad node due to local grid anomalies, the path is structurally compromised because Dijkstra must visit that exact node before moving to the next.
The Change: Eliminate strict point-snapping entirely. For each waypoint on the densified contour, use grid.tree.query_ball_point to gather a set of candidate nodes within a small radius (e.g., 3 nearest intersections). Build an HMM/Viterbi trellis across these sets. The transition cost is the true shortest-path distance between nodes in layer $i-1$ and layer $i$, while the emission cost is the node's proximity to the contour.Why it helps: This completely decouples the routing path from arbitrary, local snapping errors. The network finds the globally smoothest corridor through the street graph that matches the contour sequence, naturally absorbing or discarding awkward side-streets.


Change 2: Precomputed Static Edge-Cost MapsThe Vulnerability: In route_pair, the inner Dijkstra loop calls a closure function returned by _polyline_dist_fn(seg) for every neighbor expanded. This function dynamically calculates the minimum distance from a point coordinate to a line segment using vector operations (np.einsum, np.clip). Calling this on every neighbor expansion across long paths is highly CPU-bound and limits performance.The Change: Move from dynamic segment calculation to a static graph decoration phase. Once search_placement settles on a final placement vector, execute a single vectorized calculation across all nodes/edges in the bounding box relative to the target polyline. Cache a scalar contour_dist directly onto the grid.graph adjacency entries.Why it helps: Reduces the inner loop of Dijkstra to a trivial $O(1)$ lookup (cost = edge_length + weight * nbr_static_dist). This increases routing execution speeds, allowing you to scale up parameters like n_route_eval or run much denser grids without bottlenecking performance.
Change 3: Directional Velocity Pruning (Enforcing Monotonicity)The Vulnerability: Combing occurs because a route moves backwards or sideways relative to the overall progression of the shape to satisfy a micro-distance advantage.The Change: Compute a forward progression vector $\vec{V}_{target}$ for the current segment of the contour. When expanding neighbors in route_pair, calculate the directional vector of the candidate street edge $\vec{V}_{edge} = \text{neighbor} - \text{current}$. Compute the dot product:$$\vec{V}_{target} \cdot \vec{V}_{edge}$$If the dot product is negative (meaning the street edge forces the runner to travel backwards relative to the shape's vector), apply a massive multiplier penalty to that edge, or prune it from expansion completely.
Why it helps: This mathematically guarantees monotonicity along a shape's face. It permits the graph to step outward and forward (staircasing), but bans it from darting inward and backward (combing), blocking high-frequency oscillations before they can form.
Change 4: Graph Topology Coarsening (Virtual Edge Bundling)The Vulnerability: Because _densify_edge breaks long street links into chains of small sub-blocks to catch fine shape adjustments, it inflates the graph's node depth. This massive node count allows Dijkstra to make micro-jogs down side alleys mid-block.The Change: Keep a dual-graph representation. Perform routing on the simplified structural intersection graph (where edges only exist between actual road junctions) using an edge-integral cost function, rather than routing on micro-segmented nodes.Why it helps: If the router can only make decisions at true physical street intersections, it structurally cannot create micro-comb teeth in the middle of a city block, forcing clean, uniform lines from corner to corner.




1. Look-Ahead Junction Deceleration (Variable Momentum Weight)

Instead of a static TURN_WEIGHT throughout the entire route, you should scale the momentum penalty based on the local curvature of the target SVG.

    Compute the turning angle of the SVG template at every waypoint.

    Where the SVG is a long straight line (the shark's belly), crank the momentum weight up to maximum to completely suppress combing.

    When the SVG approaches a sharp corner or an appendage junction (the base of a fin), dynamically drop the momentum weight to near zero.

    This signals to the Dijkstra router: "An intentional structural corner is coming up; you are allowed to slow down and make a sharp, pivoting turn here without penalty."


2. Anchor-Free "Jump Fields" at Junctions

Instead of forcing the path to route sequentially through every single waypoint at a junction, adapt route_contour to allow multipath look-aheads near complex transitions. If the router detects that stepping into a narrow street pocket creates an unrunnable nook, it should have the structural freedom to "leap" across the junction base to a node on the appendage's main stem, trading a minor piece of interior area fidelity for massive topological stability.
