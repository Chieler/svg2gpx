# Routing-fidelity plan (anti-combing)

Reducing **combing** — the high-frequency in-and-out oscillation the router
produces when it chases sub-block outline detail — at the *router and cleanup*
layers. This is the counterpart to [`rendering-fidelity-plan.md`](rendering-fidelity-plan.md),
which attacks the same artifact from the *input* side (Fourier low-pass matches
the target's detail to the grid's Nyquist). The two are complementary: low-pass
lowers the *demand* for combing, the changes here lower the *supply*.
**Status: Phase 0–3 implemented. Phase 0 (metric) + the Phase-3 Hausdorff fix
ship default-ON; the Phase-2 turn penalty and Phase-3 trellis ship default-OFF
as documented knobs (neither is a clean win — see below).**

## Diagnosis (measured on the synthetic 50×50 lattice, all bundled shapes)

Baseline `python benchmark.py`:

| shape | Frechet | Hausdorff | turning | IoU | failure |
| --- | --- | --- | --- | --- | --- |
| Cat | 0.205 | 0.197 | **1.21** | 0.53 | combing on legs/tail |
| Horse | 0.025 | 0.095 | **0.87** | 0.54 | combing on legs |
| star | 0.046 | 0.118 | 0.49 | 0.64 | staircased points |
| lshape | 0.012 | **0.29** | 0.21 | 0.64 | single bad excursion (snap) |
| square | 0.012 | **0.34** | 0.15 | 0.66 | single bad excursion (snap) |

Two distinct failure modes, which need different fixes:

1. **Combing / jitter** (Cat, Horse, star) — a *turning* problem: the route
   winds far more than the shape does. The position metrics barely see it
   (Frechet stays low because the teeth hug the outline).
2. **Single bad excursions** (lshape, square, Knight) — a *snapping* problem:
   low Frechet but large Hausdorff, one anchor snapped to a far/wrong node and
   the leg to it cuts across.

The prior measured work (`rendering-fidelity-plan.md`) established the governing
rule for everything below: **never validate a rendering change by a cost the
search also optimizes.** Two selection-cost experiments (a void term, a
turning-recognition term) moved metrics without a robust visual win. The one
change that measurably improved the picture was input-side FD low-pass (combing
Shark 43.9→24.8, Horse 20.5→4.4). So each anti-combing change here is judged by
an **independent combing metric + visual A/B**, not by the routing cost.

## Idea evaluation (against the actual `gen.py`) + discrepancies

| Idea (from README "Future Improvements") | Claim about the code | Accurate? | Verdict |
| --- | --- | --- | --- |
| A. Turn-penalized Dijkstra | `cost = len + weight·dist_to_outline(nbr)`, state = node only | ✅ (`gen.py` `route_pair`) | **done (Phase 2), default-OFF** |
| C. dissolve_oscillations | shortcut_nooks keeps a tooth that hugs the outline | ✅ | **done (Phase 1)** |
| Change 1. Viterbi trellis | snap picks one node per anchor; a bad node compromises the path | ✅ | **done (Phase 3), default-OFF; exposed the Hausdorff artifact** |
| Change 3. Monotonicity prune | combing = backward motion for micro-gain | plausible | fold into A as a *soft* term |
| Change 2. Static cost maps | closure called on *every neighbor expansion* | ❌ it is **memoized per node** (`dev` dict) | defer (perf, not a bottleneck) |
| Change 4. Graph coarsening | `_densify_edge` inflates node depth | ❌ **no-op on the benchmark** (synthetic grid isn't densified) | defer (fights the design) |
| B. Skeleton + component warp | affine can't fit body + limb; align limbs to `grid.grid_angle` | ⚠️ partly | highest risk, prototype apart |
| 1. Variable momentum weight | "instead of a static TURN_WEIGHT…" | ❌ **no TURN_WEIGHT exists** | it's a modulation *of* A |
| 2. Jump fields at junctions | leap across bad pockets | — | **done — subsumed by the Change 1 trellis** |

Concrete discrepancies to keep in mind when implementing:

- **D1 — there is no turn penalty today.** The only cost term is
  `deviation_weight` (`route_pair`: `nc = cost + w + weight * dv`). Idea A
  *creates* the turn penalty; "Look-Ahead / variable momentum weight" is a
  *modulation* of that same term, not a separate feature. The curvature signal
  it needs already exists as `waypoint_importance` (≈0 on flats, ≈1 at corners).
- **D2 — `route_pair` already memoizes the outline distance per node** (`dev`
  dict), so Change 2's premise is wrong; it is computed once per node reached,
  not per expansion. Worse, that distance is to the **local arc `seg`**, not the
  whole contour — the locality is load-bearing (it keeps the route in sequence).
  A precomputed global `contour_dist` would let the route hug the wrong part of
  the outline. Change 2 is a behavior change dressed as an optimization; perf is
  ~2.6 s/shape, not a bottleneck.
- **D3 — `grid_angle` is a single global scalar**, used only in placement
  scoring, never in routing. Idea B's "local street orientation" field does not
  exist.
- **D4 — the pipeline assumes ONE placed contour.** `snap_waypoints`,
  `route_contour`, `cleanup`'s `contour_tree`, and every fidelity metric take a
  single outline array. Idea B (piecewise warp) and Idea 2 (jump fields) break
  this unless the warped pieces are stitched back into one polyline first.
- **D5 — the benchmark grid is not densified.** `synthetic_grid` is one node per
  lattice point; `_densify_edge` runs only on real OSM. Change 4 is a no-op on
  the benchmark and contradicts the measured finding that intersection density
  (~40 blocks/shape), not node count, is the ceiling.
- **D7 — a comb tooth and a staircase both alternate turn sign.** The literal
  `[+90,-90,+90,-90]` signature also matches a *legitimate* diagonal staircase.
  The discriminator is progress: a staircase makes monotone forward progress
  (its straightened bypass is the *same* length on a lattice), a comb tooth
  doubles back (its bypass is materially *shorter*). Phase 1 keys on that.

## Phased plan

### Phase 0 — combing metric (done)
`excess_turning(route, contour)` in `gen.py`: total absolute turning of the
route minus the target's. A clean loop tracks the shape's own winding (~2π for a
convex blob); combing and staircase add turning the shape never asked for, so
the surplus isolates the artifact from real form — exactly the quantity the FD
A/B reported. Wired into `benchmark.py` as an `excess` column so every change
below has an honest, search-independent yardstick.

### Phase 1 — dissolve_oscillations post-process (done)
A new pass in `cleanup()`. Detects runs of rapidly **alternating-sign, sharp**
turns and bypasses each with the plain shortest path (no contour bias, so it
straightens rather than re-tracing the tooth). The bypass is taken only when it
is **materially shorter** than the arc it replaces (the tooth doubled back — a
monotone staircase is not shortened on a lattice, so it survives, per D7) **and**
stays within a **hard deviation cap** (`protrusion_tolerance × avg_edge`, the
same threshold the rest of the file uses to separate artifact detours from real
protrusions — so a genuine beak/leg, whose bypass strays past the cap, survives).
Toggle: `dissolve_oscillations` in CONFIG. Isolated corners (star tips) are one
turn, not an alternating run, so they are never selected.

**Honest scope.** On the *synthetic lattice benchmark* this pass is a near-no-op:
that grid is uniform, so its artifact is the *staircase* rendering of shallow
edges, which the guards correctly refuse to touch (its shortest-path bypass is
the same length), and any true teeth are already gone by the time it runs (after
`remove_backtracks` / `collapse_loops` / `shortcut_nooks`). Its target is the
**real-OSM** failure the idea describes — a waypoint snapped into a side nook the
outline dips toward, producing a doubling-back tooth that `shortcut_nooks`
preserves because it hugs the outline. That case can't be exercised here
(Overpass is blocked), so the mechanism is locked by a unit test in
`test_routing.py` (tooth collapses; faithful staircase preserved; output stays a
connected walk) rather than by a benchmark delta.

### Phase 2 — relative turn penalty in `route_pair` (done; ships default-OFF)
Implemented. The router state is lifted to `(node, parent)` and the step cost
gains `turn_weight · max(0, street_turn − contour_turn) · avg_edge`: a turn is
free when the *shape* turns there (contour_turn high) and penalized when only the
street does (a comb tooth off a straight belly). The "Look-Ahead deceleration"
modulation falls out of the `contour_turn` subtraction — no separate schedule —
and corners, being at anchor boundaries between independent legs, are never
penalized. It is **decoupled from placement selection**: winners are ranked on
the `turn_weight=0` route (unchanged, validated), then the returned route is
re-solved with the penalty, so it only changes the *rendering*, never which
placement wins. `turn_weight=0` dispatches to the original bare-node Dijkstra
byte-for-byte (locked by a unit test).

**Measured tradeoff (why it's OFF by default).** Held to a fixed placement, the
penalty reliably lowers `excess` on organic shapes — Cat 35→30 (Frechet
unchanged), Knight 24→18, Crow 5→3, Shark 1.0→0.7 — but always at a small IoU
cost, because on a grid "fewer turns" means a straighter path that deviates more
from a curvy outline. And on a lattice it **cannot separate a comb tooth from the
staircase approaching a sharp tip** (both are ~45–90° turns — the same D7 problem,
now on the cost side), so at the strength that helps organic shapes it rounds
pointy corners: star Frechet 0.046→0.070, IoU 0.64→0.58; lshape IoU 0.64→0.61.
No single `turn_weight` lowers combing *and* respects "don't lower IoU on pointy
shapes." So, exactly like the repo's `recognition_weight`, it ships correct,
decoupled, and **default-OFF** (`turn_weight=0.0`), a documented knob rather than
a default. This is the third experiment (after the void and recognition terms) to
confirm the FD-plan's thesis: **router/selection-side changes move the target
metric but aren't a clean visual win; the input-side FD low-pass is.** The two
compose — the penalty's corner-rounding is largely moot on an FD-low-passed
target, which has already shed the sub-block detail the penalty fights — so the
natural next step is to gate `turn_weight` on (low corner-content ∧ FD-low-pass
applied).

### Phase 3 — Viterbi/trellis snap + route (done; ships default-OFF)
Implemented. `route_contour_trellis` gives each anchor its `trellis_k` nearest
candidate nodes and picks the globally cheapest sequence with an exact
first-order DP (`_trellis_dp`): transition = the contour-biased `route_pair` cost
(length + `weight`·deviation, so legs *hug* the outline, not just stay short),
emission = `trellis_emit_weight`·node-to-anchor distance. Because the objective is
additive and first-order it satisfies Bellman, and the single-snap path is one
lattice path so the DP is never worse *on that surrogate*. It subsumes Idea 2
(it "jumps" a bad side-pocket by preferring a neighbour) and threads `turn_weight`
through the legs (couples Phase 2). Decoupled from selection and locked by a unit
test (closed connected walk; `trellis_k=1` degenerates to single-snap).

**Two findings, one clean win and one honest null.**

- **The "big-Hausdorff excursion" failure mode did not exist — it was a metric
  artifact.** `hausdorff` compared the route against the *raw* `placed` polyline,
  which under `CHAIN_APPROX_SIMPLE` is only a square's 4 corners / an L's 8 — so a
  perfectly-traced mid-edge point read as ~half an edge (up to ~12 avg-edges) from
  the nearest *vertex*, a phantom excursion. Densifying inside `hausdorff` (it now
  resamples both curves, like `frechet`/`dtw`) collapses square 0.29→0.02, lshape
  0.29→0.02: **the routes were near-perfect all along.** This is the actual clean
  win of the phase — a trustworthy Hausdorff column — and it ships default-ON.
- **The trellis itself is not a clean win → default-OFF.** With the corrected
  metric, full-pipeline A/B is `haus 0.036→0.040, frechet 0.038→0.043,
  iou 0.643→0.646, excess 17.1→16.6`: it helps some shapes (face IoU 0.67→0.70,
  star Hausdorff 0.054→0.043) and hurts others (Crow/Knight Frechet), net slightly
  negative on the order-aware metrics. This is precisely the Bellman caveat made
  concrete: **exact on the additive surrogate ≠ better true fidelity**, because
  the surrogate (length + deviation + emission) doesn't track Frechet. So, like
  `turn_weight`, it ships correct, decoupled, unit-tested, and **default-OFF**
  (`trellis=False`), a knob rather than a default.

This is the fourth experiment to land on the same verdict (after the void term,
the recognition term, and the turn penalty): **router/selection-side objectives
move their own metric but are not a clean visual win; the input-side FD low-pass
is.** The trellis's real payoff would need a *better transition cost* — one that
tracks perceptual/turning fidelity rather than raw length+deviation — and/or a
node-pair (second-order) state carrying the anchor-boundary turn, which is the
BPO-correct way to fold `turn_weight` across legs. Both are future work.

### Deferred / prototype-behind-a-flag
Change 2 (perf-only, semantics-changing, not a bottleneck). Idea B + Change 4
(break the single-contour + dense-waypoint invariants, need a new `skimage`
dep, help only real-OSM appendage cases) — prototype on real data against the
Phase-0 metric before touching mainline. Pair the effort with the
already-measured FD low-pass, the input-side complement.

## Validation
Phase-0 `excess` column + matched-scale visual A/B. A change is kept only if it
lowers `excess` (and `turning`) **without** raising `Frechet`/lowering `IoU` on
the pointy shapes (star, lshape, square), which have the most to lose from
over-smoothing.
