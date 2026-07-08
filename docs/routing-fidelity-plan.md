# Routing-fidelity plan (anti-combing)

Reducing **combing** — the high-frequency in-and-out oscillation the router
produces when it chases sub-block outline detail — at the *router and cleanup*
layers. This is the counterpart to [`rendering-fidelity-plan.md`](rendering-fidelity-plan.md),
which attacks the same artifact from the *input* side (Fourier low-pass matches
the target's detail to the grid's Nyquist). The two are complementary: low-pass
lowers the *demand* for combing, the changes here lower the *supply*.
**Status: Phase 0–4 implemented. Grid-independent wins (default-ON): Phase 0
(the `excess` metric) and the Phase-3 Hausdorff fix. Situational knobs: the
Phase-2 turn penalty and Phase-3 trellis (default-OFF), and the Phase-4 Fourier
low-pass (default-ON, now with an **elongation-aware gate** — a real-grid
per-family check showed it was rounding compact shapes into blobs, so it now fires
only on long-edged shapes and is a no-op on compact ones, byte-identical to the
sharp engine there).**

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

### Phase 4 — Fourier low-pass of the target (done; the clean win, ships default-ON)
The input-side lever from `rendering-fidelity-plan.md`, finally wired in and
measured on this benchmark. `fourier_lowpass` represents the closed outline as
`z(t)=x(t)+i·y(t)`, keeps the first `fd_harmonics` (=20, the grid Nyquist), and
reconstructs; `maybe_lowpass_contour` applies it at the top of `search_placement`
so the router traces a grid-renderable shape instead of chasing sub-block detail
it can only staircase. See [`fd-lowpass-gate.png`](fd-lowpass-gate.png).

The gate is **three-part**. Two on total absolute turning: (1) detail worth
removing — turning above `fd_detail_turns`·2π (a smooth circle/donut sits near 2π
even from raster, so low-passing it only reshuffles the placement); (2) the
low-pass must *smooth* it — cut turning by ≥ `fd_min_turn_drop` (a square/L of
straight edges meeting sharp corners **rings** (Gibbs) and its turning *rises*, so
it is skipped and keeps its crisp target). The third, added after the real-grid
check below, is the **elongation gate**: the low-pass must reshape the *silhouette*
by ≥ `fd_min_silhouette_change` (`perceptual_cost(lp, contour)`). See the resolution
note under the caveat. This first pair already separates the families a
corner-displacement test could not (a star's tips and a horse's legs
move alike, but only the former rings). Applied to Crow/Horse/Knight/Pawn/Shark/
star; skipped on Cat/circle/donut/face/lshape/square (all byte-identical).

**Measured, full benchmark (the first default-ON win of this effort):**
`frechet 0.0376→0.0338, hausdorff 0.0362→0.0317, iou 0.643→0.654,
perceptual 0.032→0.029, turning 0.474→0.379 (−20%)` — six of seven metrics
improve, with standouts Horse (IoU 0.54→0.65, 162→147 nodes) and star
(Frechet 0.046→0.034). The seventh, `excess`, *rises* 17→23, but that is a
**reference artifact, not a regression**: `excess = route_turning −
target_turning`, and the low-passed target has less turning, so the surplus grows
even as the route's own turning and node count fall. The clean cross-condition
signals are route-intrinsic (node count ↓, absolute turning ↓) and the
form-metric `turning` (↓20%); the gate + the figure confirm the smoothed target
still reads as the shape. Why this wins where the router knobs didn't: it changes
the *input*, never a cost the search optimizes — the discipline the whole plan is
built on. Default-ON (`fd_lowpass=True`), gated; set `fd_lowpass=False` to disable.

> **⚠️ Real-grid caveat (measured on the cached Chicago snapshot, Logan Square —
> [`chicago_maps/fd_lowpass_chicago_fullpipeline.png`](../chicago_maps/fd_lowpass_chicago_fullpipeline.png)).**
> The synthetic-grid win **does not transfer to real streets.** Same-placement
> (isolating routing) barely moves anything — Shark combing 52.0→52.0, Knight
> 92.5→92.5 — so the plan's dramatic same-placement numbers do not reproduce here.
> Full pipeline (each does its own placement search) is mixed to net-negative:
> Crow improves (IoU 0.38→0.43, turning 1.16→0.67, the spurious spike gone), but
> Horse (IoU 0.37→0.30) and Knight (0.38→0.30) *regress*, Shark is lateral. The
> low-pass's real effect is to change *which placement wins*, and on irregular
> real streets (diagonals, varying block size) that is a coin-flip, not the
> consistent improvement a uniform lattice gives. The synthetic benchmark
> over-stated this lever; its benefit was mostly the placement-search reseating
> that a uniform grid rewards. The honest conclusion: the **`excess` metric and
> the Hausdorff fix are the grid-independent wins; every rendering/routing lever
> (turn penalty, trellis, FD low-pass) is a situational knob, not a clean
> default.** Whether `fd_lowpass` should stay default-ON (synthetic-CI win) or
> flip to default-OFF (real-grid is the use case) is an open call; the evidence
> leans OFF. n=4 and the comparison is confounded (different placements/targets),
> so a wider real-grid sweep — and deriving `fd_harmonics` from the real
> blocks-per-shape rather than a fixed 20 — should precede a firm default.

> **Per-family engine comparison on Chicago (sharp vs FD vs trellis —
> [`chicago_maps/engines_chicago.png`](../chicago_maps/engines_chicago.png)).**
> The engines split by shape family, but along **elongation, not
> blob-vs-angular** (means, IoU / turning): *angular* sharp 0.31/0.78, FD
> **0.32/0.71**, trellis 0.30/0.79; *blob* sharp **0.36**/0.76, FD 0.35/**0.67**,
> trellis **0.36**/0.72; *mixed* all 0.32, FD best turning 1.21. FD low-pass helps
> shapes with long sweeping edges (Shark IoU 0.33→0.35 turn 1.34→1.11; star turn
> 0.84→0.63) but **rounds compact feature-rich shapes into mush** (Knight IoU
> 0.33→0.30 — visually a lump; Pawn 0.38→0.34; Crow 0.39→0.32). Compact/defined
> shapes want the crisp engine — sharp, or **trellis**, which matches sharp's IoU
> without FD's damage and shaves turning on a few (circle, Cat). Consequence: the
> current FD gate keys only on "does it smooth the turning," so it **mis-fires on
> Knight/Pawn/Crow**. The fix is an **elongation-aware gate** (apply FD only where
> long edges dominate — high aspect / long straight runs — skip compact
> feature-rich shapes), which would make FD default-ON fire only where it helps.

> **✅ Resolution — the elongation gate (implemented,
> [`chicago_maps/fd_elongation_gate_chicago.png`](../chicago_maps/fd_elongation_gate_chicago.png)).**
> A clean geometric separator was *not* obvious: aspect ratio, straight-edge
> fraction, corner-displacement, form-change (turning/IoU of lp-vs-original) and
> residual-localization all **overlap** between the families (Shark's body is as
> "un-straight" as Knight's). The one measure that separates them is the
> **blur-tolerant perceptual change** `perceptual_cost(lp, contour)`: FD helps
> ⇒ 0.013–0.037 (Horse/Shark/star), FD hurts ⇒ 0.004–0.009 (Knight/Pawn/Crow), a
> real gap at ~0.011. Rationale: long thin structure (legs/arms/points) is
> area-sensitive, so the low-pass moves the filled silhouette a lot — and those
> are exactly the shapes the router was combing worst; a compact shape barely
> changes in silhouette, so the low-pass isn't removing combable detail, only
> rounding the one notch that gives the shape its identity. Added as the third
> gate condition (`fd_min_silhouette_change`, default 0.011). Result: FD now
> **applies to Horse/Shark/star and skips Cat/Crow/Knight/Pawn/circle/donut/face/
> lshape/square** — so the compact shapes are byte-identical to the sharp engine
> on Chicago (Knight reads as a knight again, not a blob), while the long-edged
> shapes keep the de-jag. This makes **`fd_lowpass` default-ON defensible**: it
> only fires where it helps and is a no-op everywhere it hurt. Cost: the synthetic
> benchmark gives back a little (Frechet 0.0338→0.0353, still < 0.0376 baseline)
> because the synthetic grid *liked* FD on Knight/Pawn where the real grid hated
> it — the right trade, since the real grid is the use case. Honest caveat: the
> gate threshold is calibrated on the bundled shapes (n≈6, noisy real-grid ground
> truth); Horse remains the one applied shape whose real-grid benefit is
> placement-noisy rather than proven.

**Re-test of the router knobs on the low-passed target (measured).** The
hypothesis was that `turn_weight` / `trellis` were penalized for corner-rounding
they no longer need once the sub-block detail is gone. Measured over all shapes
with `fd_lowpass` on (means): base `frechet 0.0338, iou 0.654, turning 0.379`;
`+turn_weight` `0.0341 / 0.642 / 0.384` (still net-negative — Frechet up, IoU
down: it does not flip, the corner-rounding was not its only problem); `+trellis`
`0.0349 / 0.667 / 0.384` — a **broad IoU gain** (+2%: Crow/Horse/Pawn/donut/face/
lshape/star all rise) bought with a small Frechet cost concentrated on the pointy
shapes (lshape/square 0.012→0.019). So the trellis is a **coverage-vs-order
tradeoff, not a clean flip**, and both stay default-OFF. The one avenue left for
the trellis is a gate (apply on organic shapes, skip pointy ones) to bank the IoU
gain without the pointy-shape Frechet cost — future work. Separately,
`fd_harmonics` could be derived per-run from placed blocks-per-shape rather than
fixed at 20. Verdict stands: **the input-side low-pass is the win; the router-side
knobs are not, low-passed target or no.**

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
