# Recognizability plan

**Reframe:** the contour is a *template*, not a constraint. The goal is a route
that **reads as the target animal**, not a route that minimizes distance to a
polyline. This document says what to do about measurement — since the distance
metrics demonstrably can't carry that goal — and records the first shipped
changes built on the reframe.

## Why the distance metrics failed

Five engine experiments in a row (the void term, the recognition term, the turn
penalty, the trellis, the Fourier low-pass) moved Frechet/Hausdorff/IoU/DTW
without producing routes that *look* better; the low-pass looked actively worse
on real streets while winning the synthetic benchmark. Four structural reasons:

1. **Wrong reference.** Every metric scores the route against the *placed*
   template — which the search itself already warped (up to 1.5× stretch +
   0.2 shear) and simplified. A perfectly-traced stretched horse scores 1.0
   and reads wrong; the reference hides the crime.
2. **Adherence ≠ identity.** A shape's identity at ~30-block resolution lives in
   a handful of features (the knight's snout notch, the shark's fin). IoU barely
   notices a feature vanishing (Knight 0.33→0.30 while turning into a blob);
   Frechet is hostage to one worst point the eye forgives.
3. **Noise dominates signal.** Staircase jitter moves the pointwise metrics far
   more than feature loss does, so the metrics rank *smoothness*, not likeness.
4. **Goodhart.** Any of these used as a search objective or validation target
   gets optimized into meaninglessness — measured four separate times in this
   repo's history.

## What to use instead

Ordered from cheap/now to heavier/later:

### 1. Fix the reference: canonical-template scoring
Score candidate routes against the **unwarped canonical template**, aligned by
similarity transform only (rotation + uniform scale + translation). Warp then
becomes a *measured defect* instead of a hidden one. The turning-function
distance already lives in this family; the change is what we compare against,
not the formula.

### 2. The feature ledger: recall/precision of defining features
The RDP simplification (shipped in this branch) reduces a template to ~22
corners — an explicit list of the features that *are* the identity. That enables
a direct metric:

- match template corners to route corners with a cyclic, order-preserving
  assignment (small DP, both sequences are short);
- **feature recall** — fraction of template corners with a matched route corner
  of compatible turn sign/magnitude and relative position;
- **feature precision** — fraction of route corners that correspond to a real
  template feature (penalizes hallucinated combs the way `excess` did, but
  locally attributable).

This is the metric that would have caught the Knight-becomes-blob failure that
IoU scored as −0.03. It is cheap, deterministic, and CI-friendly. **Highest
value next implementation.**

### 3. Semantic judges (dev-time oracle, never an objective)
The route is literally a doodle, so use doodle recognizers:

- **QuickDraw-class sketch classifier** — Google's QuickDraw categories include
  horse, shark, star, cat, face, circle, donut, square: almost exactly the
  bundled shape set. A small CNN gives `P(label | rendered route)`; report the
  correct label's rank. Small, CPU-fast, offline once weights are fetched.
- **CLIP text–image similarity** — embed the rendered route, compare against
  "a drawing of a horse" vs distractor labels; zero training, heavier dep.
- **VLM-as-judge** — in CI, ask a vision model "what animal is this route?"
  Strongest signal, external dependency.

Doctrine: these run as *evaluation* after generation and in benchmarks. They
must never become the router's cost function (reason 4 above).

### 4. Human-verdict calibration (make the eyeball reusable)
The eyeball is the final metric but doesn't scale. Bottle it once: collect
pairwise A/B verdicts on the renders this repo already generates (a
`verdicts.csv` of "left/right reads better"), then score every candidate metric
by **agreement % with the human pairs**. A metric earns trust only by beating
the others there. ~50 pairs is enough to rank metrics; the failed experiments
already produced the images.

### 5. Usage doctrine
Metrics are **filters and regression alarms, not objectives**: prune obviously
bad candidates, alert when a change tanks a calibrated score, and always gate
releases on side-by-side renders. `option_mode="placements"` (already in main)
is the human-pick path and stays the last word.

## Shipped in this branch (evidence-backed, from the knob study on main)

Measured on the real Chicago snapshot (Logan Square), main's engine, knobs only
— the **stack recipe** produced the largest fidelity jump of any change tested
in this project (defaults → stack: Horse IoU 0.28→0.44, Knight 0.33→0.48,
Shark 0.33→0.46, star 0.31→0.41, and the renders visibly read as the animals):

1. **Template simplification** (`template_vertices`, default 22) —
   corner-preserving RDP of the template before placement. Keeps the ~22 corners
   that carry identity, discards raster wiggle the streets can't draw. The exact
   opposite of the Fourier low-pass (which smoothed corners and kept wiggle, and
   lost the eyeball test).
2. **Warp caps** (`aspect_max` 1.5→1.15, `shear_max` 0.20→0.05) — placement may
   no longer stretch/shear the animal into a better-seating but wrong-reading
   shape. Free on Chicago (best small-radius Shark/star came from this alone).
   Raise them back for abstract/geometric shapes on strongly anisotropic grids
   (the original Manhattan-blocks rationale).
3. **Feature-hug routing** (`flat_deviation_frac`) — the deviation weight now
   ramps per leg with `waypoint_importance`: full hug near defining corners,
   relaxed on featureless stretches so connecting streets run clean instead of
   staircasing after the template's raster jitter. This is the reframe applied
   to the router: adhere at features, delegate the in-between to the street
   fabric.
4. **Canvas guidance** — resolution ≈ blocks-per-shape is the dominant fidelity
   lever: radius 1600→2400 alone was worth ≈ +0.10 IoU, ~3× any engine change,
   paid in route length (13–22 km). Documented at `radius_m`; an auto
   `target_blocks` knob is future work (radius is a data-fetch parameter, so
   auto-scaling it belongs with the fetch layer).

## Follow-ups
- Implement the **feature ledger** metric (§2) and calibrate all metrics against
  a first human-verdict set (§4).
- QuickDraw judge prototype (§3) once weights can be fetched.
- `target_blocks` auto-scale in the fetch layer.
- Walk-network (alleys/footpaths) remains the blocked ~2× resolution lever
  (Overpass 403 from this environment; a walk-network GraphML snapshot would
  unblock it).
