# Post-mortem: why each approach worked or didn't

Every intervention this project has measured, judged by its end state (real-grid
renders + the ledger), reduced to the principle it proved. Newest evidence
lives in `recognizability-plan.md` / `deformation-plan.md`; the earlier router
experiments live on the `claude/gen-py-architecture-optimization-n9nx19` branch
(PR #7) and `routing-fidelity-plan.md`.

## What worked, and the reason it worked

| Intervention | Result | The principle it proved |
| --- | --- | --- |
| **Canvas scale** (radius 1600→2400) | +0.10 IoU, ~3× any engine change, visibly better | **P1 — information budget first.** Fidelity ≈ blocks-per-shape; the canvas is the pixel count and nothing downstream can add information it lacks. |
| **RDP template** (`template_vertices`) | biggest jump of the knob study; Horse finally read | **P2 — identity is corners.** At 25–60 blocks a figure *is* its ~20 corners plus their arrangement; everything else is delegable to the street fabric. RDP keeps exactly the corners and deletes exactly the noise. |
| **Warp caps** (aspect 1.5→1.15, shear 0.20→0.05) | free win (best small-canvas Shark/star) | **P5 — the reference must be canonical.** The search was spending identity (stretch/shear) to buy adherence, and every metric scored against the warped copy, hiding the spend. |
| **Feature-hug** (`flat_deviation_frac`) | part of the winning recipe | P2 applied to the router: spend the hug where identity lives, let streets draw the flats. |
| **ARAP bend v2 + identity budget** (`bend_template`, ledger-metered) | star recall 0.47→0.73; never worse; routes visibly follow the bent target | **P3 — operate at or above block scale.** Corridor-scale re-draping is something the router structurally cannot do; the budget converts "at all cost" into "up to a measured identity price". |
| **Feature ledger** (`feature_ledger`) | caught Knight-blob, star corner-loss that IoU scored ±0.03 | **P4 — judge outside the loop.** It audits the identity carriers directly, order-preserving so it can't be gamed by out-of-order matches, and the search never optimizes it. |
| **Compactness dispatch + engines panel** | Shark→classic (recall 0.44 vs 0.20), duck→recipe (same IoU, −25% km) | **P6 — no single engine.** The families' strengths are complementary, and one scalar (P²/4πA — literally "perimeter spent on protrusions") separates them with a natural gap at 2.0. |
| **Instrumentation** (excess metric, Hausdorff resample fix, PR #7) | exposed phantom failures; trustworthy columns | You can't steer with a broken gauge: the "big-Hausdorff excursions" the trellis was built to fix never existed. |

## What didn't work, and the reason it didn't

| Intervention | Result | The principle it violated |
| --- | --- | --- |
| **Turn penalty** (`turn_weight`, PR #7) | lowered combing on organic shapes but rounded pointy corners; no good global value | A *local* cost cannot encode identity, which is *global*: a comb-tooth turn and a staircase-approaching-a-tip turn are the same local event with opposite meanings. |
| **Trellis router** (`trellis`, PR #7) | BPO-exact, end-to-end mixed | Exact optimization of the wrong objective: the additive surrogate (length+deviation) ≠ fidelity. Optimality transfers nothing across an objective gap. |
| **Fourier low-pass** (`fd_lowpass`, PR #7) | won the synthetic benchmark, failed the real-grid eyeball; blobbed compact shapes | Identity (corners) and noise (raster jitter) are BOTH high-frequency — the spectral basis cannot separate them; spatial locality (RDP) can. Most of its synthetic "win" was placement reshuffling. Violated P2. |
| **Sub-block bend** (v1, node/segment pull ≤ ½ block) | physics perfect (−40% street dist), end-to-end neutral | Violated P3: snapping + the deviation router + cleanup already form a discretizer that absorbs sub-block mismatch. It elegantly redid work that was already done. |
| **Rotation cap** (`rot_max`, rescinded) | solved a non-problem | Identity properties are those invariant to how the art is viewed; orientation belongs to the display frame, not the figure. |
| **Distance metrics as objectives/validators** | five experiments moved them with no visual win | **Goodhart.** Any judge inside the optimization loop stops judging. (Also: adherence ≠ identity — IoU went *up* as shapes shrank into unrecognizable blobs.) |
| **selection-cost tuning** (void term, recognition term — pre-branch history) | reverted / default-off | Same two failures: optimizing a proxy, scored against a warped reference. |

## The synthesis

A recognizable route needs, in causal order: **enough canvas (P1) → a demand
matched to it with corners preserved (P2) → block-scale placement/deformation
under a metered identity budget (P3, P5) → an engine matched to the shape
family (P6) → judged by instruments the search can't touch, with the human
verdict last (P4).** Every success sits somewhere on this chain; every failure
was an attempt to substitute cleverness at the wrong link — usually sub-block
router intelligence (P3) or proxy optimization (P4).

## Residual honest weaknesses

- Ledger **precision** is structurally low on grids (street-drape corners count
  as "invented"); use it comparatively, not absolutely.
- **Crow (2.00) and Pawn (2.27)** sit on the dispatch boundary — to be settled
  by the human-verdict set, which remains uncollected (the one open item of the
  measurement plan).
- **Horse legs** still comb at every tested scale: thin parallel limbs need
  either corridor-scale limb assignment beyond the current budgeted bend, or
  the blocked walk-network (alleys ≈ 2× resolution).
- The synthetic lattice benchmark is a **weak predictor of real-grid outcomes**
  (FD low-pass being the cautionary tale); treat it as a regression alarm only.
