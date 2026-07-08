# Deformation plan: bend the template onto the streets (ARAP & friends)

**Goal restated:** use the street grid and the contour as *references* and
produce a route that resembles the figure **at all cost**. That phrasing points
at a formalism this project hasn't used yet: **non-rigid shape registration**.
Instead of (a) placing the template with one global affine and then (b) asking a
router to chase lines the grid cannot draw, *deform the template minimally so
its lines ARE street lines* — and account for every bend in an explicit energy.
This dissolves the staircase-vs-identity dilemma at the source and, as a
side-effect, produces the attributable fidelity numbers the distance metrics
never gave us.

## Why this is the right next abstraction

Everything measured so far reduces to one tension:

- the template demands line segments at arbitrary angles/positions;
- the grid offers segments only along its streets;
- the router resolves the mismatch by **aliasing** (staircase/comb) — ugly;
- spectral smoothing resolved it by **destroying identity** (Knight → blob);
- the winning knob study resolved it by **reducing demand** (RDP corners,
  bigger canvas) — good, but the placed template still lies *across* blocks,
  so every flat still renders as staircase.

The missing move is the third option: **move the template's lines onto the
streets**, spending a *bounded* amount of shape distortion to do it. "As rigid
as possible" is precisely "resembling at all cost": all freedom that doesn't
change local shape (translation, rotation of parts by a few degrees) is free;
everything that does change local shape (stretch, shear, corner-angle change)
is charged.

## The toolbox (established formalisms, mapped to our problem)

### 1. ARAP contour registration (the recommended core)
As-Rigid-As-Possible deformation (Sorkine & Alexa 2007), 1-D chain variant.
Template vertices `V_i` (the RDP corners plus subdivision points), deformed
positions `P_i`, per-neighborhood rotations `R_i`:

```
E(P, R) =  Σ_i  Σ_{j∈N(i)} ||(P_i − P_j) − R_i (V_i − V_j)||²      (rigidity)
         + λ_s Σ_i  d_streets(P_i)²                                 (attraction)
         + λ_b Σ_i  ||angle(R_i) − angle(R_{i+1})||²                (bend smoothness)
```

- `d_streets` = distance to the street fabric (we already have the cKDTree; a
  distance-to-nearest-edge field is a small upgrade).
- Solved by the standard **local–global alternation**: fix `P`, each `R_i` is a
  closed-form 2×2 SVD (in 2-D just an `atan2` of a covariance); fix `R`, `P` is
  one sparse linear solve (scipy). Fast, dependency-free.
- The bend-smoothness term is the "**sheet/wire bending**" intuition: the
  template behaves like an elastic wire pressed onto the street lattice — it
  may flex, it resists kinking anywhere the figure didn't already kink.
- **Corners are protected by construction**: at a template corner the reference
  difference vectors already turn, so reproducing the turn costs nothing, while
  *removing* it (straightening a fin) costs rigidity energy. This is the exact
  inverse of the FD low-pass failure mode.

### 2. Stiffness-annealed non-rigid ICP (how to run #1 globally)
The attraction target (which street each part should lie on) is unknown up
front — the classic chicken-and-egg of registration. The standard answer
(optimal-step non-rigid ICP, Amberg et al. 2007): iterate
*assign-nearest-street → solve ARAP → re-assign*, starting **stiff** (≈ the
current global affine — which stays as the initializer) and **annealing**
looser, so global pose locks in before limbs start bending. Terminates when the
assignment stabilizes or the energy budget (below) is spent.

### 3. Snakes / active contours (the minimal prototype)
Kass–Witkin–Terzopoulos with the street distance field as external energy and
internal energy measured **relative to the template's geometry** (edge-length
and turning-angle deviation from the *template*, not from straightness — a
plain snake wants to be smooth; ours must want to be *the shark*). This is a
~100-line scipy prototype of #1 with per-vertex gradient steps instead of the
local–global solve; the right first implementation to de-risk the idea.

### 4. Discrete alternative: octilinear schematization
Metro-map schematization (Nöllenburg et al.): snap every template edge to the
grid's allowed orientations (`grid_angle` + 45° steps) with hinges at vertices,
minimizing displacement + distortion. Our lattice has exactly those
orientations. Cruder than ARAP (hard orientation constraints instead of an
energy), but it produces perfectly street-parallel lines by construction and
has known exact/ILP and heuristic algorithms. Worth a look if ARAP's soft
attraction leaves residual staircase.

## "At all cost" made precise: the identity budget

ARAP's energy decomposes **per vertex** into stretch (edge-length change vs the
template) and bend (corner-angle change vs the template). That enables the
formulation the goal statement asks for:

> **maximize street-conformance subject to E_identity ≤ B** —
> spend deformation where the streets demand it, refuse to overspend.

`B` is a user knob in intuitive units ("how much may the figure flex before it
stops being itself"), replacing the tangle of aspect/shear/granularity
trade-offs. Placement search survives as the initializer and can shrink to a
coarse pose search (the ARAP refinement absorbs the fine fitting that
`n_refine` jitter does today).

## What this does to fidelity measurement (the explainer)

The registration view is what finally makes fidelity *attributable*. After the
bend stage, report:

1. **Spent identity energy `E_identity`** — total and per-vertex stretch/bend
   breakdown. For the first time a number can say *"the tail-fork angle gave up
   38° and the belly stretched 6%"* instead of "Frechet is 0.07". This is the
   canonical-reference principle from `recognizability-plan.md` §1, upgraded:
   the reference is the undeformed template, and the *deformation itself* is
   the measurement.
2. **Feature ledger vs the canonical template** (plan §2) — recall/precision of
   the RDP corners after deformation: did every identity-carrying corner
   survive with compatible turn sign/magnitude? Catches exactly the
   Knight-becomes-blob failure that IoU scored as −0.03.
3. **Residual street distance** of the bent template — predicts routing quality
   *before* routing (high residual = this neighborhood can't draw that part).
4. **Route-vs-bent-template adherence** — the old metrics, now meaningful,
   because the bent template is drawable: adherence should be near-perfect and
   any gap is a router bug, not aliasing.
5. **Orientation cost** — rotation beyond the canonical band is identity loss
   too (see below), reported alongside `E_identity`.

The distance metrics aren't discarded — they're *re-scoped* to (4), the one
question they can actually answer. Recognizability lives in (1), (2) and (5),
calibrated against the human-verdict set (plan §4).

## Orientation: a display concern, not an identity one (`rot_max` off)

The vertical-Shark placement was first read as an identity failure and a
`rot_max=±35°` cap was tried — then reverted on the correct observation that
**the finished route is map art the viewer can rotate**: a "vertical shark" is
fine because the print/phone can be turned. The knob remains (default `None` =
free spin) for the one context where orientation genuinely matters: embedding
in a fixed north-up map next to labels. Recognizability judging (semantic
oracles, human verdicts) should likewise evaluate renders at the figure's
canonical orientation, not north-up.

## Plan & prototype results

1. **Prototype — DONE (`bend_template` in gen.py, ships default-OFF).** Full
   1-D ARAP local–global (per-vertex closed-form Procrustes rotations + one
   cyclic-Laplacian solve per round) with stiffness-annealed attraction, i.e.
   #1 + #2 directly rather than the gradient snake. Two attraction variants
   were measured on Chicago @ r2400:
   - *node attraction*: template street-distance −40% at only 0.015–0.03
     perceptual change (physics works), but the bent target wobbles
     node-to-node → route turning +10%, IoU unchanged;
   - *segment-projection attraction* (consecutive points project onto the same
     street → collinear targets → stretches settle along block faces): wobble
     gone, but end-to-end still **neutral** — IoU ties on all four shapes,
     renders indistinguishable.

   **Why neutral, and what it teaches:** snapping + the contour-biased router +
   cleanup already form a discretizer that absorbs sub-block mismatch. Bending
   the target ≤ half a block therefore doesn't change *which streets* the route
   takes — the router was already choosing those streets. The remaining quality
   gap on this canvas is **corridor-scale** (which street a limb lies along,
   resolution, template demand), not sub-block alignment. Consistent with every
   other measured result: resolution (+0.10 IoU) and template demand (RDP) moved
   the picture; sub-block cleverness did not.
2. **v2 — corridor-scale ARAP (the version with leverage):** raise the pull far
   beyond the sub-block regime so limbs can relocate onto *different* streets,
   governed by the **identity budget** `E_identity ≤ B` so the figure cannot
   dissolve; add self-intersection guards for thin limbs. This is where the
   registration idea stops duplicating the router and starts doing something the
   router cannot: block-level re-draping of the figure onto the fabric.
3. Evaluate **schematization (#4)** as the discrete alternative if v2's soft
   attraction proves hard to control.
4. Calibrate everything against the human-verdict set
   (recognizability-plan §4) before trusting any number as a gate.
