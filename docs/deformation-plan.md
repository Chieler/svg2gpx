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

## Immediate fix shipped alongside: orientation is identity (`rot_max`)

The Shark regression on this branch was measured and diagnosed: the placement
search selected a ~90°-rotated (vertical) Shark because it scored *highest
overlap* (IoU 0.47) — and a rotated shark stops reading as a shark at perfect
overlap, which no overlap metric notices. That is reason #1 from the
recognizability plan (adherence metrics can't see identity) biting in
production. Fix: `rot_max` (default ±35°) caps placement rotation around the
drawn orientation; `reflect` still allows facing either way; ±35° nearly covers
a 72–90° symmetry period so star/square lose little seating freedom; `None`
restores free spin for orientation-free shapes. In the ARAP formulation this
becomes a global-rotation term of the same energy.

## Plan

1. **Prototype (snake form, #3)**: bend stage between placement and snapping on
   the Chicago window; eyeball A/B + energy report. No pipeline surgery — it
   transforms `placed` before `snap_waypoints`.
2. If the eyeball agrees: **productionize as ARAP local–global (#1) with
   stiffness annealing (#2)**, expose the identity budget `B`, wire the energy
   breakdown + feature ledger into the benchmark as the new fidelity columns.
3. Evaluate **schematization (#4)** only if soft attraction leaves staircase.
4. Calibrate everything against the human-verdict set (plan §4) before trusting
   any of it as a gate.
