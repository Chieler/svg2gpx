# Rendering-fidelity plan

Raising the ceiling on *how well a routed loop can look like the shape* — as
opposed to more selection-cost tuning, which two experiments (the void term and
the turning-function recognition term) showed moves metrics without a robust
visual win. This plan targets the rendering ceiling itself. **Status: proposed,
not started.**

## Diagnosis (measured on the cached Chicago snapshot, downtown window)

Fidelity ≈ **(shape span) / (block size) = blocks per shape** — how many
independently-placeable boundary points the streets give you.

1. **The cached network is a *drive* network, not walk.** Highway types are
   residential/secondary/tertiary/primary/motorway; **zero alleys, zero
   footways, zero paths** (18 `living_street` of 76k edges). Chicago's
   ~1,900-mile alley grid, park paths and pedestrian cut-throughs are all
   absent — and the drive net even includes non-runnable motorways.
2. **The real resolution ceiling is intersection density, ~40 blocks/shape** —
   not node count:
   - real intersections only (`densify=False`): 752 nodes, **avg block ≈ 104 m**,
     a shape spanning 1.3 of the box crosses **~40 blocks**.
   - `node_spacing=30` densify: 3,439 nodes, avg_edge 34 m — but these are
     mid-edge subdivisions *along* streets, not new places you can **turn**.
     Halving `node_spacing` to 15 m doubles nodes (7,805) and adds no turn options.

We are effectively **drawing each shape at ~40-pixel resolution**. `node_spacing`
mostly smooths curves along a street; it cannot add a mid-block move where no
mid-block street exists. That is the staircase/blob ceiling, quantified.

## Levers, ranked by measured leverage

| # | Lever | Effect | Testable now? |
|---|-------|--------|---------------|
| L1 | Mid-block connectivity (alleys + pedestrian/park paths) | ~2× turn-resolution where it matters; also makes routes actually runnable | Real fix needs Overpass (blocked) / richer snapshot; upside boundable now |
| L2 | Blocks-per-shape (radius / scale) | resolution scales ~linearly; paid in run-length | Yes |
| L3 | Match target detail to grid resolution (FD low-pass) | stop asking the router to draw sub-block detail → less combing/aliasing | Yes |
| L4 | Densification (`node_spacing`) | curve smoothing only; diminishing returns | Yes |

## Phases

### Phase 0 — Instrumentation (do first, testable now)
Make fidelity measurable *at matched scale* so we don't repeat circular
validation:
- **Effective-resolution report**: blocks-per-shape, block-size distribution,
  real-intersection count vs densified-node count.
- **Resolution-controlled fidelity metric**: turning / Fréchet against the
  *ideal* outline, reported next to blocks-per-shape, so "fidelity improved" is
  separable from "the shape just got bigger." This is the yardstick every later
  phase is judged by.

### Phase 1 — Mid-block connectivity (highest leverage; partly blocked)
- **Real fix (needs Overpass or a richer snapshot):** `network_type` /
  `custom_filter` config to pull `walk` + `service`(alley) + `footway`/`path`.
  In Chicago this ~doubles mid-block turn-resolution and makes routes runnable.
  Overpass is 403 in this environment → "when unblocked, or drop in a
  walk-network GraphML."
- **Bound the upside now (no network):** synthesize an alley grid by injecting
  one mid-block connector per block into the *existing* cached graph, re-run a
  shape, measure the fidelity delta. Tells us whether chasing the real walk
  network is worth it before depending on Overpass.

### Phase 2 — Blocks-per-shape via radius/scale (testable now)
Sweep `radius_m` / scale to characterize the **fidelity-vs-run-length**
trade-off (bigger shape = finer rendering = longer run). Optionally add a
`target_blocks` auto-scale ("≈60-block resolution"). A knob, not a default —
it costs kilometers.

### Phase 3 — Match detail to resolution: Fourier low-pass + densification (testable now)
Two sub-parts, same theme (don't ask for detail the streets can't draw):

- **Fourier-descriptor low-pass of the target (the promising part).** Represent
  the closed target contour as `z(t)=x(t)+i·y(t)`, DFT it, keep only the low
  harmonics `K ≈ blocks/2 ≈ 20` (grid Nyquist), reconstruct, and route to *that*
  smoothed target. Above the grid's Nyquist the router chases detail it renders
  as staircase/comb (aliasing); low-passing removes that source at the input.
  - **Tradeoff:** FD low-pass *rounds sharp corners* and can add Gibbs ringing.
    A star / L-shape lives in its corners → this hurts them; organic shapes
    (shark, crow, horse body) benefit. Must be a per-shape toggle; the
    turning-function's corner-preservation is the counter-argument for pointy
    shapes. Consider auto-gating by corner content (few strong corners → apply).
- **`node_spacing` sweep.** Confirm the diminishing-returns curve; keep the
  point where curve-following stops improving (~street width, 15–20 m). Cheap;
  mostly corrects an over-promised CONFIG comment.

### Selection-metric shelf item (only if we revisit scoring)
A **low-band Fourier-descriptor distance** — compare only the first ~15
normalized-magnitude harmonics — is a scale/rotation-invariant *gross-form*
match that ignores the high-frequency staircase (what `perceptual_cost` does via
blur, but scale-invariant; what `turning_distance` fails at, since turning is
dominated by local corners inflated by staircase). It is the one form-metric
variant I'd expect to beat turning. Low overall expectations on selection
metrics (see the reverted void + off-by-default recognition experiments); test
it honestly against turning, with the corner-rounding caveat, before believing.

## Validation (all phases)
Matched-scale visual A/B + the Phase-0 resolution-controlled metric. Never
validate a rendering change by a cost the search also optimizes.

## Constraints & honest risks
- **Overpass is blocked here**, so Phase 1's real fix can't be exercised in this
  environment — the synthetic-alley experiment is the workaround.
- Phase 2 lengthens routes; Phase 1 alleys raise a real-world safety question →
  both stay opt-in toggles.
- FD low-pass is shape-dependent (rounds corners) — toggle + auto-gate, never a
  blanket default.
- Biggest likely payoff is Phase 1 (~2× the resolution that matters), but its
  real form depends on network access.

## Recommended first move
Phase 0 + the Phase-1 synthetic-alley bounding experiment, on cached data: the
honest yardstick *and* a measured answer to "how much would a proper walk/alley
network buy us" before committing to the blocked Overpass path. FD low-pass
(Phase 3) is the next testable-now experiment after that.
