"""
Checks for the routing layer (run: python test_routing.py).

Covers route_pair (incl. best-effort bridging to the closest approach when the
target is unreachable), route_contour's connected-walk guarantee across a
disconnected graph, and an end-to-end pipeline smoke on the synthetic lattice.
"""

import numpy as np
from scipy.spatial import cKDTree

import gen
from gen import Grid, route_contour, route_pair, search_placement
from benchmark import synthetic_grid


def dissolve_checks():
    """dissolve_oscillations collapses a doubling-back comb tooth but leaves a
    monotone staircase alone, and always emits a connected walk."""
    cfg = dict(gen.CONFIG)
    cfg.update(grid_size=41, grid_diagonals=True)
    grid = synthetic_grid(cfg)
    s = 1.0 / 40
    node = lambda i, j: (round(i * s, 6), round(j * s, 6))

    # A comb tooth: run along row 20, poking up to row 21 and back (doubles back).
    row = 20
    tooth = []
    for i in range(10, 21):
        tooth.append(node(i, row))
        if i % 2 == 0:
            tooth += [node(i, row + 1), node(i, row)]
    outline = np.array([node(i, row) for i in np.linspace(10, 20, 60).astype(int)],
                       dtype=np.float64)
    out = gen.dissolve_oscillations(grid, tooth, cKDTree(outline), cfg)
    check(len(out) < len(tooth), "dissolve collapses a doubling-back comb tooth")
    check(is_connected_walk(out, grid.graph),
          "dissolve output is a connected walk on real edges")

    # A monotone shallow staircase (slope ~1:4): the faithful grid rendering of a
    # shallow straight edge -- its shortest-path bypass is the same length, so it
    # must survive untouched.
    stair, x, y = [], 4, 20
    for k in range(24):
        stair.append(node(x, y)); x += 1
        if k % 4 == 3:
            y += 1
    stair_ct = cKDTree(np.array(stair, dtype=np.float64))
    kept = gen.dissolve_oscillations(grid, stair, stair_ct, cfg)
    check(len(kept) == len(stair),
          "dissolve preserves a faithful shallow staircase (no corner-cutting)")


def check(cond, msg):
    assert cond, msg
    print(f"  ok: {msg}")


def two_island_grid():
    """Two 5x5 lattices with a gap between them (disconnected components)."""
    graph, edge_list = {}, []

    def link(a, b):
        d = float(np.hypot(a[0] - b[0], a[1] - b[1]))
        graph.setdefault(a, []).append((b, d))
        graph.setdefault(b, []).append((a, d))
        edge_list.append((a, b))

    s = 0.05
    for ox_ in (0.1, 0.7):                      # two islands, 0.3 apart
        for i in range(5):
            for j in range(5):
                a = (round(ox_ + i * s, 6), round(0.4 + j * s, 6))
                if i + 1 < 5:
                    link(a, (round(ox_ + (i + 1) * s, 6), round(0.4 + j * s, 6)))
                if j + 1 < 5:
                    link(a, (round(ox_ + i * s, 6), round(0.4 + (j + 1) * s, 6)))
    node_keys = list(graph.keys())
    nodes_arr = np.array(node_keys, dtype=np.float64)
    return Grid(graph, node_keys, nodes_arr, cKDTree(nodes_arr),
                s, edge_list, 1000.0, 0.0, 0.0, 0.0)


def is_connected_walk(route, graph):
    """Every consecutive pair of route nodes is a real graph edge."""
    return all(b in {n for n, _ in graph.get(a, ())}
               for a, b in zip(route, route[1:]))


def main():
    grid = two_island_grid()
    west = (0.1, 0.4)
    east_far = (0.9, 0.6)                        # unreachable from west island
    seg = np.array([west, east_far])

    path = route_pair(grid.graph, west, (0.2, 0.5), seg, 0.0)
    check(len(path) >= 2 and path[0] == west and path[-1] == (0.2, 0.5),
          "route_pair reaches a same-island target")

    check(route_pair(grid.graph, west, east_far, seg, 0.0) == [],
          "route_pair returns [] for an unreachable target by default")

    bridged = route_pair(grid.graph, west, east_far, seg, 0.0, best_effort=True)
    check(len(bridged) >= 2 and bridged[-1][0] == 0.3,
          "best_effort bridges to the closest approach (east edge of west island)")
    check(is_connected_walk(bridged, grid.graph),
          "bridged path is a connected walk on real edges")

    # Waypoints alternating across the gap: the assembled route must still be
    # a single connected walk (closest approach instead of teleports).
    wps = [west, (0.3, 0.5), (0.75, 0.5), (0.3, 0.6), (0.2, 0.6)]
    dense = np.array(wps, dtype=np.float64)
    route = route_contour(grid, dense, wps, list(range(len(wps))), 0.0)
    check(len(route) >= 4, "route_contour produces a route across mixed waypoints")
    check(is_connected_walk(route, grid.graph),
          "route_contour output is a connected walk (no teleports)")

    # End-to-end determinism + connectivity on the standard synthetic lattice.
    cfg = dict(gen.CONFIG)
    cfg.update(grid_size=30, grid_diagonals=True, n_random=300, n_refine=100,
               n_route_eval=2, inner_features=False)
    lattice = synthetic_grid(cfg)
    spec = gen.extract_shape("shapes/star.svg", 512)
    cand = search_placement(spec.outer, lattice, cfg)[0]
    check(len(cand.route) > 10, "pipeline smoke: star routes on the lattice")
    check(is_connected_walk(cand.route, lattice.graph),
          "pipeline smoke: star route is a connected walk")
    check(cand.route[0] == cand.route[-1], "pipeline smoke: star route is closed")

    dissolve_checks()
    momentum_checks()
    trellis_checks()
    fd_lowpass_checks()

    print("\nall routing checks passed")


def fd_lowpass_checks():
    """Fourier low-pass smooths a detailed outline, and the gate fires on organic
    shapes (it smooths them) but skips sharp ones (it would ring them)."""
    turn = lambda c: gen._total_abs_turning(gen.resample(c, n=400))
    horse = gen.extract_contour("shapes/Horse.svg", 512)
    lp = gen.fourier_lowpass(horse, 20)
    check(turn(lp) < turn(horse), "fourier_lowpass reduces a detailed outline's turning")

    cfg = {**gen.CONFIG, "fd_lowpass": True}
    fires = lambda s: not np.array_equal(
        gen.maybe_lowpass_contour(
            np.asarray(gen.extract_contour(f"shapes/{s}.svg", 512), dtype=float), cfg),
        np.asarray(gen.extract_contour(f"shapes/{s}.svg", 512), dtype=float))
    check(fires("Horse"), "gate applies the low-pass to an organic shape (horse)")
    check(not fires("square"), "gate skips the low-pass on a sharp shape (square rings)")


def trellis_checks():
    """The candidate-set trellis router yields a closed connected walk, and with
    trellis_k=1 (a single candidate per anchor) it degenerates to single-snap."""
    cfg = dict(gen.CONFIG)
    cfg.update(grid_size=30, grid_diagonals=True, n_random=300, n_refine=100,
               n_route_eval=2, inner_features=False, trellis=True)
    lattice = synthetic_grid(cfg)
    spec = gen.extract_shape("shapes/star.svg", 512)
    cand = search_placement(spec.outer, lattice, cfg)[0]
    check(len(cand.route) > 10 and is_connected_walk(cand.route, lattice.graph)
          and cand.route[0] == cand.route[-1],
          "trellis routing: closed connected walk on the lattice")

    # trellis_k=1 -> one candidate (the nearest node) per anchor, so the trellis
    # has no choice and must reproduce a connected closed walk just like single-snap.
    dense, anchor_idx = gen._densify_and_anchor(cand.placed, lattice, cfg, closed=True)
    r1 = gen.route_contour_trellis(lattice, dense, anchor_idx,
                                   cfg["deviation_weight"], {**cfg, "trellis_k": 1})
    check(len(r1) > 10 and is_connected_walk(r1, lattice.graph) and r1[0] == r1[-1],
          "trellis_k=1 degenerates to a valid single-candidate walk")


def momentum_checks():
    """The turn-penalty (momentum) router: turn_weight=0 is byte-identical to the
    bare-node Dijkstra, and turn_weight>0 still yields a connected walk."""
    grid = two_island_grid()
    west = (0.1, 0.4)
    seg = np.array([west, (0.2, 0.5)])
    plain = route_pair(grid.graph, west, (0.2, 0.5), seg, 30.0)
    same = route_pair(grid.graph, west, (0.2, 0.5), seg, 30.0, turn_weight=0.0)
    check(plain == same, "turn_weight=0 matches the bare-node router exactly")
    momentum = route_pair(grid.graph, west, (0.2, 0.5), seg, 30.0,
                          turn_weight=20.0, avg_edge=grid.avg_edge)
    check(len(momentum) >= 2 and momentum[0] == west and momentum[-1] == (0.2, 0.5)
          and is_connected_walk(momentum, grid.graph),
          "turn_weight>0 still returns a connected walk to the target")

    # End-to-end: the penalty produces a closed connected walk on the lattice.
    cfg = dict(gen.CONFIG)
    cfg.update(grid_size=30, grid_diagonals=True, n_random=300, n_refine=100,
               n_route_eval=2, inner_features=False, turn_weight=25.0)
    lattice = synthetic_grid(cfg)
    spec = gen.extract_shape("shapes/star.svg", 512)
    cand = search_placement(spec.outer, lattice, cfg)[0]
    check(len(cand.route) > 10 and is_connected_walk(cand.route, lattice.graph)
          and cand.route[0] == cand.route[-1],
          "pipeline smoke with turn_weight>0: closed connected walk")


if __name__ == "__main__":
    main()
