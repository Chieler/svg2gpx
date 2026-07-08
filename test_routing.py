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

    ledger_checks()

    print("\nall routing checks passed")


def ledger_checks():
    """feature_ledger: perfect on identity, catches lost corners (recall) and
    invented corners (precision) -- the two identity failures IoU cannot see."""
    sq = np.array([(0., 0.), (1., 0.), (1., 1.), (0., 1.), (0., 0.)])
    led = gen.feature_ledger(sq, sq)
    check(led["recall"] > 0.99 and led["precision"] > 0.99,
          "ledger: identity square scores recall=precision=1")

    clipped = np.array([(0., 0.), (1., 0.), (1., .8), (.8, 1.), (0., 1.), (0., 0.)])
    led = gen.feature_ledger(clipped, sq)
    check(led["recall"] < 0.99 and abs(led["recall"] - 0.75) < 0.1,
          "ledger: clipping one square corner drops recall to ~3/4")

    tooth = np.array([(0., 0.), (.4, 0.), (.4, .15), (.5, .15), (.5, 0.),
                      (1., 0.), (1., 1.), (0., 1.), (0., 0.)])
    led = gen.feature_ledger(tooth, sq)
    check(led["recall"] > 0.99, "ledger: a comb tooth does not hurt recall")
    check(led["precision"] < 0.75,
          "ledger: a comb tooth's invented corners drop precision")

    t = np.linspace(0, 2 * np.pi, 200)
    circle = np.column_stack([0.5 + 0.4 * np.cos(t), 0.5 + 0.4 * np.sin(t)])
    led = gen.feature_ledger(circle, circle)
    check(led["recall"] > 0.99 and led["n_template"] == 0,
          "ledger: corner-free template is a vacuous pass, not a fail")


if __name__ == "__main__":
    main()
