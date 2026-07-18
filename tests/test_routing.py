"""
Checks for the routing layer (run: python test_routing.py).

Covers route_pair (incl. best-effort bridging to the closest approach when the
target is unreachable), route_contour's connected-walk guarantee across a
disconnected graph, and an end-to-end pipeline smoke on the synthetic lattice.
"""

import numpy as np
from scipy.spatial import cKDTree

from svg2gpx import gen
from svg2gpx.gen import Grid, route_contour, route_pair, search_placement
from svg2gpx.benchmark import synthetic_grid
from svg2gpx.shapes import shape_path


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
    spec = gen.extract_shape(shape_path("star"), 512)
    cand = search_placement(spec.outer, lattice, cfg)[0]
    check(len(cand.route) > 10, "pipeline smoke: star routes on the lattice")
    check(is_connected_walk(cand.route, lattice.graph),
          "pipeline smoke: star route is a connected walk")
    check(cand.route[0] == cand.route[-1], "pipeline smoke: star route is closed")

    ledger_checks()
    dispatch_checks()
    get_route_checks()

    print("\nall routing checks passed")


def get_route_checks():
    """The get_route() library entry point: API surface always; a real route
    when osmnx + a cached GraphML are available (skipped cleanly otherwise, so
    the offline test suite still passes)."""
    import os
    import svg2gpx

    check(callable(svg2gpx.get_route), "get_route is exported and callable")
    fields = svg2gpx.RouteResult.__dataclass_fields__
    for f in ("latlon", "distance_km", "iou", "frechet", "features_latlon"):
        check(f in fields, f"RouteResult has field '{f}'")

    # A bad shape fails fast (before any network work), with a clear error.
    try:
        svg2gpx.get_route(0.0, 0.0, "definitely_not_a_shape_xyz")
        check(False, "get_route should reject an unknown shape")
    except FileNotFoundError:
        check(True, "get_route rejects an unknown shape with FileNotFoundError")

    graphml = os.path.join("data", "Chicago_Network.graphml")
    try:
        import osmnx  # noqa: F401
    except ImportError:
        print("  skip: osmnx not installed -- get_route network smoke skipped")
        return
    if not os.path.exists(graphml):
        print(f"  skip: {graphml} not found -- get_route network smoke skipped")
        return

    route = svg2gpx.get_route(41.9285, -87.7075, "star", graphml=graphml,
                              radius_m=1600, seed=42)
    check(isinstance(route, svg2gpx.RouteResult), "get_route returns a RouteResult")
    check(route.latlon.ndim == 2 and route.latlon.shape[1] == 2,
          "get_route .latlon is an (N, 2) lat/lon array")
    check(bool((route.latlon[0] == route.latlon[-1]).all()),
          "get_route route is a closed loop")
    check(route.distance_km > 0, f"get_route distance_km positive ({route.distance_km:.1f})")
    check(-90 <= route.latlon[:, 0].min() and route.latlon[:, 0].max() <= 90,
          "get_route latitudes are in range")


def dispatch_checks():
    """Engine dispatch: compactness separates the families and auto routes
    compact shapes to the recipe engine, protrusive ones to classic."""
    heart = gen.extract_contour(shape_path("heart"), 512)
    star = gen.extract_contour(shape_path("star"), 512)
    c_heart, c_star = gen.shape_compactness(heart), gen.shape_compactness(star)
    check(c_heart < 2.0 < c_star,
          f"compactness separates heart ({c_heart:.2f}) from star ({c_star:.2f})")

    cfg = dict(gen.CONFIG)
    d_heart = gen._dispatch_engine(heart, cfg)
    d_star = gen._dispatch_engine(star, cfg)
    check(d_heart["engine"] == "recipe" and d_heart["bend_template"],
          "auto dispatch: heart -> recipe engine (bend on)")
    check(d_star["engine"] == "classic" and not d_star["bend_template"]
          and d_star["template_vertices"] is None,
          "auto dispatch: star -> classic engine (raw template, no bend)")
    same = gen._dispatch_engine(star, {**cfg, "engine": None})
    check("template_vertices" in same and same.get("engine") is None,
          "engine=None disables dispatch (cfg used as-is)")


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
