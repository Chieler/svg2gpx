"""
SVG -> running route on a city street grid.

Given an SVG shape and a city location, this builds a closed running route on
the real walkable street network whose outline resembles the SVG.

Pipeline
--------
    1. build_grid       Pull + normalize the walkable street network (+ parks).
    2. extract_contour  Render the SVG and trace its outer outline as a polyline.
    3. search_placement  Find the scale / rotation / offset that lays the shape
                         over the streets with the smallest snap distance.
    4. snap_waypoints   Densify the placed outline and snap each point to the
                         nearest street node -> a dense sequence of target nodes.
    5. route_contour    Walk consecutive targets with a contour-biased Dijkstra
                         so the path hugs the shape instead of cutting corners.
    6. cleanup + plot   Close the loop, dissolve backtracks / loops / nooks,
                         report fidelity, draw.

Fidelity comes from *dense* waypoints: spacing the targets well below one block
means each Dijkstra hop is short and has almost no room to deviate from the
outline. This replaces the old approach of sparse waypoints + heavy post-hoc
spike/loop repair, and folds three near-identical routers into one.

Tune everything from CONFIG. Run:  python script.py
"""

import heapq
from dataclasses import dataclass

import numpy as np
import cv2
import skia
from scipy.spatial import cKDTree
from shapely.geometry import LineString

# osmnx (live OSM fetch) and matplotlib (plotting) are imported lazily inside the
# functions that need them -- build_grid and plot* -- so that callers which only
# reuse the geometry/routing/metric helpers (e.g. benchmark.py on a synthetic
# grid) don't pay for those heavy, network-oriented deps just to import this.




# --------------------------------------------------------------------------- #
# Street grid                                                                  #
# --------------------------------------------------------------------------- #
@dataclass
class Grid:
    """Everything the router needs about the street network, in [0, 1] space."""
    graph: dict          # node -> list[(neighbor, edge_length)]
    node_keys: list      # ordered node tuples (row i <-> nodes_arr[i])
    nodes_arr: np.ndarray
    tree: cKDTree
    avg_edge: float
    edge_list: list      # (p1, p2) pairs, for plotting only
    span: float          # projected-meters span of one [0, 1] unit
    x_min: float
    y_min: float
    grid_angle: float    # dominant street orientation, radians (period 90 deg)


def build_grid(cfg):
    """Fetch (or load) the walkable network, normalize to [0, 1], index it.

    Two sources, same result:
      * live fetch (default)     -- ox.graph_from_point around lat/lng, or
      * local GraphML (offline)  -- cfg["graphml_path"]: a previously saved
        ox.save_graphml network (e.g. a whole-city snapshot), cropped to the
        same lat/lng + radius_m window. This makes real-map runs reproducible
        and usable where the Overpass API is slow, rate-limited or unreachable.
    """
    import osmnx as ox
    if cfg.get("graphml_path"):
        G = ox.load_graphml(cfg["graphml_path"])
        bbox = ox.utils_geo.bbox_from_point((cfg["lat"], cfg["lng"]),
                                            dist=cfg["radius_m"])
        G = ox.truncate.truncate_graph_bbox(G, bbox)
    else:
        G = ox.graph_from_point((cfg["lat"], cfg["lng"]), dist=cfg["radius_m"],
                                network_type="walk", simplify=True)
    return grid_from_graph(ox.project_graph(G), cfg)


def grid_from_graph(G_proj, cfg):
    """Turn a projected osmnx graph into the router's normalized Grid."""
    import osmnx as ox
    nodes, _ = ox.graph_to_gdfs(G_proj)

    xs, ys = nodes.geometry.x.values, nodes.geometry.y.values
    x_min, y_min = xs.min(), ys.min()
    span = max(xs.max() - x_min, ys.max() - y_min)

    coord = {nid: ((row.geometry.x - x_min) / span, (row.geometry.y - y_min) / span)
             for nid, row in nodes.iterrows()}

    graph, edge_list = {}, []
    step = cfg["node_spacing"] / span if cfg["densify_streets"] else None

    def link(a, b):
        d = float(np.hypot(a[0] - b[0], a[1] - b[1]))
        graph.setdefault(a, []).append((b, d))
        graph.setdefault(b, []).append((a, d))
        edge_list.append((a, b))

    # osmnx returns a MultiDiGraph: a two-way street appears as both u->v and
    # v->u (and occasionally as parallel edges). link() already wires both
    # directions, so process each node pair once -- this halves adjacency size
    # (faster Dijkstra neighbor scans) and stops avg_edge double-counting.
    seen_pairs = set()
    for u, v, data in G_proj.edges(data=True):
        if u not in coord or v not in coord or u == v:
            continue
        pair = frozenset((u, v))
        if pair in seen_pairs:
            continue
        seen_pairs.add(pair)
        p1, p2 = coord[u], coord[v]
        if step is None:
            link(p1, p2)
            continue
        geom = data.get("geometry")
        chain = _densify_edge(p1, p2, list(geom.coords) if geom is not None else None,
                              x_min, y_min, span, step)
        for a, b in zip(chain, chain[1:]):
            link(a, b)

    avg_edge = float(np.mean([d for nbrs in graph.values() for _, d in nbrs]))

    if cfg["include_parks"]:
        try:
            parks = ox.features_from_point((cfg["lat"], cfg["lng"]),
                                           dist=cfg["radius_m"], tags={"leisure": "park"})
            parks = parks.to_crs(G_proj.graph["crs"])
            n = _add_park_mesh(graph, edge_list, parks, x_min, y_min, span, avg_edge, cfg)
            print(f"park mesh nodes added: {n}")
        except Exception as exc:  # parks are a bonus, never fatal
            print(f"skipping parks ({exc})")

    node_keys = list(graph.keys())
    nodes_arr = np.array(node_keys, dtype=np.float64)
    grid_angle = _dominant_orientation(edge_list)
    print(f"grid: {len(node_keys)} nodes, {len(edge_list)} edges, "
          f"avg edge {avg_edge:.4f}, grid angle {np.degrees(grid_angle):.1f} deg")
    return Grid(graph, node_keys, nodes_arr, cKDTree(nodes_arr),
                avg_edge, edge_list, span, x_min, y_min, grid_angle)


def _dominant_orientation(edge_list, sample=6000):
    """Dominant street bearing in radians, with 90 deg period (a grid has two).

    Each edge angle is mapped through exp(i*4*theta) so directions 90 deg apart
    reinforce rather than cancel; the mean's argument gives the grid's tilt. This
    lets placement search rotate a shape so its long edges run *along* streets
    instead of fighting them into staircases.
    """
    e = edge_list if len(edge_list) <= sample else \
        [edge_list[i] for i in np.linspace(0, len(edge_list) - 1, sample).astype(int)]
    v = np.array([(b[0] - a[0], b[1] - a[1]) for a, b in e], dtype=np.float64)
    ang = np.arctan2(v[:, 1], v[:, 0])
    mean = np.mean(np.exp(4j * ang))
    return float(np.angle(mean) / 4.0)


def _densify_edge(p_u, p_v, geom_coords, x_min, y_min, span, step):
    """Subdivide one street edge into a chain of nodes ~`step` apart (normalized).

    Follows the edge's real geometry when OSMnx kept it (curved streets), else a
    straight line. The endpoints are pinned to the exact intersection nodes
    (``p_u``, ``p_v``) so chains from different edges still meet at shared nodes.
    """
    if geom_coords and len(geom_coords) >= 2:
        poly = np.array([[(x - x_min) / span, (y - y_min) / span] for x, y in geom_coords])
        if np.hypot(*(poly[0] - np.array(p_u))) > np.hypot(*(poly[-1] - np.array(p_u))):
            poly = poly[::-1]                      # orient u -> v
        poly[0], poly[-1] = np.array(p_u), np.array(p_v)
    else:
        poly = np.array([p_u, p_v], dtype=np.float64)

    seg = np.linalg.norm(np.diff(poly, axis=0), axis=1)
    total = float(seg.sum())
    if total < 1e-12:
        return [(float(p_u[0]), float(p_u[1])), (float(p_v[0]), float(p_v[1]))]
    n = max(int(total / step) + 1, 2)
    arc = np.concatenate([[0.0], np.cumsum(seg)])
    t = np.linspace(0.0, total, n)
    x, y = np.interp(t, arc, poly[:, 0]), np.interp(t, arc, poly[:, 1])
    chain = [(float(x[i]), float(y[i])) for i in range(n)]
    chain[0] = (float(p_u[0]), float(p_u[1]))      # pin exact shared endpoints
    chain[-1] = (float(p_v[0]), float(p_v[1]))
    return chain


def _contains_xy(geom, X, Y):
    """Vectorized point-in-polygon, across shapely 2.x and 1.8 layouts."""
    try:
        from shapely import contains_xy          # shapely >= 2.0
        return contains_xy(geom, X, Y)
    except Exception:
        from shapely.vectorized import contains   # shapely 1.8
        return contains(geom, X, Y)


def _add_park_mesh(graph, edge_list, parks, x_min, y_min, span, avg_edge, cfg):
    """Fill park polygons with a routable lattice tied into the street grid.

    The previous version sampled at one-edge pitch and tested each candidate with
    a Python ``prep(geom).contains`` loop; at that pitch most small city parks got
    zero interior points (their bounding box stepped right over them), so no mesh
    appeared. This samples at ``park_spacing`` (default half an edge), tests the
    whole lattice with one vectorized call, and wires neighbors with ``query_pairs``.
    """
    if parks is None or len(parks) == 0:
        return 0
    pitch = avg_edge * cfg["park_spacing"]   # lattice pitch in [0, 1] units
    step_m = pitch * span                    # same pitch in projected meters
    street_keys = list(graph.keys())
    street_arr = np.array(street_keys, dtype=np.float64)
    street_tree = cKDTree(street_arr)
    added = 0

    for geom in parks.geometry:
        if geom is None or geom.is_empty or geom.geom_type not in ("Polygon", "MultiPolygon"):
            continue
        minx, miny, maxx, maxy = geom.bounds
        gx = np.arange(minx, maxx + step_m, step_m)
        gy = np.arange(miny, maxy + step_m, step_m)
        if len(gx) < 1 or len(gy) < 1:
            continue
        GX, GY = np.meshgrid(gx, gy)
        inside = _contains_xy(geom, GX.ravel(), GY.ravel())
        px, py = GX.ravel()[inside], GY.ravel()[inside]
        if len(px) == 0:
            continue

        mesh = [((x - x_min) / span, (y - y_min) / span) for x, y in zip(px, py)]
        for n in mesh:
            graph.setdefault(n, [])
        added += len(mesh)

        marr = np.array(mesh, dtype=np.float64)
        mtree = cKDTree(marr)
        # Lattice edges (4- and 8-neighbours within ~1.5 pitch), one batched call.
        for i, j in mtree.query_pairs(pitch * 1.5):
            n1, n2 = mesh[i], mesh[j]
            d = float(np.hypot(n1[0] - n2[0], n1[1] - n2[1]))
            graph[n1].append((n2, d))
            graph[n2].append((n1, d))
            edge_list.append((n1, n2))
        # Tie every mesh node to its nearest street node so the patch is enterable.
        dists, ks = street_tree.query(marr)
        for n, d, k in zip(mesh, dists, ks):
            if d < pitch * 3:
                s = street_keys[int(k)]
                graph[n].append((s, float(d)))
                graph[s].append((n, float(d)))
                edge_list.append((n, s))
    return added


# --------------------------------------------------------------------------- #
# Contour extraction                                                           #
# --------------------------------------------------------------------------- #
@dataclass
class Feature:
    """One inner feature of a shape, in the same [0, 1] frame as the outline.

    closed=True  -- a loop (a hole like an eye, or a separate drawn element);
    closed=False -- an open path (an interior detail stroke, e.g. a wing line),
                    which routing can trace as an out-and-back.
    """
    pts: np.ndarray
    closed: bool


@dataclass
class ShapeSpec:
    """A shape as the pipeline sees it: outer outline + inner features."""
    outer: np.ndarray    # closed [0, 1] polyline (what placement seats on streets)
    inners: list         # list[Feature], same coordinate frame as `outer`


def _render_ink_mask(svg_path, img_size):
    """Rasterize the SVG and return the binary ink mask (ink=255, paper=0)."""
    surface = skia.Surface(img_size, img_size)
    with surface as canvas:
        canvas.clear(skia.ColorWHITE)
        with open(svg_path, 'rb') as f:
            data = f.read()
        stream = skia.MemoryStream(data)
        svg = skia.SVGDOM.MakeFromStream(stream)
        svg.setContainerSize(skia.Size(img_size, img_size))
        svg.render(canvas)

    arr = np.array(surface.makeImageSnapshot().toarray())
    gray = cv2.cvtColor(arr, cv2.COLOR_BGRA2GRAY)
    _, binary = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY_INV)
    return binary


def _norm_pts(pts_px, img_size):
    """Pixel coords -> [0, 1] with Y flipped to a math-up axis (north-up grid)."""
    pts_px = np.asarray(pts_px, dtype=np.float64)
    return np.column_stack([pts_px[:, 0] / img_size, 1.0 - pts_px[:, 1] / img_size])


def _cycle_runs(mask):
    """Contiguous True runs on a cyclic boolean mask, as (start, stop) inclusive
    index pairs in original order (stop may wrap below start). None if all True."""
    n = len(mask)
    if mask.all():
        return None
    if not mask.any():
        return []
    start = int(np.argmin(mask))               # rotate so position 0 is False
    m = np.roll(mask, -start)
    runs, i = [], 0
    while i < n:
        if m[i]:
            j = i
            while j < n and m[j]:
                j += 1
            runs.append(((i + start) % n, (j - 1 + start) % n))
            i = j
        else:
            i += 1
    return runs


def _cycle_slice(pts, a, b):
    """pts[a..b] inclusive on a cyclic array (handles wrap-around)."""
    return pts[a:b + 1] if a <= b else np.vstack([pts[a:], pts[:b + 1]])


def _arc_len(pts):
    return float(np.linalg.norm(np.diff(np.asarray(pts, np.float64), axis=0),
                                axis=1).sum())


def extract_shape(svg_path, img_size, min_perimeter=0.05, dup_tol=None):
    """Render the SVG and extract the outer outline AND its inner features.

    The outer outline is what the old extract_contour returned (flood-filled
    solid silhouette, largest external contour). Inner features are everything
    the silhouette step used to throw away, found from the full ink/paper
    contour *tree* of the raw mask (depth alternates ink edge / hole):

      * depth 0 besides the main outline  -- completely disconnected elements;
      * even depth >= 2                   -- ink drawn inside a hole (an eye dot,
                                             an emblem), also possibly disconnected
                                             from the outer contour;
      * odd depth (white pockets)         -- classified against their parent
        ink edge. A pocket much shorter than its parent (< 35% of its arc) can
        only be a true hole (a shark's eye, a donut hole) and is kept closed.
        A parent-sized pocket is either the interior of a stroked outline (it
        shadows the parent at pen-stroke distance everywhere -> dropped), a fat
        ring's hole (it sits far from the parent everywhere -> kept closed), or
        an interior partitioned by detail strokes -- then exactly the arcs that
        deviate from the parent are kept as OPEN paths: those arcs run along
        the drawn interior lines (a wing line, a mane).

    Finally near-duplicates are removed (the two edges of one stroked line, or
    the same dividing stroke seen from two adjacent pockets): any feature lying
    entirely within the stroke-width tolerance of a longer kept feature (or of
    the outline) is dropped.

    `min_perimeter` (in [0, 1] units) filters raster noise; `dup_tol` (same
    units) overrides the stroke-width-derived duplicate tolerance.
    """
    binary = _render_ink_mask(svg_path, img_size)

    # Outer outline: identical to the historical behaviour (solidify, then the
    # largest external contour).
    holes = binary.copy()
    cv2.floodFill(holes, np.zeros((img_size + 2, img_size + 2), np.uint8), (0, 0), 255)
    solid = cv2.bitwise_or(binary, cv2.bitwise_not(holes))
    ext, _ = cv2.findContours(solid, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not ext:
        raise ValueError(f"no drawable outline found in {svg_path!r} "
                         "(empty or unsupported SVG)")
    outer = _close(_norm_pts(max(ext, key=cv2.contourArea)[:, 0, :], img_size))

    # Full boundary tree of the raw ink mask, at pixel density.
    contours, hier = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    if hier is None:
        return ShapeSpec(outer, [])
    hier = hier[0]
    pts_of = [c[:, 0, :].astype(np.float64) for c in contours]
    depth = []
    for i in range(len(contours)):
        d, p = 0, hier[i][3]
        while p != -1:
            d, p = d + 1, hier[p][3]
        depth.append(d)
    main = max((i for i in range(len(contours)) if depth[i] == 0),
               key=lambda i: cv2.contourArea(contours[i]))

    if dup_tol is None:
        # Pen-stroke thickness ~ 4x the median ink-pixel depth (a stroke's
        # distance-to-paper profile is triangular, so its median is ~1/4 of the
        # full thickness). Capped for filled shapes, where ink depth is
        # body-scale rather than stroke-scale.
        dt = cv2.distanceTransform(binary, cv2.DIST_L2, 3)
        ink = dt[binary > 0]
        thickness = 4.0 * float(np.median(ink)) if len(ink) else 0.0
        tol_px = min(1.3 * thickness, 0.04 * img_size)
    else:
        tol_px = dup_tol * img_size
    min_px = min_perimeter * img_size

    candidates = []                                # (pts_px, closed)
    for i, pts in enumerate(pts_of):
        d = depth[i]
        if i == main or len(pts) < 4:
            continue
        if d % 2 == 0:                             # ink edge: a drawn element
            if _arc_len(pts) + np.linalg.norm(pts[0] - pts[-1]) >= min_px:
                candidates.append((pts, True))
            continue
        # White pocket: hole, stroked-outline interior, or detail-stroke carrier?
        parent = pts_of[hier[i][3]]
        if _arc_len(pts) < 0.35 * _arc_len(parent):
            candidates.append((pts, True))         # far too short to be "the
            continue                               # interior": a true hole
        # Parent-sized pocket. Its hugging distance (low percentile of the
        # point distances to the parent) is the local pen-stroke gap; deviation
        # beyond that marks drawn interior lines. The parent-extent cap keeps a
        # fat ring's hole (donut) from being mistaken for a shadowing interior.
        dist = cKDTree(parent).query(pts)[0]
        extent = float(max(np.ptp(parent[:, 0]), np.ptp(parent[:, 1])))
        tol_pocket = min(1.6 * float(np.percentile(dist, 15)), 0.15 * extent)
        runs = _cycle_runs(dist > tol_pocket)
        if runs is None:                           # far from parent everywhere:
            candidates.append((pts, True))         # a real hole (donut)
            continue
        for a, b in runs:
            seg = _cycle_slice(pts, a, b)
            if _arc_len(seg) >= min_px:
                candidates.append((seg, False))    # interior detail stroke

    # Drop near-duplicates: longest first, keep a feature only if some part of
    # it is farther than tol from everything kept so far (incl. the outline).
    # The tolerance is capped by the candidate's own extent: a duplicate edge
    # (the second side of a stroke) parallels something its own size, whereas a
    # small hole that merely sits NEAR the outline (a shark's eye at high
    # raster resolution) must not be swallowed by the stroke-width tolerance.
    candidates.sort(key=lambda t: -_arc_len(t[0]))
    kept, refs = [], [pts_of[main]]
    for pts, closed in candidates:
        extent = float(max(np.ptp(pts[:, 0]), np.ptp(pts[:, 1])))
        eff_tol = min(tol_px, 0.5 * extent)
        if any(cKDTree(r).query(pts)[0].max() < eff_tol for r in refs):
            continue
        kept.append((pts, closed))
        refs.append(pts)

    inners = [Feature(_close(_norm_pts(p, img_size)) if closed
                      else _norm_pts(p, img_size), closed)
              for p, closed in kept]
    return ShapeSpec(outer, inners)


def extract_contour(svg_path, img_size):
    """Render the SVG, trace its outer outline, return a closed [0, 1] polyline.

    Y is flipped to a math-up axis so the outline aligns with the (north-up)
    projected street grid. (Outline only -- use extract_shape() to also get the
    inner features.)
    """
    binary = _render_ink_mask(svg_path, img_size)

    # Flood from a corner then OR back in, so interior holes are treated as solid.
    holes = binary.copy()
    cv2.floodFill(holes, np.zeros((img_size + 2, img_size + 2), np.uint8), (0, 0), 255)
    solid = cv2.bitwise_or(binary, cv2.bitwise_not(holes))

    contours, _ = cv2.findContours(solid, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise ValueError(f"no drawable outline found in {svg_path!r} "
                         "(empty or unsupported SVG)")
    outer = max(contours, key=cv2.contourArea)
    return _close(_norm_pts(outer[:, 0, :], img_size))



# --------------------------------------------------------------------------- #
# Geometry helpers                                                             #
# --------------------------------------------------------------------------- #
def _close(pts):
    """Ensure a polyline ends where it starts."""
    pts = np.asarray(pts, dtype=np.float64)
    if not np.allclose(pts[0], pts[-1]):
        pts = np.vstack([pts, pts[0]])
    return pts


def resample(pts, n=None, step=None, closed=True):
    """Evenly respace a polyline by point count or arc-length step.

    closed=True (default) treats `pts` as a loop, appending the start point if
    needed; closed=False resamples an open path between its endpoints.
    """
    pts = _close(pts) if closed else np.asarray(pts, dtype=np.float64)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    arc = np.concatenate([[0.0], np.cumsum(seg)])
    total = arc[-1]
    if total < 1e-12:
        return pts[:1]
    if step is not None:
        n = max(int(total / step), 3)
    targets = np.linspace(0.0, total, n)
    x = np.interp(targets, arc, pts[:, 0])
    y = np.interp(targets, arc, pts[:, 1])
    return np.column_stack([x, y])


def place(pts, scale, rot_deg, dx, dy, aspect=1.0, shear=0.0, center=None):
    """Affine-place a polyline about its centroid, then translate.

    Beyond similarity (scale + rot + translate) this adds two shape-fitting DOF:
      * aspect -- area-preserving anisotropic stretch (diag(a, 1/a)); lets the
                  shape elongate to match Manhattan's ~3:1 blocks, which both fits
                  better and turns shallow-angle edges into aligned ones.
      * shear  -- a small skew, for grids that aren't perfectly orthogonal.
    aspect=1, shear=0 reduces exactly to the old similarity transform.

    `center` overrides the pivot: an inner feature must be placed about the
    OUTER contour's centroid (the same pivot its outline was placed about) so
    it lands where the drawing put it, not spun about its own middle.
    """
    pts = np.asarray(pts, dtype=np.float64)
    c = np.asarray(center, dtype=np.float64) if center is not None else pts.mean(axis=0)
    a = np.radians(rot_deg)
    R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    H = np.array([[1.0, shear], [0.0, 1.0]])
    Sc = np.array([[scale * aspect, 0.0], [0.0, scale / aspect]])
    M = R @ H @ Sc
    return (pts - c) @ M.T + c + np.array([dx, dy])


def waypoint_importance(pts, k=4):
    """Per-point importance in [0, 1]: how much each point *defines* the shape.

    A point on a straight run can be dropped without changing the outline, so it
    scores ~0. A corner or the tip of a protrusion (a beak, leg, tail) sits far
    off the chord joining its neighbours `k` steps away, so it scores ~1. This is
    the perpendicular distance to that chord -- the same notion of significance
    RDP uses -- and it is what lets the rest of the pipeline treat a beak point
    as worth far more than a redundant point on a flat edge.
    """
    pts = np.asarray(pts, dtype=np.float64)
    n = len(pts)
    if n < 2 * k + 1:
        return np.zeros(n)
    a = np.roll(pts, k, axis=0)                    # neighbour k steps behind
    b = np.roll(pts, -k, axis=0)                   # neighbour k steps ahead
    chord = b - a
    L = np.hypot(chord[:, 0], chord[:, 1])
    # perpendicular distance from p to the line through a,b (2*area / base)
    cross = chord[:, 0] * (a[:, 1] - pts[:, 1]) - chord[:, 1] * (a[:, 0] - pts[:, 0])
    imp = np.where(L > 1e-12, np.abs(cross) / np.where(L > 1e-12, L, 1.0), 0.0)
    peak = imp.max()
    return imp / peak if peak > 0 else imp


# --------------------------------------------------------------------------- #
# Placement search                                                            #
# --------------------------------------------------------------------------- #
def _score(placed, grid, scale, cfg, importance,
           feat_pts=None, feat_w=None, feat_share=0.0):
    """How routable a placement is, weighted by how much each point matters.

    Coverage and snap distance are weighted by `importance`, so seating a flat
    edge on streets counts for little while seating a beak tip counts for a lot.
    A dedicated worst-feature term makes the score collapse if any high-importance
    point is left off-grid -- this is what stops the search from cheating
    protrusions to make the bulk body sit prettily.

    When the shape has inner features, `feat_pts` carries a few sample points
    per feature placed with the same transform (weighted by feature size via
    `feat_w`). Their snap closeness and node resolvability join the score as a
    bonus scaled by `feat_share` (the features' share of drawn length), so
    stage 1 already prefers placements whose eyes land on streets -- otherwise
    the expensive stage 2 may never get to see a feature-friendly candidate.
    """
    m = cfg["margin"]
    if placed.min() < m or placed.max() > 1 - m:
        return -np.inf
    d, nn = grid.tree.query(placed)
    # No-ocean guard: too much of the outline far from any street node means it
    # spills into water / off-network void -> reject outright.
    if np.mean(d < grid.avg_edge * cfg["land_reach"]) < cfg["min_land_fraction"]:
        return -np.inf
    w = importance + 0.05                                  # floor: flats still count a little
    on_street = (d < grid.avg_edge * 0.5).astype(float)
    coverage = float(np.sum(w * on_street) / np.sum(w))   # importance-weighted coverage
    feat_snap = float(np.sum(w * d) / np.sum(w))           # importance-weighted snap dist
    closeness = 1.0 / (1.0 + feat_snap / grid.avg_edge)
    feat = importance > 0.6                                # the defining points
    worst = float(d[feat].max()) if feat.any() else float(d.mean())
    feature = 1.0 / (1.0 + worst / grid.avg_edge)          # tanks if a feature is stranded
    # Resolvability: the defining points must snap to *distinct* nodes. If a whole
    # feature (e.g. both lips of a mouth) collapses onto one node it can't be drawn,
    # so this rewards placements/scales where features land on their own streets.
    resolvable = (len(np.unique(nn[feat])) / int(feat.sum())) if feat.any() else 1.0
    # Orientation: a grid renders an edge cleanly when it is EITHER exactly on an
    # axis (0/90 deg -> a straight line) OR near 45 deg (a fine even crisscross).
    # The ugly case is the *shallow* band in between: a ~10 deg edge staircases as
    # cot(10 deg) ~= 6 blocks of parallel run, then one big 90 deg jog. The old
    # term rewarded "near axis" and so loved exactly that band; this one rewards
    # 0 and 45 and punishes the shallow zone. delta = angle to nearest axis.
    tang = np.diff(np.vstack([placed, placed[:1]]), axis=0)
    ang = np.arctan2(tang[:, 1], tang[:, 0])
    delta = (ang - grid.grid_angle) % (np.pi / 2)
    delta = np.minimum(delta, np.pi / 2 - delta)              # fold to [0, 45 deg]
    quality = np.where(delta < np.radians(2.5), 1.0, np.sin(2.0 * delta))
    orientation = float(np.sum(w * quality) / np.sum(w))
    s_lo, s_hi = cfg["scale_range"]
    bigness = (scale - s_lo) / (s_hi - s_lo)               # prefer larger
    score = (0.22 * coverage + 0.15 * closeness + 0.20 * feature
             + 0.13 * resolvable + 0.18 * orientation + 0.12 * bigness)
    if feat_pts is not None and len(feat_pts):
        fd, fnn = grid.tree.query(feat_pts)
        feat_close = 1.0 / (1.0 + float(np.average(fd, weights=feat_w))
                            / grid.avg_edge)
        feat_resolve = len(np.unique(fnn)) / len(feat_pts)
        score += (cfg.get("inner_proxy_weight", 0.15) * feat_share
                  * (0.6 * feat_close + 0.4 * feat_resolve))
    return score


def _placement_far(a, b, drot=22.0, doff=0.12, dscale=0.15, daspect=0.2):
    """True if two placements are visibly distinct (params may be 4- or 6-tuples)."""
    s1, r1, x1, y1 = a[:4]
    s2, r2, x2, y2 = b[:4]
    a1, a2 = (a[4] if len(a) > 4 else 1.0), (b[4] if len(b) > 4 else 1.0)
    return (abs((r1 - r2 + 180) % 360 - 180) > drot
            or np.hypot(x1 - x2, y1 - y2) > doff
            or abs(s1 - s2) > dscale
            or abs(a1 - a2) > daspect)


def build_route(grid, placed, cfg):
    """Run the full snap -> route -> cleanup pipeline for one placement."""
    dense, waypoints, wp_idx = snap_waypoints(placed, grid, cfg)
    if len(waypoints) < 3:
        return [], dense, waypoints
    w = cfg["deviation_weight"]
    route = route_contour(grid, dense, waypoints, wp_idx, w)
    route = cleanup(grid, route, dense, wp_idx, w, cfg)
    return route, dense, waypoints


@dataclass
class Candidate:
    """One evaluated placement: the outer route plus its routed inner features."""
    cost: float          # combined cost: outer fidelity + weighted feature term
    placed: np.ndarray   # placed outer contour
    route: list          # routed outer loop
    feats: list          # [(Feature, placed_pts, route), ...]; route may be []


def build_feature_route(grid, placed_feat, closed, cfg):
    """Route one placed inner feature: a loop if closed, an open path if not.

    An open path is a drawn interior line (a wing line); the runner traces it
    and returns the same way (out-and-back), so the routed polyline is the
    single pass. Returns [] when the feature is too small to resolve on this
    grid (e.g. an eye smaller than a block).

    Cleanup's corner-cut slack is sized in street-edge units for the OUTER
    shape; on a feature a few blocks wide that budget spans the whole feature
    and shortcut_nooks would legally smooth an eye into a triangle. Raise the
    effective granularity until the slack is a small fraction of the feature's
    own span (never below the configured granularity, so big features keep the
    user's smoothing).
    """
    span = float(max(np.ptp(placed_feat[:, 0]), np.ptp(placed_feat[:, 1])))
    full = grid.avg_edge * 3.5                      # slack at granularity 0
    g = cfg["granularity"]
    if full > 0 and 0.12 * span < full * (1.0 - g) ** 0.6:
        g_feat = 1.0 - (0.12 * span / full) ** (1.0 / 0.6)
        cfg = {**cfg, "granularity": float(np.clip(g_feat, g, 1.0))}
    dense, waypoints, wp_idx = snap_waypoints(placed_feat, grid, cfg, closed=closed)
    if len(waypoints) < (3 if closed else 2):
        return []
    w = cfg["deviation_weight"]
    route = route_contour(grid, dense, waypoints, wp_idx, w)
    return cleanup(grid, route, dense, wp_idx, w, cfg, close=closed)


def _feat_deviation(route, target):
    """Mean two-way deviation between a feature's route and its target."""
    r = np.asarray(route, np.float64)
    t = np.asarray(target, np.float64)
    return 0.5 * (float(cKDTree(r).query(t)[0].mean())
                  + float(cKDTree(t).query(r)[0].mean()))


def refine_feature(grid, fp0, closed, cfg):
    """Tailor one placed feature to the street fabric around its drawn spot.

    The global placement seats the OUTER contour; a small feature can land
    astride a block where nothing matches its outline even though a perfect
    seat exists one street over. So rather than blind nudges, search the local
    fabric, mirroring the outer two-stage search in miniature:

      * positions -- the drawn centroid plus every street node within
        `inner_search_radius` x the feature's span (a few blocks minimum),
      * rotations -- 0 and +/- `inner_rot_deg` (small, so an eyebrow stays an
        eyebrow),
      * scales -- 0.9 .. 1.35, plus a rescue upscale for a feature narrower
        than `inner_min_span_blocks` street edges (capped `inner_max_inflate`x;
        a slightly-too-big eye that READS beats a faithful invisible one).

    Every variant gets a cheap street-fit proxy: snap closeness, on-street
    coverage, and crucially how many DISTINCT nodes the outline resolves to --
    tiny features die by collapsing onto one node. The proxy's best few are
    actually routed; the winner minimizes routed deviation (against its own
    moved target, so a bigger target must pay its own way) plus a drift
    penalty that anchors the feature to where the drawing put it.
    Returns (target, route) -- the original, unrouted, on failure.
    """
    e = grid.avg_edge
    c0 = fp0.mean(axis=0)
    span = float(max(np.ptp(fp0[:, 0]), np.ptp(fp0[:, 1])))
    if span <= 0.0:
        return fp0, []
    proxy_pts = resample(fp0, n=48, closed=closed) - c0    # centered template

    # Candidate seats: the drawn spot plus nearby street nodes, nearest first.
    radius = max(cfg.get("inner_search_radius", 1.0) * span, 3.0 * e)
    near = grid.tree.query_ball_point(c0, radius)
    nodes = grid.nodes_arr[near]
    if len(nodes):
        order = np.argsort(np.linalg.norm(nodes - c0, axis=1))
        nodes = nodes[order[:36]]                          # cap the fan-out
    centers = [c0] + list(nodes)

    rot = np.radians(cfg.get("inner_rot_deg", 12.0))
    rots = (0.0, rot, -rot) if rot > 0 else (0.0,)
    scales = {0.9, 1.0, 1.15, 1.35}
    min_span = cfg.get("inner_min_span_blocks", 6.0) * e
    if span < min_span:
        scales.add(min(min_span / span, cfg.get("inner_max_inflate", 3.0)))

    rescue = max(scales) if span < min_span else None
    scored = []
    for a in rots:
        R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
        rotated = proxy_pts @ R.T
        for s in sorted(scales):
            base = rotated * s
            for c in centers:
                p = base + c
                d, nn = grid.tree.query(p)
                closeness = 1.0 / (1.0 + float(d.mean()) / e)
                coverage = float(np.mean(d < 0.5 * e))
                # Distinct snapped nodes, absolute: below ~12 a loop can't
                # look like anything, however faithfully it sits.
                resolve = min(len(np.unique(nn)) / 12.0, 1.0)
                shift = float(np.hypot(*(c - c0)))
                proxy = (0.40 * closeness + 0.25 * coverage + 0.35 * resolve
                         - 0.3 * shift / max(span, e))
                drift = shift + 0.15 * abs(s - 1.0) * span
                scored.append((proxy, a, s, c, drift))
    scored.sort(key=lambda t: -t[0])

    # Route the proxy's favourites, but structurally guarantee two fallbacks a
    # mistuned proxy must never starve: the drawn identity variant, and (for a
    # sub-resolution feature) the best-ranked rescue-scale variant.
    chosen = list(scored[:cfg.get("inner_route_eval", 6)])
    identity = next(t for t in scored if t[2] == 1.0 and t[1] == 0.0
                    and t[3] is centers[0])
    if identity not in chosen:
        chosen.append(identity)
    if rescue is not None and not any(t[2] == rescue for t in chosen):
        chosen.append(next(t for t in scored if t[2] == rescue))

    best = None
    for proxy, a, s, c, drift in chosen:
        R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
        fp = (fp0 - c0) @ R.T * s + c
        fr = build_feature_route(grid, fp, closed, cfg)
        if len(fr) < 2:
            continue
        score = _feat_deviation(fr, fp) + 0.3 * drift
        if best is None or score < best[0]:
            best = (score, fp, fr)
    if best is None:
        return fp0, []
    return best[1], best[2]


def route_features(grid, inners, params, center, cfg):
    """Place every inner feature with the outer contour's transform and route it.

    `center` is the outer contour's centroid: features get the full affine the
    outline got (scale, rotation, aspect, shear) about the same pivot, so they
    land where the drawing put them. Each feature is then locally refined --
    snap/nudge/rescue-upscale variants, best routed one wins (refine_feature).
    """
    feats = []
    for f in inners:
        fp = place(f.pts, *params, center=center)
        if cfg.get("inner_refine", True):
            fp, fr = refine_feature(grid, fp, f.closed, cfg)
        else:
            fr = build_feature_route(grid, fp, f.closed, cfg)
        feats.append((f, fp, fr))
    return feats


def feature_cost(grid, placed_outer, feats):
    """Score how well the inner features landed, for placement ranking.

    Returns (miss, weight). `miss` is the size-weighted mean per-feature cost
    in [0, 1]: 0 when every feature's route hugs its target, 1 for a feature
    that is unroutable here (off-grid, or collapsed below street resolution) --
    so placements that seat the body nicely but strand the eye rank below ones
    that draw both. `weight` is how much the features matter relative to the
    outline (total feature arc / outline arc, capped at 1): an eye dot can tip
    a close call but never outvote the body.
    """
    if not feats:
        return 0.0, 0.0
    sizes, costs = [], []
    for f, fp, fr in feats:
        sizes.append(_arc_len(fp))
        if len(fr) < 2:
            costs.append(1.0)
            continue
        costs.append(min(_feat_deviation(fr, fp) / (3.0 * grid.avg_edge), 1.0))
    total = float(sum(sizes))
    if total <= 0.0:
        return 0.0, 0.0
    weight = min(total / max(_arc_len(placed_outer), 1e-9), 1.0)
    return float(np.average(costs, weights=sizes)), weight


def search_placement(contour, grid, cfg, inners=None):
    """Rank placements by their *routed* fidelity; return the best DISTINCT ones.

    Two stages. (1) A fast geometric proxy (`_score`) ranks thousands of
    placements on snap distance, feature coverage, resolvability and grid-angle
    quality -- cheap, but blind to what the streets *between* anchors do.
    (2) The proxy's best *visibly distinct* placements are actually routed and
    judged by the composite match cost. When `inners` (the shape's inner
    features from extract_shape) are given, each of those top placements also
    places and routes the features with the same transform, and their routed
    fidelity joins the cost -- a good contour that strands its inner features
    loses to one that draws both. We return a ranked list of Candidates, not
    one winner, because a placement-determined artifact (a shallow-angle edge
    that staircases badly) can't be fixed downstream -- only a different
    placement escapes it, so the caller presents several to choose from.
    """
    rng = np.random.default_rng(cfg["seed"])
    base = resample(contour, n=250)          # cheap, transform-invariant proxy
    importance = waypoint_importance(base)   # which of those points define the shape
    s_lo, s_hi = cfg["scale_range"]
    lim = 0.5 - cfg["margin"]
    ln_a = np.log(cfg["aspect_max"])
    shm = cfg["shear_max"]

    # A few sample points per inner feature, placed with the outline's pivot,
    # so the stage-1 proxy can score feature seating too (see _score).
    inners = inners or []
    base_center = base.mean(axis=0)
    probe_feats, feat_w, feat_share = None, None, 0.0
    if inners:
        samples = [resample(f.pts, n=8, closed=f.closed)[:-1 if f.closed else None]
                   for f in inners]
        sizes = np.array([max(_arc_len(f.pts), 1e-9) for f in inners])
        probe_feats = np.vstack(samples)
        feat_w = np.concatenate([np.full(len(s), sz)
                                 for s, sz in zip(samples, sizes)])
        feat_share = min(float(sizes.sum()) / max(_arc_len(base), 1e-9), 1.0)

    def trial(scale, rot, dx, dy, aspect, shear):
        cand = place(base, scale, rot, dx, dy, aspect, shear)
        fp = (place(probe_feats, scale, rot, dx, dy, aspect, shear,
                    center=base_center)
              if probe_feats is not None else None)
        return (_score(cand, grid, scale, cfg, importance, fp, feat_w, feat_share),
                (scale, rot, dx, dy, aspect, shear))

    results = [trial(rng.uniform(s_lo, s_hi), rng.uniform(0, 360),
                     rng.uniform(-lim, lim), rng.uniform(-lim, lim),
                     float(np.exp(rng.uniform(-ln_a, ln_a))), rng.uniform(-shm, shm))
               for _ in range(cfg["n_random"])]
    results.sort(key=lambda r: r[0], reverse=True)

    for _ in range(cfg["n_refine"]):
        _, (s, r, dx, dy, asp, sh) = results[rng.integers(0, min(8, len(results)))]
        results.append(trial(
            float(np.clip(s + rng.normal(0, 0.04), s_lo, s_hi)),
            r + rng.normal(0, 6),
            dx + rng.normal(0, 0.03),
            dy + rng.normal(0, 0.03),
            float(np.clip(asp * np.exp(rng.normal(0, 0.05)), 1 / cfg["aspect_max"], cfg["aspect_max"])),
            float(np.clip(sh + rng.normal(0, 0.03), -shm, shm))))
    results.sort(key=lambda r: r[0], reverse=True)

    # Stage 2: route the proxy's best *distinct* placements (dedup first, so we
    # don't waste the budget on near-identical refinements), score each by the
    # perceptual placement cost (blurred render-compare) plus, when the shape
    # has inner features, their routed fidelity -- and return them ranked.
    center = np.asarray(contour, np.float64).mean(axis=0)
    w_inner = cfg.get("inner_cost_weight", 0.6)
    routed, tried, chosen_params = [], 0, []
    for proxy_score, params in results:
        if any(not _placement_far(params, q) for q in chosen_params):
            continue                                   # skip near-duplicate placement
        chosen_params.append(params)
        placed = place(contour, *params)
        route, _, _ = build_route(grid, placed, cfg)
        if len(route) >= 3:
            cost = placement_cost(route, placed)
            feats = route_features(grid, inners, params, center, cfg)
            note = ""
            if feats:
                miss, weight = feature_cost(grid, placed, feats)
                cost += w_inner * weight * miss
                note = (f" feat_miss={miss:.2f}x{weight:.2f} "
                        f"({sum(1 for _, _, fr in feats if len(fr) >= 2)}"
                        f"/{len(feats)} routed)")
            routed.append(Candidate(cost, placed, route, feats))
            print(f"  placement proxy={proxy_score:.3f} -> cost={cost:.4f} "
                  f"(scale={params[0]:.2f} rot={params[1]:.0f} "
                  f"aspect={params[4]:.2f} shear={params[5]:+.2f}){note}")
        tried += 1
        if tried >= cfg["n_route_eval"]:
            break

    if not routed:  # nothing routed; fall back to the proxy's top placement
        params = results[0][1]
        placed = place(contour, *params)
        return [Candidate(float("inf"), placed, build_route(grid, placed, cfg)[0],
                          route_features(grid, inners, params, center, cfg))]

    routed.sort(key=lambda c: c.cost)
    return routed


# --------------------------------------------------------------------------- #
# Snapping + routing                                                          #
# --------------------------------------------------------------------------- #
def snap_waypoints(placed, grid, cfg, closed=True):
    """Pick routing anchors off the outline and snap them to street nodes.

    The outline is densified once at a fine, fixed resolution (`dense`) -- this is
    the reference the router and cleanup measure against. From it we *select*
    anchors:

      * every defining point (corner / protrusion tip) is always taken, so a beak
        survives even at low granularity, and
      * filler points are added to cap the gap between anchors.

    Granularity controls both knobs strongly: at 0 only strong features plus
    coarse filler remain (a clean, simple, easy-to-run loop); at 1 nearly every
    outline point becomes an anchor (every jog traced). Anchor count is what the
    router actually follows, so this gives granularity real bite -- unlike tying
    it to snap spacing alone, which saturated once spacing dropped below a block.

    Returns the dense outline, the snapped node anchors, and each anchor's index
    within `dense` (so the router can slice the local arc between anchors).
    """
    g = cfg["granularity"]
    dense = resample(placed, step=grid.avg_edge * 0.4, closed=closed)
    importance = waypoint_importance(dense)
    # (On an open path the cyclic importance chord wraps end-to-start, which
    # marks both endpoints as important -- exactly right: they must be anchors.)

    max_gap = grid.avg_edge * (5.0 - 4.6 * g)              # g=0 -> very sparse; g=1 -> ~0.4 edge
    feat_thresh = 0.35 * (1.0 - g)                         # g=1 -> keep all; g=0 -> only features
    pts = np.asarray(dense)

    anchor_idx, acc = [0], 0.0
    for i in range(1, len(dense)):
        acc += float(np.hypot(*(pts[i] - pts[i - 1])))
        if importance[i] >= feat_thresh or acc >= max_gap:
            anchor_idx.append(i)
            acc = 0.0
    if anchor_idx[-1] != len(dense) - 1:
        anchor_idx.append(len(dense) - 1)

    _, idx = grid.tree.query(pts[anchor_idx])
    waypoints, wp_dense_idx = [], []
    last = -1
    for a, k in zip(anchor_idx, idx):
        if k != last:
            waypoints.append(grid.node_keys[int(k)])
            wp_dense_idx.append(a)
            last = int(k)
    return dense, waypoints, wp_dense_idx


def _polyline_dist_fn(seg):
    """Vectorized closure: min distance from a point to a polyline `seg`."""
    seg = np.asarray(seg, dtype=np.float64)
    if len(seg) < 2:
        anchor = seg[0] if len(seg) else None
        return lambda p: 0.0 if anchor is None else float(np.hypot(*(np.asarray(p) - anchor)))
    a, b = seg[:-1], seg[1:]
    d = b - a
    L = np.einsum("ij,ij->i", d, d)
    ok = L > 1e-18

    def dist(p):
        p = np.asarray(p, dtype=np.float64)
        t = np.zeros(len(L))
        t[ok] = np.clip(np.einsum("ij,ij->i", p - a, d)[ok] / L[ok], 0.0, 1.0)
        proj = a + t[:, None] * d
        return float(np.min(np.hypot(proj[:, 0] - p[0], proj[:, 1] - p[1])))
    return dist


def route_pair(graph, src, dst, seg, weight, best_effort=False):
    """Dijkstra from src to dst, penalizing distance from the local outline arc.

    Cost = edge_length + weight * distance_to_outline(neighbor). With weight >> 1
    and a short local `seg`, the cheapest path is the one that stays on the line.

    best_effort: when dst is unreachable (a disconnected pocket -- a park-mesh
    island, a clipped component), return the path to the reachable node nearest
    to dst (the closest approach) instead of []. The caller continues from that
    node, so the assembled route stays a connected walk on real edges rather
    than teleporting across the gap.
    """
    if src == dst:
        return [src]
    dist_to = _polyline_dist_fn(seg)
    dev = {}                 # node -> distance to outline, computed once per node
    heap = [(0.0, 0, src)]
    best = {src: 0.0}
    prev = {src: None}
    seen = set()
    c = 1
    while heap:
        cost, _, u = heapq.heappop(heap)
        if u in seen:
            continue
        seen.add(u)
        if u == dst:
            break
        for v, w in graph.get(u, ()):
            if v in seen:
                continue
            dv = dev.get(v)
            if dv is None:
                dv = dev[v] = dist_to(v)
            nc = cost + w + weight * dv
            if nc < best.get(v, np.inf):
                best[v] = nc
                prev[v] = u
                heapq.heappush(heap, (nc, c, v))
                c += 1
    if dst not in prev:
        if not best_effort or not seen:
            return []
        # dst is in another component: bridge to the closest approach instead.
        reached = list(seen)
        arr = np.asarray(reached, dtype=np.float64)
        dst = reached[int(np.argmin(np.hypot(arr[:, 0] - dst[0],
                                             arr[:, 1] - dst[1])))]
    path, u = [], dst
    while u is not None:
        path.append(u)
        u = prev[u]
    return path[::-1]


def route_contour(grid, dense, waypoints, wp_idx, weight):
    """Chain route_pair across all waypoints, sharing junction nodes.

    An unreachable waypoint (a disconnected pocket in the graph) doesn't break
    the chain: its leg bridges to the reachable node nearest the target and
    the next leg continues from wherever the route actually ended -- so the
    result is always a connected walk on real edges, never a straight-line
    teleport that would be counted as if it were run.
    """
    route, bridged = [], 0
    cur = waypoints[0]
    for i in range(1, len(waypoints)):
        seg = dense[wp_idx[i - 1]:wp_idx[i] + 1]
        if len(seg) < 2:
            seg = np.array([cur, waypoints[i]])
        part = route_pair(grid.graph, cur, waypoints[i], seg, weight,
                          best_effort=True)
        if not part:
            continue
        route.extend(part[1:] if route else part)
        cur = route[-1]
        if cur != waypoints[i]:
            bridged += 1
    if bridged:
        print(f"  bridged {bridged} unreachable waypoint(s) at closest approach")
    return route


# --------------------------------------------------------------------------- #
# Cleanup                                                                      #
# --------------------------------------------------------------------------- #
def _deviations(points, contour_tree):
    """Distance from each point to the target outline (empty -> empty array)."""
    if len(points) == 0:
        return np.zeros(0)
    d, _ = contour_tree.query(np.asarray(points, dtype=np.float64))
    return d


def remove_backtracks(route, contour_tree, tol):
    """Cancel immediate edge retraces (``A, B, A`` -> ``A``), protrusion-safe.

    A stack pass: arriving back at the node we just left undoes the detour, which
    dissolves hairpin spikes and the thick "doubled" street segments. But the
    removal only fires when the detour node *strays* from the target shape by
    more than ``tol``. An out-and-back that hugs the shape is kept -- that is how
    a real thin protrusion (beak, leg, tail) has to be run: up the street and
    back down. A genuine 3-cycle (``A, B, C, A``) is also kept.
    """
    out = []
    for n in route:
        if len(out) >= 2 and out[-2] == n:
            strays = _deviations([out[-1]], contour_tree)[0] > tol
            if strays:
                out.pop()               # artifact detour -> undo it
                continue
        if out and out[-1] == n:
            continue                    # consecutive duplicate
        out.append(n)
    return out


def collapse_loops(route, max_arc, contour_tree, tol):
    """Drop the inner arc of a small self-intersection (loop / nook).

    Any node visited twice marks a loop in what should be a simple outline. The
    inner arc is removed only when it is short (<= ``max_arc`` nodes) *and*
    strays from the shape by more than ``tol`` -- so artifacts vanish while a
    genuine protrusion that revisits a street is preserved.
    """
    route = list(route)
    while True:
        seen = {}
        for i, n in enumerate(route):
            if n in seen and i - seen[n] <= max_arc:
                a = seen[n]
                inner = route[a + 1:i + 1]
                if _deviations(inner, contour_tree).max(initial=0.0) > tol:
                    del route[a + 1:i + 1]      # keep one copy at index a
                    break
            seen[n] = i
        else:
            return route


def shortcut_nooks(grid, route, contour_tree, weight, window, slack):
    """Straighten staircase wiggles and little in-and-out nooks.

    Each short window is re-solved with the contour-biased router and the
    replacement is taken only if it is shorter and lands within ``slack`` of the
    shape. ``slack`` (set from granularity) is how much corner-cutting is allowed
    when smoothing: 0 preserves every step, larger values flatten the staircase.
    A protrusion can never be shortcut away -- cutting across it would push the
    path far past ``slack`` from the outline tip, so the swap is rejected.
    """
    out = list(route)
    i = 0
    while i < len(out) - 2:
        j = min(i + window, len(out) - 1)
        arc = out[i:j + 1]
        alt = route_pair(grid.graph, out[i], out[j], np.asarray(arc), weight)
        if (2 <= len(alt) < len(arc)
                and _deviations(alt, contour_tree).max(initial=0.0)
                <= _deviations(arc, contour_tree).max(initial=0.0) + slack):
            out = out[:i] + alt + out[j + 1:]
        else:
            i += 1
    return out


def cleanup(grid, route, dense, wp_idx, weight, cfg, close=True):
    """Close the loop, then strip artifacts while protecting real protrusions.

    The densified placed outline (``dense``) is the reference shape: cleanup only
    removes route features that disagree with it. close=False skips the loop
    closure (for open feature paths, which are run out-and-back instead).
    """
    contour_tree = cKDTree(np.asarray(dense, dtype=np.float64))
    tol = grid.avg_edge * cfg["protrusion_tolerance"]
    g = cfg["granularity"]
    window = int(round(4 + (1.0 - g) * 40))         # g=1 -> 4 (keep detail); g=0 -> 44 (smooth hard)
    slack = grid.avg_edge * (1.0 - g) ** 0.6 * 3.5  # corner-cut budget when smoothing

    route = remove_backtracks(route, contour_tree, tol)
    if close:
        route = close_loop(grid, route, dense, wp_idx, weight)
    if len(route) < 4:
        return route
    closed = route[0] == route[-1]
    body = route[:-1] if closed else route          # protect the closure seam
    max_arc = max(4, len(body) // 4)
    for _ in range(3):                              # passes reach a fixed point
        before = len(body)
        body = remove_backtracks(body, contour_tree, tol)
        body = collapse_loops(body, max_arc, contour_tree, tol)
        body = shortcut_nooks(grid, body, contour_tree, weight, window, slack)
        if len(body) == before:
            break
    return body + [body[0]] if closed else body


def close_loop(grid, route, dense, wp_idx, weight):
    """Route the wrap-around arc from the last waypoint back to the first."""
    if len(route) < 2 or route[-1] == route[0]:
        return route
    seg = np.vstack([dense[wp_idx[-1]:], dense[:wp_idx[0] + 1]])
    closing = route_pair(grid.graph, route[-1], route[0], seg, weight,
                         best_effort=True)
    return route + closing[1:] if closing else route


# --------------------------------------------------------------------------- #
# Distance + land checks                                                       #
# --------------------------------------------------------------------------- #
def route_length_m(route, grid):
    """Real-world length of a route, in metres.

    The route lives in [0, 1] coords that were obtained by dividing projected
    metres by `grid.span`, so multiplying the summed segment lengths back by
    `grid.span` recovers metres. (Returns the on-street distance actually run,
    not straight-line.)
    """
    if len(route) < 2:
        return 0.0
    seg = np.linalg.norm(np.diff(np.asarray(route, np.float64), axis=0), axis=1)
    return float(seg.sum() * grid.span)


def format_distance(metres):
    """'7.43 km / 4.62 mi'."""
    return f"{metres / 1000:.2f} km / {metres / 1609.344:.2f} mi"


def land_fraction(pts, grid, reach=2.5):
    """Fraction of points that sit on walkable land.

    A point is "on land" if a street node lies within `reach` average edges of
    it. Water (rivers, ocean) and any area outside the fetched network have no
    nearby nodes, so a shape spilling into them shows up as a low fraction. Parks
    count as land because the park mesh added routable nodes there.
    """
    d, _ = grid.tree.query(np.asarray(pts, np.float64))
    return float(np.mean(d < grid.avg_edge * reach))


def on_land(placed, grid, cfg):
    """True if the placed outline stays on walkable land (the no-ocean guard)."""
    return land_fraction(placed, grid, cfg["land_reach"]) >= cfg["min_land_fraction"]


# --------------------------------------------------------------------------- #
# Fidelity metrics                                                            #
# --------------------------------------------------------------------------- #
def hausdorff(route, contour):
    """Symmetric Hausdorff distance between two polylines (lower is better)."""
    r, c = np.asarray(route), np.asarray(contour)
    dr, _ = cKDTree(c).query(r)
    dc, _ = cKDTree(r).query(c)
    return float(max(dr.max(), dc.max()))


def frechet(route, contour, samples=140):
    """Discrete Frechet distance between two ordered curves (lower is better).

    Unlike Hausdorff/IoU, Frechet respects *order*: imagine walking a dog, you on
    the route and it on the target, neither backtracking -- the score is the
    shortest leash that lets you both finish. A route that nails every point but
    in big out-of-order detours (the rectangular excursions in the bad cases)
    keeps Hausdorff low yet blows Frechet up, which is exactly why it's the right
    judge of "does the path actually trace the shape".
    """
    P = resample(route, n=samples)
    Q = resample(contour, n=samples)
    D = np.linalg.norm(P[:, None, :] - Q[None, :, :], axis=2)
    n, m = D.shape
    ca = np.empty((n, m))
    ca[0, 0] = D[0, 0]
    for i in range(1, n):
        ca[i, 0] = max(ca[i - 1, 0], D[i, 0])
    for j in range(1, m):
        ca[0, j] = max(ca[0, j - 1], D[0, j])
    for i in range(1, n):
        prev, cur = ca[i - 1], ca[i]
        for j in range(1, m):
            cur[j] = max(min(prev[j], prev[j - 1], cur[j - 1]), D[i, j])
    return float(ca[-1, -1])


def _dtw_pair(P, Q):
    """Summed-distance DTW between two equal-length point arrays (lower better)."""
    D = np.linalg.norm(P[:, None, :] - Q[None, :, :], axis=2)
    n, m = D.shape
    acc = np.empty((n, m))
    acc[0, 0] = D[0, 0]
    for i in range(1, n):
        acc[i, 0] = acc[i - 1, 0] + D[i, 0]
    for j in range(1, m):
        acc[0, j] = acc[0, j - 1] + D[0, j]
    for i in range(1, n):
        prev, cur = acc[i - 1], acc[i]
        for j in range(1, m):
            cur[j] = D[i, j] + min(prev[j], prev[j - 1], cur[j - 1])
    return float(acc[-1, -1]) / (n + m)        # normalize by alignment-path scale


def dtw(route, contour, samples=120, offsets=12):
    """Cyclic Dynamic Time Warping between two closed curves (lower is better).

    DTW finds the same kind of monotonic alignment as Frechet, but *sums* the
    leash over the alignment instead of taking its max. Frechet is hostage to the
    single hardest point (one star tip), so it can't separate a route that hugs
    the whole outline from one that only nails the worst spot; DTW's running
    total rewards tracking the shape everywhere, breaking those ties toward
    tighter fits. Because these are *closed* loops with no canonical start, we
    try a handful of cyclic offsets (and both winding directions) of the target
    and keep the best -- otherwise a correct route that merely begins at a
    different point on the loop would score as a mismatch.
    """
    P = resample(route, n=samples)[:-1]        # drop the duplicated closing point
    Q = resample(contour, n=samples)[:-1]
    n = len(Q)
    step = max(1, n // offsets)
    best = np.inf
    for Qd in (Q, Q[::-1]):                     # both winding directions
        for k in range(0, n, step):
            best = min(best, _dtw_pair(P, np.roll(Qd, k, axis=0)))
    return best


def turning_distance(route, contour, samples=180):
    """Turning-function (tangent-angle) distance between two closed curves.

    Each curve is re-expressed as cumulative turning angle vs normalized arc
    length -- a translation/scale/rotation-invariant signature of its *form*:
    where the corners and protrusions are, not where the curve sits. Comparing
    these signatures (min over cyclic start + winding, with a constant rotation
    offset removed) answers "does the route bend like the shape". It is sharp
    about real features (a beak, a leg) yet largely ignores staircase jitter,
    which the position-based metrics (Frechet/Hausdorff) cannot separate out.
    """
    def signature(pts):
        p = resample(pts, n=samples)[:-1]
        d = np.diff(np.vstack([p, p[:1]]), axis=0)
        return np.unwrap(np.arctan2(d[:, 1], d[:, 0]))

    a = signature(route)
    b0 = signature(contour)
    n = len(a)
    idx = (np.arange(n)[None, :] - np.arange(n)[:, None]) % n   # row k == roll(b, k)
    best = np.inf
    for bd in (b0, b0[::-1]):                    # both winding directions
        diff = a[None, :] - bd[idx]              # all cyclic shifts at once
        diff -= diff.mean(axis=1, keepdims=True)  # remove constant rotation offset
        best = min(best, float(np.sqrt((diff ** 2).mean(axis=1).min())))
    return best


def perceptual_cost(route, placed, res=128, blur=4.0):
    """Holistic, blur-tolerant shape distance via low-res render-and-compare.

    Pointwise metrics (Frechet, mean deviation) are fooled by staircase noise --
    a jagged edge is still "close on average" -- so a blocky blob can score well
    yet not read as the shape. This rasterizes both closed outlines to small
    filled masks, blurs them, and returns 1 - soft-IoU. Blur discards the
    high-frequency staircase while preserving the gestalt, which is how the eye
    judges it: it rewards capturing the overall form, not hugging every point.
    """
    def mask(poly):
        m = np.zeros((res, res), np.float32)
        p = np.clip(np.asarray(poly) * (res - 1), 0, res - 1).astype(np.int32)
        cv2.fillPoly(m, [p], 1.0)
        cv2.polylines(m, [p], True, 1.0, 1)        # so thin shapes still register
        return cv2.GaussianBlur(m, (0, 0), blur)
    a, b = mask(route), mask(placed)
    union = float(np.sum(np.maximum(a, b)))
    inter = float(np.sum(np.minimum(a, b)))
    return 1.0 - (inter / union if union > 1e-9 else 0.0)


def placement_cost(route, placed):
    """Selection cost for the search: perceptual distance (primary) + a slice of
    the geometric composite (keeps thin protrusions honest, since area-overlap
    under-weights a skinny tail or beak)."""
    return perceptual_cost(route, placed) + 0.3 * route_match_cost(route, placed)


def route_match_cost(route, placed):
    """Composite cost for *choosing* a placement (lower = better resemblance).

    Frechet alone is worst-case: dominated by the single hardest point, blind to
    the rest. So it will trade away a protrusion's fidelity or a tight overall hug
    just to shave the worst leash -- which is what degraded the thumbs-up (compact
    body + one thumb) even as it helped the elongated shark. We blend three views:

      * Frechet           -- no big out-of-order excursions (the original guard),
      * mean two-way dev  -- the route hugs the shape *on average*, not just at its
                             worst point (breaks Frechet's ties toward tighter fits),
      * feature deviation -- the defining points (a thumb tip) are actually visited,
                             so protrusions are credited rather than sacrificed.
    """
    P, C = np.asarray(route, np.float64), np.asarray(placed, np.float64)
    d_c2r, _ = cKDTree(P).query(C)        # each shape point -> nearest route point
    d_r2c, _ = cKDTree(C).query(P)        # each route point -> nearest shape point
    mean_dev = 0.5 * (float(d_c2r.mean()) + float(d_r2c.mean()))
    feat = waypoint_importance(C) > 0.6
    feature_dev = float(d_c2r[feat].mean()) if feat.any() else float(d_c2r.mean())
    return frechet(route, placed) + 2.0 * mean_dev + 3.0 * feature_dev


def iou(route, contour, buffer):
    """IoU of the two outlines, each thickened by `buffer` (higher is better)."""
    try:
        a = LineString(route).buffer(buffer)
        b = LineString(contour).buffer(buffer)
        u = a.union(b).area
        return a.intersection(b).area / u if u else 0.0
    except Exception:
        return 0.0


# --------------------------------------------------------------------------- #
# Plot                                                                         #
# --------------------------------------------------------------------------- #
def _matplotlib(show):
    """Import matplotlib, falling back to the headless Agg backend when the
    figure is only being saved (no display needed / available)."""
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def _finish(plt, fig, save, show):
    plt.tight_layout()
    if save:
        fig.savefig(save, dpi=150, bbox_inches="tight")
        print(f"saved plot -> {save}")
    if show:
        plt.show()
    else:
        plt.close(fig)


def _draw_features(ax, feats):
    """Overlay placed inner-feature targets (dashed) and their routes."""
    for i, (f, fp, fr) in enumerate(feats):
        ax.plot(fp[:, 0], fp[:, 1], "--", color="purple", lw=1.2, alpha=0.6)
        if len(fr) >= 2:
            fx, fy = zip(*fr)
            ax.plot(fx, fy, color="darkorange", lw=2.5, zorder=5,
                    label="inner features" if i == 0 else None)


def plot(grid, contour, route, feats=(), save=None, show=True):
    plt = _matplotlib(show)
    from matplotlib.collections import LineCollection
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.add_collection(LineCollection(
        [[p1, p2] for p1, p2 in grid.edge_list],
        colors="steelblue", linewidths=0.5, alpha=0.3))
    ax.plot(contour[:, 0], contour[:, 1], "--", color="purple", lw=2,
            label="target shape")
    if route:
        rx, ry = zip(*route)
        ax.plot(rx, ry, color="coral", lw=3, zorder=5, label="route")
        ax.scatter(rx[0], ry[0], color="green", s=100, zorder=6)
        ax.scatter(rx[-1], ry[-1], color="red", s=100, zorder=6)
    _draw_features(ax, feats)
    ax.set(xlim=(0, 1), ylim=(0, 1), aspect="equal")
    ax.legend()
    _finish(plt, fig, save, show)


def plot_options(grid, panels, save=None, show=True):
    """Plot panels of (label, Candidate) side by side.

    Each panel carries its own placed contour, so this works for both placement
    diversity (different placements per panel) and detail variants (same shape).
    """
    plt = _matplotlib(show)
    from matplotlib.collections import LineCollection
    segs = [[p1, p2] for p1, p2 in grid.edge_list]
    fig, axes = plt.subplots(1, len(panels), figsize=(10 * len(panels), 10))
    for ax, (label, cand) in zip(np.atleast_1d(axes), panels):
        ax.add_collection(LineCollection(segs, colors="steelblue",
                                         linewidths=0.5, alpha=0.3))
        ax.plot(cand.placed[:, 0], cand.placed[:, 1], "--", color="purple", lw=2)
        if cand.route:
            rx, ry = zip(*cand.route)
            ax.plot(rx, ry, color="coral", lw=3, zorder=5)
            ax.scatter(rx[0], ry[0], color="red", s=100, zorder=6)
        _draw_features(ax, cand.feats)
        ax.set(xlim=(0, 1), ylim=(0, 1), aspect="equal")
        ax.set_title(label)
    _finish(plt, fig, save, show)


# --------------------------------------------------------------------------- #
# Main                                                                         #
# --------------------------------------------------------------------------- #
def feature_run_length_m(feats, grid):
    """Total running distance of the routed inner features, in metres.

    A closed feature is run once around; an open one is run out-and-back, so
    its single-pass length counts twice.
    """
    return sum(route_length_m(fr, grid) * (1.0 if f.closed else 2.0)
               for f, _, fr in feats if len(fr) >= 2)


def _report(label, grid, cand):
    placed, route = cand.placed, cand.route
    if not route:
        print(f"{label}: no route"); return
    msg = (f"{label}: {len(route)} nodes  IoU={iou(route, placed, 0.01):.3f}  "
           f"Frechet={frechet(route, placed):.4f}  "
           f"on-land={land_fraction(placed, grid) * 100:.0f}%  "
           f"distance={format_distance(route_length_m(route, grid))}")
    if cand.feats:
        n = sum(1 for _, _, fr in cand.feats if len(fr) >= 2)
        msg += (f"  inner-features={n}/{len(cand.feats)} routed "
                f"(+{format_distance(feature_run_length_m(cand.feats, grid))})")
    print(msg)
# --------------------------------------------------------------------------- #
# CONFIG                                                                       #
# --------------------------------------------------------------------------- #
CONFIG = dict(
    svg_path="shapes/star.svg",   # any of the bundled shapes/*.svg, or your own
    # Street network location (default: midtown Manhattan).
    lat=40.7527,
    lng=-73.9943,
    radius_m=1600,
    # Optional: path to a saved ox.save_graphml network to load instead of
    # fetching from the Overpass API (offline / reproducible real-map runs).
    # The graph is cropped to the lat/lng + radius_m window above.
    graphml_path=None,
    include_parks=True,
    park_spacing=0.5,         # park lattice pitch as a multiple of avg edge length

    # Street resolution. OSMnx returns intersection-to-intersection edges, so the
    # router can't make a move smaller than a block -- sub-block features (a thin
    # mouth, a notch) are simply unrepresentable. Densifying inserts nodes every
    # `node_spacing` metres along each street (following its real curve), giving
    # the router fine anchors to trace features and curves. Smaller = more detail,
    # more nodes, slower. This is the lever for feature fidelity, NOT deviation_weight.
    densify_streets=True,
    node_spacing=30.0,        # metres between inserted street nodes

    # SVG raster size for contour tracing.
    img_size=2048,

    # Inner features (holes, separate elements, interior detail strokes) are
    # extracted alongside the outline by extract_shape; features whose length
    # is below this fraction of the [0, 1] frame are treated as raster noise.
    inner_min_perimeter=0.05,
    # Place + route the inner features with the chosen outer transform, and let
    # their routed fidelity join the placement cost of the top n_route_eval
    # candidates: a placement whose contour seats nicely but strands the eye
    # ranks below one that draws both. Weight scales the feature term (which is
    # itself bounded by the features' share of total drawn length).
    inner_features=True,
    inner_cost_weight=0.6,
    # Stage-1 proxy bonus for placements that seat the inner features well
    # (snap closeness + node resolvability of a few sample points per feature,
    # scaled by the features' share of drawn length). Keeps feature-friendly
    # placements alive into stage 2, where the full routed feature cost rules.
    inner_proxy_weight=0.15,
    # Per-feature tailoring (refine_feature). Each placed feature is re-seated
    # on the local street fabric: candidate positions are the drawn spot plus
    # street nodes within inner_search_radius x the feature's span, tried at
    # small rotations (+/- inner_rot_deg) and a ladder of scales, including a
    # rescue upscale for features narrower than inner_min_span_blocks street
    # edges (capped at inner_max_inflate x). A cheap street-fit proxy ranks
    # the variants, the best inner_route_eval are routed, and a drift penalty
    # anchors features to where the drawing put them. inner_refine=False
    # disables all of it.
    inner_refine=True,
    inner_search_radius=1.0,
    inner_rot_deg=12.0,
    inner_route_eval=6,
    inner_min_span_blocks=6.0,
    inner_max_inflate=3.0,

    # Placement search.
    scale_range=(0.8, 1.8),   # shape extent as a fraction of the grid span
    aspect_max=1.5,           # max area-preserving stretch (1.0 = uniform scale)
    shear_max=0.20,           # max skew, for non-orthogonal grids
    n_random=2000,            # coarse random placements
    n_refine=600,             # refinements jittered around the best few
    n_route_eval=6,           # top candidates actually routed + judged by Frechet
    margin=0.02,              # keep the shape this far inside the [0, 1] box

    # Keep the shape on walkable land. The route is always on streets, but the
    # target placement could spill over a river/ocean (no nodes there) and distort
    # the result. Reject any placement where less than `min_land_fraction` of the
    # outline is within `land_reach` average edges of a street node.
    min_land_fraction=0.85,
    land_reach=2.5,

    # Routing / fidelity.
    deviation_weight=60.0,    # >> 1 makes "stay on the outline" dominate cost

    # Granularity: 0.0 = smooth, fewer steps, shorter and easier to run;
    #              1.0 = trace every jog of the shape, longer and more expressive.
    # Drives waypoint spacing and how hard the staircase steps get smoothed away.
    granularity=0.6,

    # Present several renderings of the chosen placement side by side. The
    # expensive placement search runs once; each option is a cheap re-render that
    # overrides a couple of knobs. Set False for a single plot at `granularity`.
    #   efficient -- deviation_weight 0 makes routing plain shortest-path Dijkstra
    #                between anchors (no contour hugging). Needs DENSER anchors so
    #                they land inside concavities; otherwise shortest-path slices
    #                straight across them and the shape blobs out. Clean + runnable.
    #   simple    -- low granularity + normal weight: a tidy, smoothed hug.
    #   faithful  -- high granularity + normal weight: traces every jog.
    present_options=True,
    # "placements": show the top n_options DISTINCT placements (different rotations
    #               / scales) so you can pick the one whose street angles read best
    #               -- the fix for "every option has the same shallow-angle steps".
    # "detail":     fix the single best placement, vary detail via option_presets.
    option_mode="placements",
    n_options=3,
    option_presets={
        "efficient": dict(granularity=0.65, deviation_weight=1.0),
        "simple":    dict(granularity=0.35),
        "faithful":  dict(granularity=0.90),
    },

    # How far (in avg-edge lengths) the route may stray from the shape before an
    # out-and-back / loop is treated as an artifact instead of a real protrusion
    # (beak, leg, tail). Higher = more forgiving of genuine thin features.
    protrusion_tolerance=2.5,

    seed=42,
)


def main(cfg=CONFIG):
    grid = build_grid(cfg)
    spec = extract_shape(cfg["svg_path"], cfg["img_size"],
                         min_perimeter=cfg["inner_min_perimeter"])
    contour = spec.outer
    if spec.inners:
        n_open = sum(1 for f in spec.inners if not f.closed)
        print(f"inner features: {len(spec.inners)} "
              f"({len(spec.inners) - n_open} closed loop(s), {n_open} open path(s))")
    inners = spec.inners if cfg.get("inner_features", True) else []
    ranked = search_placement(contour, grid, cfg, inners=inners)   # [Candidate, ...]
    best = ranked[0]
    save, show = cfg.get("save_plot"), cfg.get("show_plot", True)

    if not cfg.get("present_options"):
        _report("route", grid, best)
        plot(grid, best.placed, best.route, feats=best.feats, save=save, show=show)
        return ranked

    if cfg["option_mode"] == "placements":
        # Different placements per panel -- pick the one whose street angles read best.
        panels = [(f"placement {i + 1}  cost={c.cost:.3f}", c)
                  for i, c in enumerate(ranked[:cfg["n_options"]])]
    else:  # "detail": fix the best placement, vary detail
        panels = []
        for name, ov in cfg["option_presets"].items():
            c2 = {**cfg, **ov}
            feats = [(f, fp, build_feature_route(grid, fp, f.closed, c2))
                     for f, fp, _ in best.feats]
            panels.append((name, Candidate(best.cost, best.placed,
                                           build_route(grid, best.placed, c2)[0],
                                           feats)))

    for label, cand in panels:
        _report(label, grid, cand)
    plot_options(grid, panels, save=save, show=show)
    return ranked


def _cli(argv=None):
    """Command-line overrides for CONFIG, so a run like 'trace the star over
    Chicago and save the plot' needs no source edits."""
    import argparse
    ap = argparse.ArgumentParser(description="SVG -> running route on real streets.")
    ap.add_argument("--svg", help="path to the shape SVG")
    ap.add_argument("--lat", type=float, help="network center latitude")
    ap.add_argument("--lng", type=float, help="network center longitude")
    ap.add_argument("--radius", type=float, dest="radius_m",
                    help="network radius around the center, metres")
    ap.add_argument("--graphml", help="load this saved GraphML instead of "
                                      "fetching from the Overpass API")
    ap.add_argument("--granularity", type=float,
                    help="0=smooth/simple .. 1=trace every jog")
    ap.add_argument("--seed", type=int, help="placement-search RNG seed")
    ap.add_argument("--save", help="write the plot to this image path")
    ap.add_argument("--no-show", action="store_true",
                    help="don't open a window (headless; use with --save)")
    ap.add_argument("--no-inner-features", action="store_true",
                    help="outline only: skip placing/routing/scoring the "
                         "shape's inner features")
    args = ap.parse_args(argv)

    cfg = dict(CONFIG)
    for key, val in [("svg_path", args.svg), ("lat", args.lat), ("lng", args.lng),
                     ("radius_m", args.radius_m), ("graphml_path", args.graphml),
                     ("granularity", args.granularity), ("seed", args.seed),
                     ("save_plot", args.save)]:
        if val is not None:
            cfg[key] = val
    if args.no_show:
        cfg["show_plot"] = False
    if args.no_inner_features:
        cfg["inner_features"] = False
    return cfg


if __name__ == "__main__":
    main(_cli())
