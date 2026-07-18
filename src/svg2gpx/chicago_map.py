"""
Run the SVG -> route pipeline over the bundled shapes on the REAL Chicago
street network and render every result on the actual OSMnx map.

Street data
-----------
By default this uses a citywide OpenStreetMap snapshot of Chicago saved with
``ox.save_graphml`` (published by the UIUC CyberGIS group for the Kang et al.
COVID-19 accessibility study). The file is downloaded once into ``data/`` and
then loaded offline, which keeps runs fast and reproducible and works where
the Overpass API is unreachable. Pass ``--live`` to fetch fresh data from
OpenStreetMap instead (requires network access to the Overpass API).

The snapshot is cropped to ``--radius`` metres around ``--lat/--lng`` (default:
Logan Square / Avondale, a dense orthogonal street grid), expressways are
dropped (you can't run on the Kennedy), and the largest connected component is
fed to the router via gen.grid_from_graph.

Outputs, per shape, into ``--outdir`` (default chicago_maps/):
  * <shape>_chicago.png      the route drawn on the real street map
  * <shape>_route.geojson    route + target outline in WGS84 lon/lat
plus all_shapes_chicago.png (gallery) and chicago_results.csv (metrics).

Usage:
    python chicago_map.py                    # all shapes
    python chicago_map.py --shape star       # one shape
    python chicago_map.py --lat 41.95 --lng -87.68 --radius 2000
"""

import argparse
import csv
import glob
import os
import urllib.request

import networkx as nx
import numpy as np

from . import gen
from .gen import (
    dtw,
    extract_shape,
    feature_run_length_m,
    format_distance,
    frechet,
    grid_from_graph,
    hausdorff,
    iou,
    perceptual_cost,
    route_length_m,
    search_placement,
    turning_distance,
)

HERE = os.path.dirname(os.path.abspath(__file__))
SHAPES_DIR = os.path.join(HERE, "shapes")   # bundled read-only package data
# Downloaded/cached data is a run artifact, not package data -- keep it next to
# where the tool runs (cwd), not inside the installed package tree.
DATA_DIR = "data"
GRAPHML_PATH = os.path.join(DATA_DIR, "Chicago_Network.graphml")
GRAPHML_URL = ("https://raw.githubusercontent.com/cybergis/"
               "COVID-19AccessibilityNotebook/main/data/Chicago_Network.graphml")

# Default map window: Logan Square / Avondale. A dense, regular street grid
# (plus Milwaukee Ave's diagonal), well inside the city -- no lake, no Loop
# river tangle -- so shapes have room to seat cleanly.
CENTER_LAT, CENTER_LNG = 41.9285, -87.7075
RADIUS_M = 1600

# Roads a runner cannot use; dropped before routing.
NON_RUNNABLE = {"motorway", "motorway_link", "trunk", "trunk_link"}

FIELDS = ["shape", "cost", "frechet", "hausdorff", "iou", "perceptual",
          "dtw", "turning", "length_m", "nodes",
          "feat_routed", "feat_total", "feat_length_m"]


def ensure_graphml(path=GRAPHML_PATH, url=GRAPHML_URL):
    if os.path.exists(path):
        return path
    os.makedirs(os.path.dirname(path), exist_ok=True)
    print(f"downloading Chicago OSM network -> {path}")
    urllib.request.urlretrieve(url, path)
    print(f"  {os.path.getsize(path) / 1e6:.1f} MB")
    return path


def _runnable(G):
    """Drop expressway edges, then keep the largest connected component."""
    bad = []
    for u, v, k, data in G.edges(keys=True, data=True):
        hw = data.get("highway")
        hws = set(hw) if isinstance(hw, list) else {hw}
        if hws & NON_RUNNABLE:
            bad.append((u, v, k))
    G.remove_edges_from(bad)
    G.remove_nodes_from(list(nx.isolates(G)))
    wcc = max(nx.weakly_connected_components(G), key=len)
    return G.subgraph(wcc).copy()


def build_chicago_grid(cfg, live=False):
    """Real Chicago street Grid + the projected graph it came from (for maps)."""
    import osmnx as ox
    if live:
        G = ox.graph_from_point((cfg["lat"], cfg["lng"]), dist=cfg["radius_m"],
                                network_type="walk", simplify=True)
    else:
        G = ox.load_graphml(ensure_graphml())
        bbox = ox.utils_geo.bbox_from_point((cfg["lat"], cfg["lng"]),
                                            dist=cfg["radius_m"])
        G = ox.truncate.truncate_graph_bbox(G, bbox)
    G = _runnable(G)
    G_proj = ox.project_graph(G)
    grid = grid_from_graph(G_proj, cfg)
    return grid, G_proj


def to_projected(pts, grid):
    """[0, 1] pipeline coords -> projected metres (the map's CRS)."""
    pts = np.asarray(pts, dtype=np.float64)
    return pts * grid.span + np.array([grid.x_min, grid.y_min])


def to_lonlat(pts_proj, G_proj):
    """Projected metres -> WGS84 (lon, lat) pairs."""
    from pyproj import Transformer
    tf = Transformer.from_crs(G_proj.graph["crs"], "EPSG:4326", always_xy=True)
    lon, lat = tf.transform(pts_proj[:, 0], pts_proj[:, 1])
    return np.column_stack([lon, lat])


def draw_streets(ax, edges_gdf):
    edges_gdf.plot(ax=ax, color="#c9c9c9", linewidth=0.7, zorder=1)
    ax.set_facecolor("white")
    ax.set_aspect("equal")
    ax.set_axis_off()


def draw_result(ax, grid, placed, route, feats=()):
    tgt = to_projected(placed, grid)
    ax.plot(tgt[:, 0], tgt[:, 1], "--", color="#5b6abf", lw=1.6,
            alpha=0.9, zorder=3, label="target shape")
    if route:
        r = to_projected(np.asarray(route), grid)
        ax.plot(r[:, 0], r[:, 1], color="#e4572e", lw=2.8, zorder=4,
                solid_capstyle="round", label="running route")
        ax.scatter(*r[0], color="#2e933c", s=60, zorder=5, label="start")
    drew_label = False
    for f, fp, fr in feats:
        ft = to_projected(fp, grid)
        ax.plot(ft[:, 0], ft[:, 1], "--", color="#5b6abf", lw=1.1,
                alpha=0.7, zorder=3)
        if len(fr) >= 2:
            r = to_projected(np.asarray(fr), grid)
            ax.plot(r[:, 0], r[:, 1], color="#e88b2e", lw=2.2, zorder=4,
                    solid_capstyle="round",
                    label=None if drew_label else "inner features")
            drew_label = True


def save_geojson(path, route_ll, target_ll, props, feats_ll=()):
    import json

    def line(coords, role, extra=None):
        return {"type": "Feature",
                "properties": {**props, "role": role, **(extra or {})},
                "geometry": {"type": "LineString",
                             "coordinates": [[round(x, 6), round(y, 6)]
                                             for x, y in coords]}}

    feats = [line(route_ll, "route"), line(target_ll, "target_outline")]
    for i, (f, coords) in enumerate(feats_ll):
        feats.append(line(coords, "inner_feature_route",
                          {"feature_index": i, "closed": bool(f.closed),
                           "run_style": "loop" if f.closed else "out_and_back"}))
    with open(path, "w") as f:
        json.dump({"type": "FeatureCollection", "features": feats}, f)


def run_shape(grid, G_proj, edges_gdf, svg, cfg, outdir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    name = os.path.splitext(os.path.basename(svg))[0]
    print(f"\n=== {name} ===")
    spec = extract_shape(svg, cfg["img_size"],
                         min_perimeter=cfg["inner_min_perimeter"])
    inners = spec.inners if cfg.get("inner_features", True) else []
    ranked = search_placement(spec.outer, grid, {**cfg, "svg_path": svg},
                              inners=inners)
    cand = ranked[0]
    placed, route, feats = cand.placed, cand.route, cand.feats
    if len(route) < 2:
        print(f"  {name}: no route found")
        return None, (name, cand)

    dist_m = route_length_m(route, grid)
    feat_m = feature_run_length_m(feats, grid)
    feat_routed = sum(1 for _, _, fr in feats if len(fr) >= 2)
    row = {
        "shape": name,
        "cost": round(float(cand.cost), 4),
        "frechet": round(frechet(route, placed), 4),
        "hausdorff": round(hausdorff(route, placed), 4),
        "iou": round(iou(route, placed, 0.01), 3),
        "perceptual": round(perceptual_cost(route, placed), 4),
        "dtw": round(dtw(route, placed), 4),
        "turning": round(turning_distance(route, placed), 4),
        "length_m": round(dist_m, 1),
        "nodes": len(route),
        "feat_routed": feat_routed,
        "feat_total": len(feats),
        "feat_length_m": round(feat_m, 1),
    }
    print(f"  IoU={row['iou']}  Frechet={row['frechet']}  "
          f"distance={format_distance(dist_m)}  "
          f"inner-features={feat_routed}/{len(feats)}")

    fig, ax = plt.subplots(figsize=(11, 11))
    draw_streets(ax, edges_gdf)
    draw_result(ax, grid, placed, route, feats)
    ax.legend(loc="lower right", frameon=True)
    title = (f"'{name}' running route on real Chicago streets (OSM)\n"
             f"IoU {row['iou']}  |  {format_distance(dist_m)}")
    if feats:
        title += (f"  |  {feat_routed}/{len(feats)} inner features "
                  f"(+{format_distance(feat_m)})")
    ax.set_title(title)
    png = os.path.join(outdir, f"{name}_chicago.png")
    fig.savefig(png, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  saved {png}")

    route_ll = to_lonlat(to_projected(np.asarray(route), grid), G_proj)
    target_ll = to_lonlat(to_projected(placed, grid), G_proj)
    feats_ll = [(f, to_lonlat(to_projected(np.asarray(fr), grid), G_proj))
                for f, _, fr in feats if len(fr) >= 2]
    save_geojson(os.path.join(outdir, f"{name}_route.geojson"),
                 route_ll, target_ll,
                 {"shape": name, "length_m": row["length_m"], "city": "Chicago"},
                 feats_ll)
    return row, (name, cand)


def gallery(panels, grid, edges_gdf, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n = len(panels)
    cols = min(4, n)
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(6 * cols, 6 * rows))
    axes = np.atleast_1d(axes).ravel()
    for ax in axes[n:]:
        ax.set_axis_off()
    for ax, (name, cand) in zip(axes, panels):
        draw_streets(ax, edges_gdf)
        draw_result(ax, grid, cand.placed, cand.route, cand.feats)
        ax.set_title(name)
    fig.suptitle("Shape routes on the real Chicago street network (OpenStreetMap)",
                 fontsize=16)
    fig.tight_layout()
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"\nsaved gallery -> {path}")


def main():
    import osmnx as ox
    ap = argparse.ArgumentParser(
        description="Trace the bundled shapes on the real Chicago street network.")
    ap.add_argument("--shape", default="all",
                    help="shape file stem in shapes/ (e.g. 'star'), or 'all'")
    ap.add_argument("--lat", type=float, default=CENTER_LAT)
    ap.add_argument("--lng", type=float, default=CENTER_LNG)
    ap.add_argument("--radius", type=float, default=RADIUS_M)
    ap.add_argument("--live", action="store_true",
                    help="fetch fresh OSM data instead of the bundled snapshot")
    ap.add_argument("--no-inner-features", action="store_true",
                    help="outline only: skip placing/routing/scoring the "
                         "shapes' inner features")
    ap.add_argument("--outdir", default="chicago_maps")
    args = ap.parse_args()

    if args.shape == "all":
        svgs = sorted(glob.glob(os.path.join(SHAPES_DIR, "*.svg")))
    else:
        svgs = [os.path.join(SHAPES_DIR, f"{args.shape}.svg")]
    svgs = [s for s in svgs if os.path.exists(s)]
    if not svgs:
        raise SystemExit(f"no matching SVG shapes in {SHAPES_DIR}")
    os.makedirs(args.outdir, exist_ok=True)

    cfg = dict(gen.CONFIG)
    cfg.update(lat=args.lat, lng=args.lng, radius_m=args.radius,
               include_parks=False)   # snapshot has no park polygons
    if args.no_inner_features:
        cfg["inner_features"] = False

    grid, G_proj = build_chicago_grid(cfg, live=args.live)
    edges_gdf = ox.graph_to_gdfs(G_proj, nodes=False)

    rows, panels = [], []
    for svg in svgs:
        row, panel = run_shape(grid, G_proj, edges_gdf, svg, cfg, args.outdir)
        if row:
            rows.append(row)
        panels.append(panel)

    if rows:
        csv_path = os.path.join(args.outdir, "chicago_results.csv")
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=FIELDS)
            w.writeheader()
            w.writerows(rows)
        print(f"wrote {csv_path}")
    if len(panels) > 1:
        gallery(panels, grid, edges_gdf,
                os.path.join(args.outdir, "all_shapes_chicago.png"))


if __name__ == "__main__":
    main()
