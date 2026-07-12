# Packaging svg2gpx for PyPI

A concrete, reviewable plan to turn the current flat-module repo into an
installable package (`pip install svg2gpx`) with a `svg2gpx` command, without
losing the working pipeline. **No code is moved yet** — this is the blueprint.

The PyPI name `svg2gpx` is **available** (checked: `pypi.org/pypi/svg2gpx/json`
→ 404).

---

## 1. End state

```bash
pip install svg2gpx           # core: synthetic-grid runs, offline
pip install "svg2gpx[osm]"    # + OpenStreetMap fetch & matplotlib plotting

svg2gpx --svg star --lat 41.9285 --lng -87.7075 --gpx star.gpx   # CLI
python -m svg2gpx --svg shapes/mine.svg                          # module form
```
```python
import svg2gpx                                   # library
route = svg2gpx.generate("star", lat=41.9285, lng=-87.7075)
svg2gpx.to_gpx(route, "star.gpx")
```

The **one UX change**: `python gen.py …` becomes `svg2gpx …` / `python -m
svg2gpx …`. Every doc, the CI workflow, and the tests get updated to match in
the same change.

## 2. Target layout (src-layout, the packaging best practice)

```
svg2gpx/                       # repo root (rename happens on GitHub)
├── pyproject.toml             # NEW — the single source of build metadata
├── LICENSE                    # done (MIT)
├── README.md                  # done — update the usage commands
├── PACKAGING.md               # this file
├── src/
│   └── svg2gpx/
│       ├── __init__.py        # NEW — public API re-exports + version
│       ├── __main__.py        # NEW — `python -m svg2gpx` → cli.main()
│       ├── cli.py             # NEW — argparse (moved out of gen.py) + --gpx
│       ├── gpx.py             # NEW — geographic route → GPX 1.1 XML
│       ├── gen.py             # moved from ./gen.py  (pipeline + metrics)
│       ├── benchmark.py       # moved from ./benchmark.py
│       ├── chicago_map.py     # moved from ./chicago_map.py
│       ├── best_route.py      # moved from ./best_route.py
│       ├── preview_features.py# moved from ./preview_features.py
│       └── shapes/            # moved from ./shapes/  (bundled SVG package data)
├── tests/
│   ├── test_routing.py        # moved from ./test_routing.py
│   └── test_inner_features.py # moved from ./test_inner_features.py
└── docs/ ...                  # unchanged
```

Module filenames are kept as-is to minimize edits; the odd internal name
`svg2gpx.gen` never surfaces to users because `__init__.py` re-exports the
public API (§4).

## 3. Import edits (exhaustive)

Inside the package, switch the bare cross-imports to **explicit relative**
imports. Every site (from the current tree):

| File | Now | After |
| --- | --- | --- |
| `benchmark.py:38` | `import gen` | `from . import gen` |
| `benchmark.py:39` | `from gen import (…)` | `from .gen import (…)` |
| `best_route.py:25` | `import gen` | `from . import gen` |
| `best_route.py:26` | `from gen import (…)` | `from .gen import (…)` |
| `best_route.py:39` | `from benchmark import …` | `from .benchmark import …` |
| `chicago_map.py:39` | `import gen` | `from . import gen` |
| `chicago_map.py:40` | `from gen import (…)` | `from .gen import (…)` |
| `preview_features.py:22` | `from gen import …` | `from .gen import …` |

Tests live **outside** the package, so they use the installed absolute name:

| File | Now | After |
| --- | --- | --- |
| `test_routing.py:12` | `import gen` | `from svg2gpx import gen` |
| `test_routing.py:13` | `from gen import …` | `from svg2gpx.gen import …` |
| `test_routing.py:14` | `from benchmark import …` | `from svg2gpx.benchmark import …` |
| `test_inner_features.py:16` | `from gen import …` | `from svg2gpx.gen import …` |

That is the **complete** blast radius — 12 import lines across 6 files. Nothing
else references these modules by name.

## 4. Public API — `src/svg2gpx/__init__.py`

Re-export the handful of functions users actually call, plus a friendly
`generate()` wrapper, so the internal module names stay private:

```python
"""svg2gpx — turn an SVG shape into a runnable GPS-art street route."""
from .gen import (
    CONFIG, build_route, extract_shape, extract_contour,
    search_placement, feature_ledger,
)
from .gpx import to_gpx
from .shapes import shape_path, bundled_shapes   # small helper module (§6)

__version__ = "0.1.0"


def generate(svg, *, lat=None, lng=None, **overrides):
    """High-level: place + route an SVG (path or bundled stem) at a location.
    Returns the routed candidate; see gen.search_placement for the object."""
    ...  # thin wrapper over build_grid + search_placement using CONFIG|overrides
```

## 5. `pyproject.toml` (complete, ready to drop in)

```toml
[build-system]
requires = ["setuptools>=61", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "svg2gpx"
version = "0.1.0"
description = "Turn any SVG shape into a runnable GPS-art route on real city streets."
readme = "README.md"
requires-python = ">=3.9"
license = { file = "LICENSE" }
authors = [{ name = "Chieler", email = "chielerli@gmail.com" }]
keywords = ["gps art", "strava art", "gpx", "svg", "running", "route",
            "openstreetmap", "gps drawing", "route art"]
classifiers = [
    "Development Status :: 4 - Beta",
    "Intended Audience :: End Users/Desktop",
    "License :: OSI Approved :: MIT License",
    "Programming Language :: Python :: 3",
    "Topic :: Scientific/Engineering :: GIS",
    "Topic :: Multimedia :: Graphics",
]
dependencies = [
    "numpy", "scipy", "shapely",
    "opencv-python-headless", "skia-python",
]

[project.optional-dependencies]
osm = ["osmnx", "matplotlib"]          # real OpenStreetMap fetch + plotting
dev = ["build", "twine"]

[project.scripts]
svg2gpx = "svg2gpx.cli:main"

[project.urls]
Homepage = "https://github.com/Chieler/svg2gpx"
Issues = "https://github.com/Chieler/svg2gpx/issues"

[tool.setuptools.packages.find]
where = ["src"]

[tool.setuptools.package-data]
svg2gpx = ["shapes/*.svg"]             # bundle the sample shapes into the wheel
```

`requirements.txt` / `requirements-osm.txt` can stay for contributors, or shrink
to `-e .[osm]`. `pyproject.toml` becomes the single source of truth.

## 6. Bundling the shapes as package data

The shapes must ship inside the wheel so `svg2gpx --svg star` works without the
repo checkout. Add `src/svg2gpx/shapes/__init__.py`:

```python
from importlib.resources import files

def shape_path(stem):
    """Filesystem path to a bundled shape by stem ('star') or passthrough for
    an existing path."""
    import os
    if os.path.exists(stem):
        return stem
    p = files("svg2gpx.shapes") / f"{stem}.svg"
    if p.is_file():
        return str(p)
    raise FileNotFoundError(f"no bundled shape '{stem}' and no file at {stem}")

def bundled_shapes():
    return sorted(p.stem for p in files("svg2gpx.shapes").iterdir()
                  if p.suffix == ".svg")
```

Then `benchmark.SHAPES_DIR` and the CLI's `--svg` resolution route through
`shape_path()`, so both a bundled stem and a real path work.

## 7. The CLI — `src/svg2gpx/cli.py`

Move the `argparse` block currently under `gen.py`'s `if __name__ ==
"__main__":` into a `main()` here, add `--gpx PATH`, and call the pipeline:

```python
import argparse
from . import gen
from .gpx import to_gpx
from .shapes import shape_path

def main(argv=None):
    ap = argparse.ArgumentParser(prog="svg2gpx",
        description="Turn an SVG shape into a runnable GPS-art street route.")
    ap.add_argument("--svg", help="shape SVG path or a bundled stem (e.g. star)")
    ap.add_argument("--lat", type=float); ap.add_argument("--lng", type=float)
    ap.add_argument("--radius", type=float, dest="radius_m")
    ap.add_argument("--granularity", type=float)
    ap.add_argument("--graphml"); ap.add_argument("--seed", type=int)
    ap.add_argument("--save", help="write the plot image here")
    ap.add_argument("--gpx", help="also write the route as GPX (needs --lat/--lng)")
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--no-inner-features", action="store_true")
    args = ap.parse_args(argv)
    if args.svg:
        args.svg = shape_path(args.svg)
    ...  # build CONFIG overrides exactly as gen.py's __main__ does today,
         # run the pipeline, then if args.gpx: to_gpx(route_latlon, args.gpx)
```

Add `src/svg2gpx/__main__.py`:
```python
from .cli import main
main()
```

`gen.py` keeps a thin `if __name__ == "__main__": from svg2gpx.cli import main;
main()` (or drops the block entirely — the entry point supersedes it).

## 8. GPX export — `src/svg2gpx/gpx.py`

GPX needs **geographic** coordinates, so it applies only to real-network runs
(`--lat/--lng` or `--graphml`), not the synthetic grid. `chicago_map.py` already
projects routes to WGS84 for its GeoJSON; reuse that lat/lon list. Dependency-free
(write the XML directly — no `gpxpy` needed):

```python
def to_gpx(latlon, path, name="svg2gpx route"):
    """Write a list of (lat, lon) points as a GPX 1.1 track."""
    pts = "\n".join(f'      <trkpt lat="{a:.6f}" lon="{b:.6f}"/>'
                    for a, b in latlon)
    xml = (f'<?xml version="1.0" encoding="UTF-8"?>\n'
           f'<gpx version="1.1" creator="svg2gpx" '
           f'xmlns="http://www.topografix.com/GPX/1/1">\n'
           f'  <trk><name>{name}</name><trkseg>\n{pts}\n'
           f'  </trkseg></trk>\n</gpx>\n')
    open(path, "w").write(xml)
```

A closed running route repeats the start point at the end (already the case), so
the GPX is a loop the watch can follow. This makes the name literal and lands the
Roadmap's first item.

## 9. CI + docs

- `.github/workflows/best-route.yml`: change `python best_route.py …` to install
  the package (`pip install -e .[osm]`) and call `python -m svg2gpx.best_route …`
  (add a `main()`/`__main__` guard to `best_route.py` if missing).
- `README.md`: swap the `python gen.py` / `python chicago_map.py` examples for
  `svg2gpx` / `python -m svg2gpx.chicago_map`, and add a one-line install-from-PyPI.

## 10. Test & verify (gate before publishing)

```bash
pip install -e ".[osm]"
python -m pytest tests/ -q          # (or: python tests/test_routing.py)
python tests/test_inner_features.py
python -m svg2gpx --svg star --seed 42 --no-show --save /tmp/star.png   # smoke
python -c "import svg2gpx; print(svg2gpx.__version__, svg2gpx.bundled_shapes())"
```

All must pass and produce the same routes as today (the move is behaviour-preserving;
only import paths and the invocation change).

## 11. Build & publish

```bash
python -m build                      # -> dist/svg2gpx-0.1.0{.tar.gz,-py3-none-any.whl}
twine check dist/*
twine upload --repository testpypi dist/*     # dry run on TestPyPI first
pip install -i https://test.pypi.org/simple/ svg2gpx   # verify the install
twine upload dist/*                  # real PyPI (needs an API token)
```

Tag the release: `git tag v0.1.0 && git push --tags`.

## 12. Decisions to confirm before executing

- **Author/copyright name** — `LICENSE` and `pyproject` currently say "Chieler";
  swap in your legal name if you want it on PyPI.
- **Version scheme** — static `0.1.0` here, bumped by hand. Fine for now;
  `setuptools-scm` (version from git tags) is the upgrade if you prefer.
- **Heavy core deps** — `skia-python` + `opencv-python-headless` are required for
  SVG rasterization, so they stay in core (not optional). Acceptable, just noted.
- **GPX scope** — geographic runs only (documented); synthetic-grid routes have no
  lat/lon to export.

---

### Execution checklist

- [ ] Create `src/svg2gpx/`, move the 5 modules + `shapes/` in (git mv)
- [ ] Rewrite the 12 imports (§3)
- [ ] Add `__init__.py`, `__main__.py`, `cli.py`, `gpx.py`, `shapes/__init__.py`
- [ ] Add `pyproject.toml` (§5)
- [ ] Route shape lookups through `shape_path()` (§6)
- [ ] Move tests to `tests/`, update their imports
- [ ] Update `best-route.yml` and README usage (§9)
- [ ] `pip install -e ".[osm]"`, run all tests + smoke (§10)
- [ ] `python -m build` + TestPyPI dry run (§11)
- [ ] Tag `v0.1.0`, upload to PyPI
