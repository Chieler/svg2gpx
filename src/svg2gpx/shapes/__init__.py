"""Bundled sample shapes, shipped as package data (see pyproject.toml)."""
import os
from importlib.resources import files


def shape_path(stem_or_path):
    """Resolve a bundled shape by stem ("star") or pass through a real path.

    An existing filesystem path (relative to cwd or absolute) always wins, so
    a user's own SVG is never shadowed by a same-named bundled shape.
    """
    if os.path.exists(stem_or_path):
        return stem_or_path
    stem = os.path.splitext(os.path.basename(stem_or_path))[0]
    p = files(__name__) / f"{stem}.svg"
    if p.is_file():
        return str(p)
    raise FileNotFoundError(
        f"no bundled shape '{stem}' and no file at {stem_or_path!r} "
        f"(bundled: {', '.join(bundled_shapes())})")


def bundled_shapes():
    """Sorted stems of every bundled shape (e.g. 'star', 'Horse', 'donut')."""
    return sorted(p.stem for p in files(__name__).iterdir() if p.suffix == ".svg")
