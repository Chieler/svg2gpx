"""
Checks for inner-feature extraction (run: python test_inner_features.py).

Covers the three feature kinds and backward compatibility:
  * simple filled polygons have no inner features;
  * a donut's hole is found as one closed loop, disconnected from the outline;
  * a face's eyes/mouth (ink fully disconnected from the outer ring) are found;
  * line-art shapes (Crow/Knight) yield interior detail strokes / eye loops;
  * extract_contour still returns exactly the outer outline of extract_shape.
"""

import os

import numpy as np

from gen import extract_contour, extract_shape

HERE = os.path.dirname(os.path.abspath(__file__))
SVG = lambda name: os.path.join(HERE, "shapes", f"{name}.svg")
IMG = 1024


def check(cond, msg):
    assert cond, msg
    print(f"  ok: {msg}")


def main():
    for name in ("circle", "square", "star", "lshape"):
        spec = extract_shape(SVG(name), IMG)
        check(spec.inners == [], f"{name}: no inner features")

    donut = extract_shape(SVG("donut"), IMG)
    check(len(donut.inners) == 1 and donut.inners[0].closed,
          "donut: exactly one closed hole")
    hole = donut.inners[0].pts
    r = np.hypot(*(hole - hole.mean(axis=0)).T).mean()
    check(0.12 < r < 0.25, f"donut hole radius plausible (r={r:.3f})")

    face = extract_shape(SVG("face"), IMG)
    check(len(face.inners) == 3, f"face: eyes + mouth found ({len(face.inners)})")
    check(all(f.closed for f in face.inners), "face: all features are loops")

    shark = extract_shape(SVG("Shark"), IMG)
    check(len(shark.inners) >= 1 and any(f.closed for f in shark.inners),
          f"Shark: eye hole found ({len(shark.inners)} feature(s))")

    for name in ("Crow", "Knight"):
        spec = extract_shape(SVG(name), IMG)
        check(len(spec.inners) >= 1, f"{name}: interior detail found "
                                     f"({len(spec.inners)} feature(s))")

    for name in ("circle", "star", "Crow", "Shark", "donut", "face"):
        spec = extract_shape(SVG(name), IMG)
        outline = extract_contour(SVG(name), IMG)
        check(np.array_equal(spec.outer, outline),
              f"{name}: extract_contour == extract_shape().outer")
        check(np.allclose(outline[0], outline[-1]), f"{name}: outline closed")
        for f in spec.inners:
            if f.closed:
                assert np.allclose(f.pts[0], f.pts[-1]), f"{name}: loop not closed"

    print("\nall inner-feature checks passed")


if __name__ == "__main__":
    main()
