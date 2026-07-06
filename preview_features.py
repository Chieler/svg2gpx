"""
Visual check for inner-feature extraction (extract_shape in gen.py).

Renders one panel per shapes/*.svg: the rasterized ink in light gray, the
outer outline dashed purple, closed inner features (holes, separate elements)
in orange, and open inner features (interior detail strokes) in green.

Usage:
    python preview_features.py                       # all shapes -> preview PNG
    python preview_features.py --shape Crow --shape face
"""

import argparse
import glob
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from gen import _render_ink_mask, extract_shape

HERE = os.path.dirname(os.path.abspath(__file__))
SHAPES_DIR = os.path.join(HERE, "shapes")


def main():
    ap = argparse.ArgumentParser(description="Preview inner-feature extraction.")
    ap.add_argument("--shape", action="append",
                    help="shape stem(s) to preview (default: all)")
    ap.add_argument("--img-size", type=int, default=1024)
    ap.add_argument("--out", default=os.path.join(HERE, "inner_features_preview.png"))
    args = ap.parse_args()

    if args.shape:
        svgs = [os.path.join(SHAPES_DIR, f"{s}.svg") for s in args.shape]
    else:
        svgs = sorted(glob.glob(os.path.join(SHAPES_DIR, "*.svg")))

    n = len(svgs)
    cols = min(5, n)
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(4.4 * cols, 4.4 * rows))
    axes = np.atleast_1d(axes).ravel()
    for ax in axes[n:]:
        ax.set_axis_off()

    for ax, svg in zip(axes, svgs):
        name = os.path.splitext(os.path.basename(svg))[0]
        spec = extract_shape(svg, args.img_size)
        ink = _render_ink_mask(svg, args.img_size)
        ax.imshow(ink[::-1], cmap="Greys", alpha=0.15, origin="lower",
                  extent=(0, 1, 0, 1))
        ax.plot(spec.outer[:, 0], spec.outer[:, 1], "--", color="#7b5ea7",
                lw=1.8, label="outer")
        for f in spec.inners:
            color = "#e4572e" if f.closed else "#2e933c"
            ax.plot(f.pts[:, 0], f.pts[:, 1], color=color, lw=2.2)
        n_closed = sum(1 for f in spec.inners if f.closed)
        n_open = len(spec.inners) - n_closed
        ax.set_title(f"{name}: {n_closed} closed / {n_open} open")
        ax.set(xlim=(0, 1), ylim=(0, 1), aspect="equal")
        ax.set_axis_off()

    fig.suptitle("Inner-feature extraction (orange = closed loop, green = open path)",
                 fontsize=14)
    fig.tight_layout()
    fig.savefig(args.out, dpi=140, bbox_inches="tight")
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
