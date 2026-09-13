#!/usr/bin/env python3
"""Offline CSV plots; Matplotlib is optional and never used by the controller."""
import csv
import json
import math
import sys
from pathlib import Path


def main():
    folder = Path(sys.argv[1])
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        raise SystemExit("Matplotlib unavailable; CSV preserved. No installation attempted.")
    with (folder / "trajectory.csv").open() as f:
        rows = list(csv.DictReader(f))
    with (folder / "path.csv").open() as f:
        points = [(float(r["x"]), float(r["y"])) for r in csv.DictReader(f)]
    actual = [(float(r["x"]), float(r["y"])) for r in rows]
    errors = []
    for x, y in actual:
        best = float("inf")
        for a, b in zip(points, points[1:]):
            dx, dy = b[0]-a[0], b[1]-a[1]
            den = dx*dx+dy*dy
            t = max(0, min(1, ((x-a[0])*dx+(y-a[1])*dy)/den)) if den else 0
            best = min(best, math.hypot(x-a[0]-t*dx, y-a[1]-t*dy))
        errors.append(best)
    metrics = dict(error_definition="Euclidean distance to nearest path segment, all samples",
                   samples=len(rows), mean_error=sum(errors)/len(errors),
                   rmse=math.sqrt(sum(e*e for e in errors)/len(errors)), max_error=max(errors),
                   min_x=min(x for x,y in actual), max_x=max(x for x,y in actual),
                   min_y=min(y for x,y in actual), max_y=max(y for x,y in actual))
    (folder / "offline_metrics.json").write_text(json.dumps(metrics, indent=2))
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(*zip(*points), "--", label="Desired")
    ax.plot(*zip(*actual), label="Gazebo")
    ax.scatter(*actual[0], label="Start", s=40)
    ax.scatter(*actual[-1], label="Finish", marker="x", s=50)
    ax.set(xlabel="x (m)", ylabel="y (m)", title=folder.name, aspect="equal")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(folder / "trajectory.png", dpi=150)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(9, 3))
    t0 = float(rows[0]["timestamp"])
    ax.plot([float(r["timestamp"])-t0 for r in rows], errors)
    ax.set(xlabel="Simulation time (s)", ylabel="Tracking error (m)", title=folder.name)
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(folder / "tracking_error.png", dpi=150)
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
