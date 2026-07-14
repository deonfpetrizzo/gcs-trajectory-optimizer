import argparse
import csv
import glob
import os

import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D  
import numpy as np

TAG_COLORS = {
    "AERIAL": "tab:blue",
    "TRANS": "tab:orange",
}
DEFAULT_COLOR = "tab:gray"

GROUND_PALETTE = ["tab:green", "tab:red", "tab:purple", "tab:brown",
                  "tab:olive", "tab:cyan", "tab:pink"]


def run_style(tag, name, ground_color):
    if tag == "GROUND":
        return ground_color, f"{name} GROUND"
    return TAG_COLORS.get(tag, DEFAULT_COLOR), tag


def vehicle_name(path):
    base = os.path.splitext(os.path.basename(path))[0]
    return base[len("traj_"):] if base.startswith("traj_") else base


def load_traj(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append((int(r["seg_idx"]), r["tag"],
                         float(r["x"]), float(r["y"]), float(r["z"])))
    runs = []
    for seg, tag, x, y, z in rows:
        if runs and runs[-1][0] == seg:
            runs[-1][2].append((x, y, z))
        else:
            runs.append((seg, tag, [(x, y, z)]))
    return [(seg, tag, np.asarray(pts, float)) for seg, tag, pts in runs]


def flatten(runs):
    pts = np.vstack([p for _, _, p in runs])
    tags = np.concatenate([np.full(len(p), tag) for _, tag, p in runs])
    return pts, tags


def kinematics(pts, dt):
    n = len(pts)
    t = np.arange(n) * dt
    if n < 2:
        return t, np.zeros(n), np.zeros(n)
    vel = np.gradient(pts, dt, axis=0)
    acc = np.gradient(vel, dt, axis=0)
    return t, np.linalg.norm(vel, axis=1), np.linalg.norm(acc, axis=1)


def plot_traj(ax, runs, seen_tags, name, ground_color):
    for seg, tag, pts in runs:
        color, key = run_style(tag, name, ground_color)
        label = key if key not in seen_tags else None
        seen_tags.add(key)
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], color=color, lw=1.6, label=label)


def plot_scalar_by_tag(ax, t, vals, tags, seen_tags, name, ground_color):
    n = len(tags)
    i = 0
    while i < n:
        j = i
        while j + 1 < n and tags[j + 1] == tags[i]:
            j += 1
        color, key = run_style(tags[i], name, ground_color)
        label = key if key not in seen_tags else None
        seen_tags.add(key)
        sl = slice(i, min(j + 2, n))
        ax.plot(t[sl], vals[sl], color=color, lw=1.3, label=label)
        i = j + 1


def set_equal_aspect(ax, allpts):
    p = np.vstack(allpts)
    mins, maxs = p.min(0), p.max(0)
    center = (mins + maxs) / 2
    r = (maxs - mins).max() / 2 or 1.0
    ax.set_xlim(center[0] - r, center[0] + r)
    ax.set_ylim(center[1] - r, center[1] + r)
    ax.set_zlim(-5, 10)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    default_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "trajs")
    ap.add_argument("files", nargs="*", help="trajectory CSV files (default: all in --dir)")
    ap.add_argument("--dir", default=default_dir,
                    help="directory to scan when no files given (default: ../trajs)")
    ap.add_argument("--separate", action="store_true", help="one column per vehicle")
    ap.add_argument("--dt", type=float, default=0.05,
                    help="timestep for |v|/|a| finite differencing (default: 0.05)")
    ap.add_argument("--save", help="save figure to this path instead of showing")
    args = ap.parse_args()

    files = args.files or sorted(glob.glob(os.path.join(args.dir, "*.csv")))
    if not files:
        ap.error(f"no trajectory CSVs found (looked in '{args.dir}')")

    data = []
    for f in files:
        runs = load_traj(f)
        if not runs:
            continue
        pts, tags = flatten(runs)
        t, speed, accmag = kinematics(pts, args.dt)
        data.append(dict(name=vehicle_name(f), runs=runs, pts=pts, tags=tags,
                         t=t, speed=speed, acc=accmag))
    if not data:
        ap.error("all trajectory files were empty")

    allpts = [d["pts"] for d in data]
    ground_colors = {d["name"]: GROUND_PALETTE[i % len(GROUND_PALETTE)]
                     for i, d in enumerate(data)}

    if args.separate:
        cols = len(data)
        fig = plt.figure(figsize=(6 * cols, 11))
        gs = GridSpec(3, cols, height_ratios=[3, 1, 1], figure=fig)
        for i, d in enumerate(data):
            gc = ground_colors[d["name"]]
            ax3d = fig.add_subplot(gs[0, i], projection="3d")
            plot_traj(ax3d, d["runs"], set(), d["name"], gc)
            ax3d.set_title(d["name"])
            ax3d.set_xlabel("x"); ax3d.set_ylabel("y"); ax3d.set_zlabel("z")
            set_equal_aspect(ax3d, allpts)
            ax3d.legend(loc="upper left", fontsize=8)

            axv = fig.add_subplot(gs[1, i])
            plot_scalar_by_tag(axv, d["t"], d["speed"], d["tags"], set(), d["name"], gc)
            axv.set_ylabel("|v| [m/s]"); axv.grid(True, alpha=0.3)

            axa = fig.add_subplot(gs[2, i], sharex=axv)
            plot_scalar_by_tag(axa, d["t"], d["acc"], d["tags"], set(), d["name"], gc)
            axa.set_ylabel("|a| [m/s^2]"); axa.set_xlabel("t [s]"); axa.grid(True, alpha=0.3)
    else:
        fig = plt.figure(figsize=(10, 11))
        gs = GridSpec(3, 1, height_ratios=[3, 1, 1], figure=fig)
        ax3d = fig.add_subplot(gs[0], projection="3d")
        seen = set()
        for d in data:
            plot_traj(ax3d, d["runs"], seen, d["name"], ground_colors[d["name"]])
        ax3d.set_xlabel("x"); ax3d.set_ylabel("y"); ax3d.set_zlabel("z")
        set_equal_aspect(ax3d, allpts)
        ax3d.legend(loc="upper left", fontsize=9)

        axv = fig.add_subplot(gs[1])
        axa = fig.add_subplot(gs[2], sharex=axv)
        for d in data:
            gc = ground_colors[d["name"]]
            axv.plot(d["t"], d["speed"], lw=1.3, color=gc, label=d["name"])
            axa.plot(d["t"], d["acc"], lw=1.3, color=gc, label=d["name"])
        axv.set_ylabel("|v| [m/s]"); axv.grid(True, alpha=0.3); axv.legend(fontsize=8)
        axa.set_ylabel("|a| [m/s^2]"); axa.set_xlabel("t [s]"); axa.grid(True, alpha=0.3)

    fig.tight_layout()
    if args.save:
        fig.savefig(args.save, dpi=130)
        print(f"saved {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
