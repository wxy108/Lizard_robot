#!/usr/bin/env python3
"""Plot a results.npz produced by run.py.

Usage:
    python scripts/plot_results.py outputs/data/run_*/results.npz [--save]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

SPINE = ["Front", "Back", "Tail"]
LEGS = ["FR", "FL", "HR", "HL"]
ACT = SPINE + LEGS  # actuator order in models/lizard.xml
NQ_FREE = 7  # freejoint qpos size
# joint qpos order in the model tree (differs from actuator order):
QPOS_JOINTS = ["Front", "FR", "FL", "Back", "HR", "HL", "Tail"]


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("npz", type=Path)
    p.add_argument("--save", action="store_true", help="save png next to npz")
    args = p.parse_args()

    d = np.load(args.npz)
    t = d["time"]
    qpos = d["qpos"]
    ctrl = d["ctrl"]
    base = d["base_pos"]
    rpy = np.degrees(d["base_rpy"])
    tau = d["actuator_force"]

    qj = {n: qpos[:, NQ_FREE + i] for i, n in enumerate(QPOS_JOINTS)}
    cj = {n: ctrl[:, i] for i, n in enumerate(ACT)}

    fig, ax = plt.subplots(3, 2, figsize=(13, 10))
    fig.suptitle(f"Lizard V1 — {args.npz.parent.name}")

    a = ax[0, 0]
    a.plot(base[:, 0], base[:, 1])
    a.plot(base[0, 0], base[0, 1], "go", label="start")
    a.plot(base[-1, 0], base[-1, 1], "rs", label="end")
    a.set_xlabel("x [m]"); a.set_ylabel("y [m]")
    a.set_title("Base XY trajectory"); a.axis("equal"); a.legend(); a.grid(True)

    a = ax[0, 1]
    for i, lbl in enumerate(["roll", "pitch", "yaw"]):
        a.plot(t, rpy[:, i], label=lbl)
    a.set_xlabel("t [s]"); a.set_ylabel("[deg]")
    a.set_title("Base orientation"); a.legend(); a.grid(True)

    a = ax[1, 0]
    for n in SPINE:
        a.plot(t, np.degrees(qj[n]), label=f"q {n}")
        a.plot(t, np.degrees(cj[n]), "--", lw=0.8)
    a.set_xlabel("t [s]"); a.set_ylabel("[deg]")
    a.set_title("Spine joints (solid=actual, dashed=target)")
    a.legend(fontsize=8); a.grid(True)

    a = ax[1, 1]
    for n in LEGS:
        a.plot(t, np.degrees(np.mod(qj[n], 2 * np.pi)), lw=0.8, label=f"q {n}")
    a.set_xlabel("t [s]"); a.set_ylabel("[deg, wrapped]")
    a.set_title("Leg joints"); a.legend(fontsize=8); a.grid(True)

    a = ax[2, 0]
    for i, n in enumerate(ACT):
        a.plot(t, tau[:, i], lw=0.7, label=n)
    a.set_xlabel("t [s]"); a.set_ylabel("torque [Nm]")
    a.set_title("Actuator torques (XL430 limit ±1.4)"); a.legend(fontsize=7, ncol=2)
    a.grid(True)

    a = ax[2, 1]
    a.plot(t, base[:, 2] * 1000)
    a.set_xlabel("t [s]"); a.set_ylabel("z [mm]")
    a.set_title("Base height"); a.grid(True)

    fig.tight_layout()
    if args.save:
        out = args.npz.parent / "plots.png"
        fig.savefig(out, dpi=140)
        print(f"saved {out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
