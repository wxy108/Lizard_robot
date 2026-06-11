#!/usr/bin/env python3
"""Optimize gait.frequency and gait.wave_speed to maximize forward (+x)
displacement. amplitude / phase_offset / wavenumber stay fixed.

Loads the model once, runs a 2D grid of headless physics rollouts, then
refines around the best cell.

Usage:
    python scripts/optimize_gait.py [--duration 6] [--f 0.5 3.0 6] [--s 0 2 5]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from controllers import GaitController, load_config, apply_overrides


def rollout(model, data, cfg, freq, speed, duration):
    cfg = apply_overrides(cfg, ["gait.frequency=%g" % freq,
                                "gait.wave_speed=%g" % speed])
    ctrl = GaitController(model, cfg)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    settle = cfg["gait"]["settle_time"]
    n = int(round(duration / model.opt.timestep))
    x0 = y0 = None
    for _ in range(n):
        ctrl.update(data)
        mujoco.mj_step(model, data)
        if x0 is None and data.time >= settle:
            x0, y0 = data.qpos[0], data.qpos[1]
        if abs(data.qpos[2]) > 1.0:   # diverged
            return dict(fwd=-9.99, side=0.0)
    walk_t = duration - settle
    return dict(fwd=float(data.qpos[0] - x0),
                side=float(data.qpos[1] - y0),
                fwd_speed=1000 * (data.qpos[0] - x0) / walk_t)


def grid(model, data, cfg, fs, ss, duration, results):
    print("  freq\\speed " + " ".join("%6.2f" % s for s in ss))
    for f in fs:
        row = []
        for s in ss:
            r = rollout(model, data, cfg, f, s, duration)
            results.append((f, s, r))
            row.append(1000 * r["fwd"])
        print("  %5.2f Hz  " % f + " ".join("%+6.0f" % v for v in row) + "   (mm fwd)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=6.0)
    ap.add_argument("--f", nargs=3, type=float, default=[0.5, 3.0, 6],
                    metavar=("LO", "HI", "N"))
    ap.add_argument("--s", nargs=3, type=float, default=[0.0, 2.0, 5],
                    metavar=("LO", "HI", "N"))
    args = ap.parse_args()

    cfg = load_config(str(ROOT / "configs" / "gait.yaml"))
    model = mujoco.MjModel.from_xml_path(str(ROOT / cfg["simulation"]["model"]))
    data = mujoco.MjData(model)

    fs = np.linspace(args.f[0], args.f[1], int(args.f[2]))
    ss = np.linspace(args.s[0], args.s[1], int(args.s[2]))
    results = []
    print("=== coarse grid (forward x displacement, mm) ===")
    grid(model, data, cfg, fs, ss, args.duration, results)

    bf, bs, br = max(results, key=lambda r: r[2]["fwd"])
    print("\nrefine around f=%.2f s=%.2f" % (bf, bs))
    df = (args.f[1]-args.f[0])/max(int(args.f[2])-1, 1)
    ds = (args.s[1]-args.s[0])/max(int(args.s[2])-1, 1)
    fs2 = np.linspace(max(0.2, bf-df), bf+df, 5)
    ss2 = np.linspace(max(0.0, bs-ds), bs+ds, 5)
    grid(model, data, cfg, fs2, ss2, args.duration, results)

    bf, bs, br = max(results, key=lambda r: r[2]["fwd"])
    print("\nBEST  frequency=%.3f Hz  wave_speed=%.3f  ->  fwd %.1f mm "
          "(%.2f mm/s), side %.1f mm"
          % (bf, bs, 1000*br["fwd"], br["fwd_speed"], 1000*br["side"]))


if __name__ == "__main__":
    main()
