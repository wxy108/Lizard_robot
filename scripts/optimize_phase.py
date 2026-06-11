#!/usr/bin/env python3
"""Optimize legs.phase_offset to maximize forward (+x) speed.

Loads the model once and sweeps the leg-vs-body phase offset over a grid
(then refines around the best), running a headless physics rollout for
each value. Reports forward displacement and prints the best phase.

Usage:
    python scripts/optimize_phase.py [--duration 8] [--grid 16] [--refine]
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import mujoco

from controllers import GaitController, load_config, apply_overrides

ROOT = Path(__file__).resolve().parents[1]


def rollout(model, data, cfg, phase, duration):
    cfg = apply_overrides(cfg, ["legs.phase_offset=%g" % phase])
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
    xf, yf = data.qpos[0], data.qpos[1]
    walk_t = duration - settle
    dx = xf - x0
    dy = yf - y0
    return dict(fwd=dx, side=dy, dist=float(np.hypot(dx, dy)),
                fwd_speed_mm_s=1000 * dx / walk_t)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=8.0)
    ap.add_argument("--grid", type=int, default=16)
    ap.add_argument("--refine", action="store_true", default=True)
    args = ap.parse_args()

    cfg = load_config(str(ROOT / "configs" / "gait.yaml"))
    model = mujoco.MjModel.from_xml_path(str(ROOT / cfg["simulation"]["model"]))
    data = mujoco.MjData(model)

    phases = np.linspace(0.0, 1.0, args.grid, endpoint=False)
    results = []
    print("phase[cyc]  fwd_x[mm]  side_y[mm]  fwd_speed[mm/s]")
    for ph in phases:
        r = rollout(model, data, cfg, ph, args.duration)
        results.append((ph, r))
        print("  %5.3f     %+7.1f    %+7.1f      %+7.2f"
              % (ph, 1000 * r["fwd"], 1000 * r["side"], r["fwd_speed_mm_s"]))

    best = max(results, key=lambda kr: kr[1]["fwd"])
    bph = best[0]
    if args.refine:
        step = 1.0 / args.grid
        fine = np.linspace(bph - step, bph + step, 9)
        print("\nrefine around %.3f:" % bph)
        for ph in fine:
            r = rollout(model, data, cfg, ph % 1.0, args.duration)
            results.append((ph % 1.0, r))
            print("  %5.3f     %+7.1f    %+7.1f      %+7.2f"
                  % (ph % 1.0, 1000 * r["fwd"], 1000 * r["side"], r["fwd_speed_mm_s"]))
        best = max(results, key=lambda kr: kr[1]["fwd"])

    print("\nBEST phase_offset = %.3f cycles  ->  fwd %.1f mm  (%.2f mm/s), side %.1f mm"
          % (best[0], 1000 * best[1]["fwd"], best[1]["fwd_speed_mm_s"],
             1000 * best[1]["side"]))


if __name__ == "__main__":
    main()
