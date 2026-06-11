#!/usr/bin/env python3
"""Optimize the gait for MAXIMUM STRAIGHT forward travel.

Tunable: spine.amplitude, spine.wavenumber, spine.turn_trim,
         legs.amplitude, legs.phase_offset, leg offset magnitude.
Fixed:   gait.frequency (0.5 Hz), gait.wave_speed (1.0).

Objective = forward_x  -  W_SIDE*|y_final|  -  W_YAW*|yaw_final|
so the optimum goes far forward AND straight (small lateral + heading
drift). Optuna TPE, persistent sqlite study (resumable).

Usage:
    python scripts/optimize_search.py [--duration 8] [--budget 38] [--best]
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import mujoco
import optuna

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from controllers import GaitController, load_config, apply_overrides

STUDY_DB = "sqlite:///" + str(ROOT / "outputs" / "opt_straight.db")
W_SIDE = 2.0      # penalty weight on |lateral drift| [per m]
W_YAW = 0.40      # penalty weight on |final heading| [per rad]
_CFG = _MODEL = _DATA = None


def _yaw(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def rollout(cfg, duration):
    ctrl = GaitController(_MODEL, cfg)
    mujoco.mj_resetDataKeyframe(_MODEL, _DATA, 0)
    mujoco.mj_forward(_MODEL, _DATA)
    settle = cfg["gait"]["settle_time"]
    n = int(round(duration / _MODEL.opt.timestep))
    x0 = y0 = yaw0 = None
    for _ in range(n):
        ctrl.update(_DATA)
        mujoco.mj_step(_MODEL, _DATA)
        if x0 is None and _DATA.time >= settle:
            x0, y0, yaw0 = _DATA.qpos[0], _DATA.qpos[1], _yaw(_DATA.qpos[3:7])
        if abs(_DATA.qpos[2]) > 1.0:
            return dict(fwd=-9.9, side=0.0, yaw=0.0, score=-9.9)
    fwd = float(_DATA.qpos[0] - x0)
    side = float(_DATA.qpos[1] - y0)
    yaw = float(_yaw(_DATA.qpos[3:7]) - yaw0)
    score = fwd - W_SIDE * abs(side) - W_YAW * abs(yaw)
    return dict(fwd=fwd, side=side, yaw=math.degrees(yaw), score=score)


def objective(trial, duration):
    m = trial.suggest_float("leg_off", 0.30, 1.20)
    cfg = apply_overrides(_CFG, [
        "spine.amplitude=%g"   % trial.suggest_float("spine_amp", 0.2, 1.2),
        "spine.wavenumber=%g"  % trial.suggest_float("wavenumber", 0.0, 2.0),
        "spine.turn_trim=%g"   % trial.suggest_float("turn_trim", -0.4, 0.4),
        "legs.amplitude=%g"    % trial.suggest_float("leg_amp", 0.2, 1.0),
        "legs.phase_offset=%g" % trial.suggest_float("phase", 0.0, 1.0),
        "offset.FR=%g" % -m, "offset.FL=%g" % -m,
        "offset.HL=%g" % -m, "offset.HR=%g" % m,
    ])
    r = rollout(cfg, duration)
    trial.set_user_attr("fwd_mm", 1000 * r["fwd"])
    trial.set_user_attr("side_mm", 1000 * r["side"])
    trial.set_user_attr("yaw_deg", r["yaw"])
    return r["score"]


def main():
    global _CFG, _MODEL, _DATA
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=8.0)
    ap.add_argument("--budget", type=float, default=38.0)
    ap.add_argument("--best", action="store_true")
    args = ap.parse_args()

    _CFG = load_config(str(ROOT / "configs" / "gait.yaml"))
    _MODEL = mujoco.MjModel.from_xml_path(str(ROOT / _CFG["simulation"]["model"]))
    _DATA = mujoco.MjData(_MODEL)

    study = optuna.create_study(direction="maximize", study_name="lizard_straight",
                                storage=STUDY_DB, load_if_exists=True)
    if not args.best:
        optuna.logging.set_verbosity(optuna.logging.WARNING)
        t_end = time.time() + args.budget
        n = 0
        while time.time() < t_end:
            study.optimize(lambda tr: objective(tr, args.duration), n_trials=1)
            n += 1
        print("ran %d trials (total %d)" % (n, len(study.trials)))

    b = study.best_trial
    print("BEST score=%.3f | fwd=%.0f mm  side=%.0f mm  yaw=%.1f deg  (%d trials)"
          % (b.value, b.user_attrs["fwd_mm"], b.user_attrs["side_mm"],
             b.user_attrs["yaw_deg"], len(study.trials)))
    for k, v in b.params.items():
        print("  %-12s %.3f" % (k, v))


if __name__ == "__main__":
    main()
