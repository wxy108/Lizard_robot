#!/usr/bin/env python3
"""Optimize the LOCKED hardware C-wave gait for max forward (+x) displacement.

LOCKED (gait identity kept fixed):
  controller=hardware, body_shape {Front:+1,Back:-1,Tail:-1} (C-shape),
  phi_leg=pi, phi_lat=pi (diagonal trot), down_sign (feet press down),
  body_lock constrained near pi so a LEFT bend still lands the LEFT front
  leg (FL+HR) and RIGHT bend the RIGHT front leg (FR+HL).

TUNED (continuous): body_amp, leg_amp, duty, leg_freq, body_lock (leg<->body
  phase, within pi +/- 1.2), leg_init (initial leg crouch via down_sign).

Optuna TPE, persistent sqlite study (resumable). Objective = forward x [m].
Usage: python scripts/optimize_hw.py [--duration 8] [--budget 38] [--best]
"""
from __future__ import annotations
import argparse, math, sys, time
from pathlib import Path
import numpy as np, mujoco, optuna

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from controllers import GaitController, load_config, apply_overrides

STUDY_DB = "sqlite:///" + str(ROOT / "outputs" / "opt_hw.db")
PI = math.pi
_CFG = _MODEL = _DATA = None


def rollout(cfg, duration):
    ctrl = GaitController(_MODEL, cfg)
    mujoco.mj_resetDataKeyframe(_MODEL, _DATA, 0)
    mujoco.mj_forward(_MODEL, _DATA)
    settle = cfg["gait"]["settle_time"]
    n = int(round(duration / _MODEL.opt.timestep))
    x0 = y0 = None
    for _ in range(n):
        ctrl.update(_DATA)
        mujoco.mj_step(_MODEL, _DATA)
        if x0 is None and _DATA.time >= settle:
            x0, y0 = _DATA.qpos[0], _DATA.qpos[1]
        if abs(_DATA.qpos[2]) > 1.0:
            return dict(fwd=-9.9, side=0.0)
    return dict(fwd=float(_DATA.qpos[0]-x0), side=float(_DATA.qpos[1]-y0))


def objective(trial, duration):
    li = trial.suggest_float("leg_init", -0.3, 0.9)
    cfg = apply_overrides(_CFG, [
        "hardware.body_amp=%g"  % trial.suggest_float("body_amp", 0.2, 1.6),
        "hardware.leg_amp=%g"   % trial.suggest_float("leg_amp", 0.2, 1.6),
        "hardware.duty=%g"      % trial.suggest_float("duty", 0.3, 0.7),
        "hardware.leg_freq=%g"  % trial.suggest_float("leg_freq", 0.3, 3.5),
        "hardware.body_lock=%g" % trial.suggest_float("body_lock", PI-1.2, PI+1.2),
        # leg init crouch applied in each foot's "down" direction:
        "offset.FR=%g" % (-li), "offset.FL=%g" % (-li),
        "offset.HL=%g" % (-li), "offset.HR=%g" % (li),
    ])
    r = rollout(cfg, duration)
    trial.set_user_attr("fwd_mm", 1000*r["fwd"])
    trial.set_user_attr("side_mm", 1000*r["side"])
    return r["fwd"]


def main():
    global _CFG, _MODEL, _DATA
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=8.0)
    ap.add_argument("--budget", type=float, default=38.0)
    ap.add_argument("--best", action="store_true")
    args = ap.parse_args()
    _CFG = load_config(str(ROOT / "configs" / "gait.yaml"))
    _CFG["controller"] = "hardware"
    _MODEL = mujoco.MjModel.from_xml_path(str(ROOT / _CFG["simulation"]["model"]))
    _DATA = mujoco.MjData(_MODEL)
    study = optuna.create_study(direction="maximize", study_name="lizard_hw",
                                storage=STUDY_DB, load_if_exists=True)
    if not args.best:
        optuna.logging.set_verbosity(optuna.logging.WARNING)
        t_end = time.time() + args.budget; n = 0
        while time.time() < t_end:
            study.optimize(lambda tr: objective(tr, args.duration), n_trials=1); n += 1
        print("ran %d trials (total %d)" % (n, len(study.trials)))
    b = study.best_trial
    print("BEST fwd=%.0f mm  side=%.0f mm  (%d trials)"
          % (b.user_attrs["fwd_mm"], b.user_attrs["side_mm"], len(study.trials)))
    for k, v in b.params.items():
        print("  %-12s %.3f" % (k, v))


if __name__ == "__main__":
    main()
