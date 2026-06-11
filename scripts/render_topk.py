#!/usr/bin/env python3
"""Render the top-K trials from the straight-line Optuna study as videos.

Picks the K best DISTINCT trials (dedup by rounded params), and for the
one selected by --rank renders a 10 s side-view mp4 into outputs/videos/.

Usage:  python scripts/render_topk.py --rank 0   # 0 = best
"""
from __future__ import annotations
import argparse, sys, math
from pathlib import Path
import numpy as np, mujoco, optuna

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from controllers import GaitController, load_config, apply_overrides

STUDY_DB = "sqlite:///" + str(ROOT / "outputs" / "opt_straight.db")


def cfg_from_params(base, p):
    m = p["leg_off"]
    return apply_overrides(base, [
        "spine.amplitude=%g" % p["spine_amp"],
        "spine.wavenumber=%g" % p["wavenumber"],
        "spine.turn_trim=%g" % p["turn_trim"],
        "legs.amplitude=%g" % p["leg_amp"],
        "legs.phase_offset=%g" % p["phase"],
        "offset.FR=%g" % -m, "offset.FL=%g" % -m,
        "offset.HL=%g" % -m, "offset.HR=%g" % m,
    ])


def top_distinct(study, k):
    trials = [t for t in study.trials if t.value is not None and t.value > -1]
    trials.sort(key=lambda t: t.value, reverse=True)
    out, seen = [], set()
    for t in trials:
        key = tuple(round(v, 2) for v in t.params.values())
        if key in seen:
            continue
        seen.add(key); out.append(t)
        if len(out) >= k:
            break
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rank", type=int, required=True)
    ap.add_argument("--k", type=int, default=5)
    ap.add_argument("--duration", type=float, default=10.0)
    args = ap.parse_args()

    base = load_config(str(ROOT / "configs" / "gait.yaml"))
    study = optuna.load_study(study_name="lizard_straight", storage=STUDY_DB)
    tops = top_distinct(study, args.k)
    t = tops[args.rank]
    cfg = cfg_from_params(base, t.params)

    model = mujoco.MjModel.from_xml_path(str(ROOT / cfg["simulation"]["model"]))
    data = mujoco.MjData(model)
    ctrl = GaitController(model, cfg)
    mujoco.mj_resetDataKeyframe(model, data, 0); mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, height=720, width=1280)
    dt = model.opt.timestep
    n = int(round(args.duration / dt)); re = max(1, int(round(1/(30*dt))))
    frames = []
    for s in range(n):
        ctrl.update(data); mujoco.mj_step(model, data)
        if s % re == 0:
            renderer.update_scene(data, camera="track_side")
            frames.append(renderer.render())
    import imageio.v2 as imageio
    name = "top%d_score%.3f_fwd%.0f.mp4" % (args.rank + 1, t.value,
                                            1000 * t.user_attrs.get("fwd_mm", 0) / 1000)
    out = ROOT / "outputs" / "videos" / name
    imageio.mimsave(str(out), frames, fps=30, codec="libx264", quality=8)
    print("rank %d  score=%.3f  fwd=%.0fmm side=%.0fmm yaw=%.1fdeg  -> %s"
          % (args.rank + 1, t.value, t.user_attrs.get("fwd_mm", 0),
             t.user_attrs.get("side_mm", 0), t.user_attrs.get("yaw_deg", 0), name))


if __name__ == "__main__":
    main()
