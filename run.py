#!/usr/bin/env python3
"""Run the Lizard V1 gait simulation.

Examples
--------
# Interactive viewer (default config)
python run.py

# Headless 10 s run with video
python run.py --duration 10 --headless --video test.mp4

# Override any config value from the CLI (dot notation)
python run.py --headless --video trot.mp4 \
    --set gait.frequency=1.5 --set spine.wavenumber=1.0 --set legs.mode=oscillate

Outputs (headless): outputs/data/run_<timestamp>/
    results.npz   - full time series (qpos, qvel, ctrl, sensors, base pose)
    summary.json  - distance, speed, heading drift, pitch/roll RMS, CoT
    config.yaml   - exact config used
"""

from __future__ import annotations

import argparse
import json
import shutil
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

import mujoco

from controllers import GaitController, apply_overrides, load_config

ROOT = Path(__file__).resolve().parent


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Lizard V1 MuJoCo gait simulation")
    p.add_argument("--config", default=str(ROOT / "configs" / "gait.yaml"))
    p.add_argument("--duration", type=float, default=None, help="[s] sim duration")
    p.add_argument("--headless", action="store_true", help="no viewer")
    p.add_argument("--video", default=None, help="mp4 output path (implies offscreen render)")
    p.add_argument("--camera", default="track_side", help="camera for video")
    p.add_argument("--fps", type=int, default=30, help="video framerate")
    p.add_argument("--set", dest="overrides", action="append", default=[],
                   metavar="KEY=VALUE", help="config override, e.g. spine.wavenumber=1.0")
    p.add_argument("--no-save", action="store_true", help="skip npz/json output")
    return p.parse_args()


def quat_to_rpy(q: np.ndarray) -> np.ndarray:
    """wxyz quaternion -> roll, pitch, yaw [rad]."""
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def main() -> None:
    args = parse_args()
    cfg = apply_overrides(load_config(args.config), args.overrides)
    duration = args.duration if args.duration is not None else float(
        cfg["simulation"]["duration"])

    model_path = ROOT / cfg["simulation"]["model"]
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY,
                            cfg["simulation"].get("keyframe", "stand"))
    if key >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)

    controller = GaitController(model, cfg)
    dt = model.opt.timestep
    n_steps = int(round(duration / dt))

    # ---------------- logging buffers ----------------
    log_every = max(1, int(round(0.002 / dt)))  # 500 Hz log rate
    n_log = n_steps // log_every + 1
    log = {
        "time": np.zeros(n_log),
        "qpos": np.zeros((n_log, model.nq)),
        "qvel": np.zeros((n_log, model.nv)),
        "ctrl": np.zeros((n_log, model.nu)),
        "actuator_force": np.zeros((n_log, model.nu)),
        "base_pos": np.zeros((n_log, 3)),
        "base_rpy": np.zeros((n_log, 3)),
        "sensordata": np.zeros((n_log, model.nsensordata)),
    }
    li = 0
    energy = 0.0  # integral of sum |tau * qdot| over actuated joints

    # actuated joint dof indices for energy computation
    act_dof = [model.jnt_dofadr[model.actuator_trnid[i, 0]] for i in range(model.nu)]

    # ---------------- video ----------------
    renderer = None
    frames = []
    if args.video:
        renderer = mujoco.Renderer(model, height=720, width=1280)
        render_every = max(1, int(round(1.0 / (args.fps * dt))))

    # ---------------- viewer or headless loop ----------------
    t_wall = time.time()
    if args.headless or args.video:
        for step in range(n_steps):
            controller.update(data)
            mujoco.mj_step(model, data)
            tau = data.actuator_force
            qd = np.array([data.qvel[d] for d in act_dof])
            energy += float(np.sum(np.abs(tau * qd))) * dt
            if step % log_every == 0:
                log["time"][li] = data.time
                log["qpos"][li] = data.qpos
                log["qvel"][li] = data.qvel
                log["ctrl"][li] = data.ctrl
                log["actuator_force"][li] = data.actuator_force
                log["base_pos"][li] = data.qpos[:3]
                log["base_rpy"][li] = quat_to_rpy(data.qpos[3:7])
                log["sensordata"][li] = data.sensordata
                li += 1
            if renderer is not None and step % render_every == 0:
                renderer.update_scene(data, camera=args.camera)
                frames.append(renderer.render())
    else:
        from mujoco import viewer as mj_viewer
        with mj_viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and data.time < duration:
                step_start = time.time()
                controller.update(data)
                mujoco.mj_step(model, data)
                viewer.sync()
                lag = dt - (time.time() - step_start)
                if lag > 0:
                    time.sleep(lag)
        return

    wall = time.time() - t_wall
    for k in log:
        log[k] = log[k][:li]

    # ---------------- summary metrics ----------------
    p0, p1 = log["base_pos"][0], log["base_pos"][-1]
    walk_t = duration - cfg["gait"]["settle_time"]
    disp = p1[:2] - p0[:2]
    distance = float(np.linalg.norm(disp))
    rpy = log["base_rpy"]
    mass = float(model.body_mass.sum())
    summary = {
        "duration_s": duration,
        "wall_time_s": round(wall, 1),
        "distance_xy_m": round(distance, 4),
        "displacement_x_m": round(float(disp[0]), 4),
        "displacement_y_m": round(float(disp[1]), 4),
        "mean_speed_mm_s": round(1000 * distance / max(walk_t, 1e-9), 2),
        "speed_body_lengths_s": round(distance / max(walk_t, 1e-9) / 0.45, 3),
        "heading_drift_deg": round(float(np.degrees(rpy[-1, 2] - rpy[0, 2])), 2),
        "roll_rms_deg": round(float(np.degrees(np.sqrt(np.mean(rpy[:, 0] ** 2)))), 2),
        "pitch_rms_deg": round(float(np.degrees(np.sqrt(np.mean(rpy[:, 1] ** 2)))), 2),
        "base_z_mean_m": round(float(np.mean(log["base_pos"][:, 2])), 4),
        "energy_J": round(energy, 4),
        "cost_of_transport": round(energy / (mass * 9.81 * max(distance, 1e-9)), 2),
        "fell_over": bool(np.max(np.abs(np.degrees(rpy[:, :2]))) > 60),
    }
    print(json.dumps(summary, indent=2))

    # ---------------- save ----------------
    if args.video:
        import imageio.v2 as imageio
        video_path = Path(args.video)
        video_path.parent.mkdir(parents=True, exist_ok=True)
        imageio.mimsave(str(video_path), frames, fps=args.fps,
                        codec="libx264", quality=8)
        print(f"video: {video_path}")

    if not args.no_save:
        out = ROOT / "outputs" / "data" / datetime.now().strftime("run_%m_%d_%Y_%H%M%S")
        out.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(out / "results.npz", **log)
        with open(out / "summary.json", "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2)
        with open(out / "config.yaml", "w", encoding="utf-8") as f:
            yaml.safe_dump(cfg, f, sort_keys=False)
        print(f"data: {out}")


if __name__ == "__main__":
    main()
