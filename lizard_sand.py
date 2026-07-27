#!/usr/bin/env python3
"""Run the Lizard V1 on granular media using triangle-level 3D RFT.

The existing CPG controller supplies joint targets. Each mesh triangle below
the sand surface is evaluated by the RFT model, and its force is applied at
that triangle's own MuJoCo site. Headless runs save motion, sinkage, force, and
triangle-activation diagnostics.
"""

from __future__ import annotations

import argparse
import json
import tempfile
import time
from datetime import datetime
from pathlib import Path

import mujoco
import numpy as np
import yaml

from controllers import GaitController, apply_overrides, load_config
from sim_fxn_lib import (
    initialize_sites_on_mesh,
    load_and_process_mesh,
    rft_3D_body_full_mat,
)

ROOT = Path(__file__).resolve().parent

BODIES = ["Mid", "Front", "FR", "FL", "Back", "HR", "HL", "Tail"]
FOOT_TIPS = ["FR_foot_tip", "FL_foot_tip", "HR_foot_tip", "HL_foot_tip"]

RFTCOEFF = 3.75  # Quikrete play sand (Li et al.)
MESH_SCALE = 1  # lizard RFT meshes are already in metres
ALPHA = 0.5
FORCE_SITE_GROUP = 5  # hidden by default in MuJoCo visual options


def parse_args():
    parser = argparse.ArgumentParser(description="Lizard on sand (3D RFT-SiM)")
    parser.add_argument("--config", default=str(ROOT / "configs" / "gait.yaml"))
    parser.add_argument("--model", default=str(ROOT / "Lizard_Sand.xml"))
    parser.add_argument("--duration", type=float, default=6.0)

    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--view", action="store_true", help="open the live MuJoCo viewer")
    mode.add_argument("--video", default=None, help="write an MP4 instead of opening a viewer")

    parser.add_argument("--camera", default="track_side")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--alpha",
        type=float,
        default=ALPHA,
        help="new-force weight for per-triangle exponential smoothing (0 < alpha <= 1)",
    )
    parser.add_argument(
        "--rft-coeff",
        type=float,
        default=RFTCOEFF,
        help="granular-material RFT coefficient (3.75 for Quikrete play sand)",
    )
    parser.add_argument(
        "--show-force-sites",
        nargs="?",
        const="all",
        choices=["all", *BODIES],
        default=None,
        metavar="BODY",
        help="show one body's force sites, or all sites if BODY is omitted",
    )
    parser.add_argument(
        "--set",
        dest="overrides",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="gait config override",
    )
    parser.add_argument("--no-save", action="store_true")
    args = parser.parse_args()

    if args.duration <= 0:
        parser.error("--duration must be positive")
    if not 0 < args.alpha <= 1:
        parser.error("--alpha must satisfy 0 < alpha <= 1")
    if args.rft_coeff <= 0:
        parser.error("--rft-coeff must be positive")
    if args.fps <= 0:
        parser.error("--fps must be positive")
    return args


def require_id(model, object_type, name):
    object_id = mujoco.mj_name2id(model, object_type, name)
    if object_id < 0:
        raise ValueError(f"Required MuJoCo object not found: {name}")
    return object_id


def prepare_video_path(raw_path):
    path = Path(raw_path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with tempfile.NamedTemporaryFile(
            dir=path.parent,
            prefix=".rft_write_test_",
            delete=True,
        ):
            pass
    except OSError as exc:
        raise PermissionError(
            f"Video output directory is not writable: {path.parent}"
        ) from exc
    return path


def setup(args):
    cfg = apply_overrides(load_config(args.config), args.overrides)
    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)

    key = require_id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)
    controller = GaitController(model, cfg)

    sand_h_id = require_id(model, mujoco.mjtObj.mjOBJ_SITE, "sand_height")
    sand_z = float(data.site_xpos[sand_h_id, 2])
    print(f"sand surface z = {sand_z:.4f} m")

    floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    if floor_id >= 0:
        floor_z = float(model.geom_pos[floor_id, 2])
        print(f"safety floor z = {floor_z:.4f} m")
        if floor_z >= sand_z:
            raise ValueError(
                "The rigid floor must be below the RFT sand surface; otherwise "
                "contact prevents the robot from entering the RFT region"
            )

    body_dict = {}
    body_ids = {}
    site_ids = {}
    for name in BODIES:
        body_ids[name] = require_id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        stl_path = ROOT / "asset" / f"{name}.STL"
        body, _, faces, mesh = load_and_process_mesh(
            str(stl_path), scale_factor=MESH_SCALE
        )
        ids = initialize_sites_on_mesh(
            model,
            data,
            mesh,
            sitename=f"force_{name}",
            bodyname=name,
        )
        if len(ids) != len(faces):
            raise ValueError(
                f"{name}: {len(ids)} force sites for {len(faces)} mesh triangles"
            )

        body_dict[name] = body
        site_ids[name] = ids
        model.site_group[ids] = FORCE_SITE_GROUP
        if args.show_force_sites in ("all", name):
            model.site_group[ids] = 0
        print(f"  {name}: {len(ids)} force sites, {len(faces)} triangles")

    if args.show_force_sites == "all" and sum(map(len, site_ids.values())) > 10_000:
        print(
            "WARNING: showing every force site exceeds the live viewer's usual "
            "10,000-geometry buffer; choose a body name for a lighter view"
        )

    tip_ids = {
        name: require_id(model, mujoco.mjtObj.mjOBJ_SITE, name)
        for name in FOOT_TIPS
    }
    mujoco.mj_forward(model, data)
    return (
        model,
        data,
        controller,
        cfg,
        sand_z,
        body_dict,
        site_ids,
        tip_ids,
        body_ids,
    )


def make_force_buffers(site_ids):
    return {name: np.zeros((len(site_ids[name]), 3)) for name in BODIES}


def apply_sand_forces(
    model,
    data,
    sand_z,
    body_dict,
    site_ids,
    body_ids,
    smoothed,
    *,
    alpha,
    rft_coeff,
):
    """Compute and apply RFT forces at the current state.

    Returns force and triangle-count diagnostics in ``BODIES`` order.
    """
    body_forces = np.zeros((len(BODIES), 3))
    body_powers = np.zeros(len(BODIES))
    peak_site_forces = np.zeros(len(BODIES))
    submerged_counts = np.zeros(len(BODIES), dtype=int)
    active_counts = np.zeros(len(BODIES), dtype=int)

    for body_index, name in enumerate(BODIES):
        ids = site_ids[name]
        below_surface = data.site_xpos[ids, 2] < sand_z
        submerged_counts[body_index] = int(np.count_nonzero(below_surface))
        if submerged_counts[body_index] == 0:
            smoothed[name].fill(0)
            continue

        body_id = body_ids[name]
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(
            model,
            data,
            mujoco.mjtObj.mjOBJ_BODY,
            body_id,
            velocity,
            0,
        )
        angular_velocity = velocity[:3]
        linear_velocity = velocity[3:]

        _, _, _, _, include, force_matrix, _ = rft_3D_body_full_mat(
            body_dict[name],
            np.asarray(data.xpos[body_id]),
            None,
            linear_velocity,
            angular_velocity,
            RFTCOEFF=rft_coeff,
            sand_height_m=sand_z,
            rotation_matrix=np.asarray(data.xmat[body_id]).reshape(3, 3),
        )

        # rft_3D_body_full_mat follows the reference implementation's
        # body-on-sand convention. MuJoCo needs the equal-and-opposite
        # sand-on-body reaction force (the Crab reference applies ``-F`` too).
        force_matrix = -np.nan_to_num(
            force_matrix, nan=0.0, posinf=0.0, neginf=0.0
        )
        active_counts[body_index] = int(np.count_nonzero(include))

        # Never retain force on a face that is no longer intruding/leading.
        smoothed[name][~include] = 0
        smoothed[name][include] = (
            alpha * force_matrix[include]
            + (1.0 - alpha) * smoothed[name][include]
        )

        active_indices = np.flatnonzero(include)
        for triangle_index in active_indices:
            force = smoothed[name][triangle_index]
            if not np.any(force):
                continue
            site_world = data.site_xpos[ids[triangle_index]]
            mujoco.mj_applyFT(
                model,
                data,
                force,
                np.zeros(3),
                site_world,
                body_id,
                data.qfrc_applied,
            )
            point_velocity = linear_velocity + np.cross(
                angular_velocity, site_world - data.xpos[body_id]
            )
            body_powers[body_index] += float(np.dot(force, point_velocity))
        body_forces[body_index] = np.sum(smoothed[name][include], axis=0)
        if active_indices.size:
            peak_site_forces[body_index] = float(
                np.max(np.linalg.norm(smoothed[name][include], axis=1))
            )

    return (
        body_forces,
        submerged_counts,
        active_counts,
        body_powers,
        peak_site_forces,
    )


def advance_simulation(
    model,
    data,
    sand_z,
    body_dict,
    site_ids,
    body_ids,
    smoothed,
    *,
    alpha,
    rft_coeff,
):
    # Applied forces must be present before mj_step integrates the state.
    data.qfrc_applied[:] = 0
    diagnostics = apply_sand_forces(
        model,
        data,
        sand_z,
        body_dict,
        site_ids,
        body_ids,
        smoothed,
        alpha=alpha,
        rft_coeff=rft_coeff,
    )
    mujoco.mj_step(model, data)
    if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
        raise FloatingPointError(
            f"Non-finite MuJoCo state after stepping at t={data.time:.6f}s"
        )
    return diagnostics


def read_foot_depths(data, tip_ids, sand_z):
    """Return signed foot depth and physical sinkage.

    Signed depth is positive below the surface and negative above it. Sinkage
    is clipped to zero while the foot is above the surface.
    """
    signed_depth = np.asarray(
        [sand_z - data.site_xpos[tip_ids[name], 2] for name in FOOT_TIPS]
    )
    return signed_depth, np.maximum(signed_depth, 0.0)


def build_summary(log, args, sand_z):
    displacement = log["base_pos"][-1] - log["base_pos"][0]
    force_norm = np.linalg.norm(log["rft_force"], axis=1)
    active_any = np.sum(log["active_triangles"], axis=1) > 0
    active_power = log["rft_power"][active_any]
    return {
        "duration_s": float(log["time"][-1] - log["time"][0] + log["dt"]),
        "displacement_x_m": float(displacement[0]),
        "displacement_y_m": float(displacement[1]),
        "sand_surface_z_m": float(sand_z),
        "rft_coefficient": float(args.rft_coeff),
        "force_smoothing_alpha": float(args.alpha),
        "rft_active_step_fraction": float(np.mean(active_any)),
        "peak_total_rft_force_N": float(np.max(force_norm)),
        "peak_site_rft_force_N": float(np.max(log["peak_site_force"])),
        "minimum_rft_power_W": float(np.min(log["rft_power"])),
        "maximum_rft_power_W": float(np.max(log["rft_power"])),
        "positive_rft_power_step_fraction": (
            float(np.mean(active_power > 1e-9)) if active_power.size else 0.0
        ),
        "mean_sinkage_mm": [
            float(value) for value in np.mean(log["sinkage"], axis=0) * 1000
        ],
        "max_sinkage_mm": [
            float(value) for value in np.max(log["sinkage"], axis=0) * 1000
        ],
    }


def print_summary(summary):
    print(f"\nForward distance: {summary['displacement_x_m'] * 1000:.1f} mm")
    print(f"Lateral distance: {summary['displacement_y_m'] * 1000:.1f} mm")
    print(
        "RFT active steps: "
        f"{summary['rft_active_step_fraction'] * 100:.1f}%  "
        f"peak total/site force: {summary['peak_total_rft_force_N']:.3f} / "
        f"{summary['peak_site_rft_force_N']:.3f} N"
    )
    print(
        "RFT power range: "
        f"{summary['minimum_rft_power_W']:.3f} to "
        f"{summary['maximum_rft_power_W']:.3f} W  "
        f"positive active steps: "
        f"{summary['positive_rft_power_step_fraction'] * 100:.2f}%"
    )
    print("Mean / max sinkage per foot (mm):")
    for index, tip in enumerate(FOOT_TIPS):
        print(
            f"  {tip}: {summary['mean_sinkage_mm'][index]:.2f} / "
            f"{summary['max_sinkage_mm'][index]:.2f}"
        )


def main():
    args = parse_args()
    video_path = prepare_video_path(args.video) if args.video else None
    (
        model,
        data,
        controller,
        cfg,
        sand_z,
        body_dict,
        site_ids,
        tip_ids,
        body_ids,
    ) = setup(args)

    dt = float(model.opt.timestep)
    n_steps = max(1, int(round(args.duration / dt)))
    smoothed = make_force_buffers(site_ids)

    if args.view:
        from mujoco import viewer as mj_viewer

        print("\nOpening viewer... close the window to stop.")
        with mj_viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and data.time < args.duration:
                start = time.perf_counter()
                controller.update(data)
                advance_simulation(
                    model,
                    data,
                    sand_z,
                    body_dict,
                    site_ids,
                    body_ids,
                    smoothed,
                    alpha=args.alpha,
                    rft_coeff=args.rft_coeff,
                )
                viewer.sync()
                remaining = dt - (time.perf_counter() - start)
                if remaining > 0:
                    time.sleep(remaining)
        print("viewer closed.")
        return

    log = {
        "time": [],
        "base_pos": [],
        "base_quat": [],
        "ctrl": [],
        "foot_depth": [],
        "sinkage": [],
        "rft_force": [],
        "rft_force_by_body": [],
        "rft_power": [],
        "rft_power_by_body": [],
        "peak_site_force": [],
        "submerged_triangles": [],
        "active_triangles": [],
    }

    renderer = None
    frames = []
    if video_path is not None:
        max_geom = max(10_000, model.nsite + model.ngeom + 1_000)
        renderer = mujoco.Renderer(
            model,
            height=720,
            width=1280,
            max_geom=max_geom,
        )
        render_every = max(1, int(round(1.0 / (args.fps * dt))))

    print(
        f"\nRunning {n_steps} steps ({args.duration}s)  "
        f"alpha={args.alpha}  RFT={args.rft_coeff}..."
    )
    for step in range(n_steps):
        controller.update(data)
        (
            body_forces,
            submerged_counts,
            active_counts,
            body_powers,
            peak_site_forces,
        ) = advance_simulation(
            model,
            data,
            sand_z,
            body_dict,
            site_ids,
            body_ids,
            smoothed,
            alpha=args.alpha,
            rft_coeff=args.rft_coeff,
        )
        foot_depth, sinkage = read_foot_depths(data, tip_ids, sand_z)

        log["time"].append(float(data.time))
        log["base_pos"].append(np.array(data.qpos[:3]))
        log["base_quat"].append(np.array(data.qpos[3:7]))
        log["ctrl"].append(np.array(data.ctrl))
        log["foot_depth"].append(foot_depth)
        log["sinkage"].append(sinkage)
        log["rft_force_by_body"].append(body_forces)
        log["rft_force"].append(np.sum(body_forces, axis=0))
        log["rft_power_by_body"].append(body_powers)
        log["rft_power"].append(np.sum(body_powers))
        log["peak_site_force"].append(peak_site_forces)
        log["submerged_triangles"].append(submerged_counts)
        log["active_triangles"].append(active_counts)

        if renderer is not None and step % render_every == 0:
            renderer.update_scene(data, camera=args.camera)
            frames.append(renderer.render())
        if step % 1000 == 0:
            print(
                f"  step {step}/{n_steps}  t={data.time:.2f}s  "
                f"x={data.qpos[0]:.3f}m  "
                f"active={int(np.sum(active_counts))}"
            )

    if renderer is not None:
        renderer.close()

    for key in log:
        log[key] = np.asarray(log[key])
    log["dt"] = np.asarray(dt)
    summary = build_summary(log, args, sand_z)
    print_summary(summary)

    if video_path is not None:
        import imageio.v2 as imageio

        imageio.mimsave(
            str(video_path), frames, fps=args.fps, codec="libx264", quality=8
        )
        print(f"video: {video_path}")

    if not args.no_save:
        output = (
            ROOT
            / "outputs"
            / "sand"
            / datetime.now().strftime("run_%m_%d_%Y_%H%M%S_%f")
        )
        output.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            output / "results.npz",
            **log,
            foot_order=np.asarray(FOOT_TIPS),
            body_order=np.asarray(BODIES),
            sand_surface_z=np.asarray(sand_z),
            rft_coefficient=np.asarray(args.rft_coeff),
            force_smoothing_alpha=np.asarray(args.alpha),
        )
        with open(output / "config.yaml", "w", encoding="utf-8") as stream:
            yaml.safe_dump(cfg, stream, sort_keys=False)
        with open(output / "summary.json", "w", encoding="utf-8") as stream:
            json.dump(summary, stream, indent=2)
        print(f"data: {output}")


if __name__ == "__main__":
    main()
