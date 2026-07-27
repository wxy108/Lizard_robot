"""Generate the 3-view x 3-scenario locomotion evidence matrix.

Scenarios
---------
1. original detailed CAD meshes on the rigid flat floor;
2. simplified external-envelope meshes on RFT sand, force sites hidden;
3. the same RFT sand replay with all triangle force sites visible.

Each video contains a robot center-of-mass trajectory and an eight-component
binary contact timeline. Machine-readable CSV/NPZ data, contact-event
intervals, static diagrams, hashes, and a manifest are written beside the
videos.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

import cv2
import imageio.v2 as imageio
import mujoco
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from controllers import GaitController, apply_overrides, load_config  # noqa: E402
from lizard_sand import (  # noqa: E402
    ALPHA,
    BODIES,
    FORCE_SITE_GROUP,
    RFTCOEFF,
    advance_simulation,
    make_force_buffers,
    setup as setup_sand,
)
from scripts.compose_video_matrix import compose_master_video  # noqa: E402


VIEW_CAMERAS = {
    "top": "track_top",
    "side": "track_side",
    "diag45": "diag",
}
SCENARIO_LABELS = {
    "rigid_original": "Original CAD | rigid flat ground",
    "sand_simplified": "Simplified envelope | RFT sand | sites hidden",
    "sand_simplified_sites": (
        "Simplified envelope | RFT sand | all RFT sites visible"
    ),
}
CONTACT_COLORS = np.asarray(
    [
        (91, 192, 235),
        (245, 166, 35),
        (126, 211, 33),
        (189, 16, 224),
        (248, 231, 28),
        (74, 144, 226),
        (208, 2, 27),
        (80, 227, 194),
    ],
    dtype=np.uint8,
)


@dataclass
class SimulationBundle:
    model: mujoco.MjModel
    time: np.ndarray
    com: np.ndarray
    base_pos: np.ndarray
    contact: np.ndarray
    render_qpos: np.ndarray
    render_steps: np.ndarray
    extra: dict[str, np.ndarray]
    force_site_ids: dict[str, np.ndarray] | None = None


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def robot_com(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    masses = np.asarray(model.body_mass[1:], dtype=float)
    positions = np.asarray(data.xipos[1:], dtype=float)
    positive = masses > 0
    if not np.any(positive):
        raise ValueError("Robot model has no positive-mass bodies")
    return np.average(positions[positive], axis=0, weights=masses[positive])


def named_component_for_body(
    body_id: int,
    body_parent_ids: np.ndarray,
    component_body_ids: dict[str, int],
) -> str | None:
    """Return the nearest named component at or above ``body_id``."""
    names_by_id = {body: name for name, body in component_body_ids.items()}
    current = int(body_id)
    while current > 0:
        if current in names_by_id:
            return names_by_id[current]
        current = int(body_parent_ids[current])
    return None


def rigid_component_contact(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    floor_geom_id: int,
    component_body_ids: dict[str, int],
) -> np.ndarray:
    contact = np.zeros(len(BODIES), dtype=np.uint8)
    body_index = {name: index for index, name in enumerate(BODIES)}
    for index in range(data.ncon):
        pair = data.contact[index]
        geom1 = int(pair.geom1)
        geom2 = int(pair.geom2)
        if geom1 == floor_geom_id:
            other = geom2
        elif geom2 == floor_geom_id:
            other = geom1
        else:
            continue
        component = named_component_for_body(
            int(model.geom_bodyid[other]),
            np.asarray(model.body_parentid),
            component_body_ids,
        )
        if component is not None:
            contact[body_index[component]] = 1
    return contact


def binary_intervals(
    time_s: np.ndarray,
    values: np.ndarray,
    dt: float,
) -> list[tuple[float, float]]:
    """Convert one binary contact series into half-open time intervals."""
    binary = np.asarray(values, dtype=bool)
    if binary.ndim != 1 or len(binary) != len(time_s):
        raise ValueError("binary contact values must match the time vector")
    padded = np.concatenate(([False], binary, [False])).astype(np.int8)
    changes = np.diff(padded)
    starts = np.flatnonzero(changes == 1)
    ends = np.flatnonzero(changes == -1)
    result = []
    for start, end in zip(starts, ends, strict=True):
        start_time = float(time_s[start])
        end_time = (
            float(time_s[end])
            if end < len(time_s)
            else float(time_s[-1] + dt)
        )
        result.append((start_time, end_time))
    return result


def simulation_frame_steps(n_steps: int, dt: float, fps: int) -> np.ndarray:
    render_every = max(1, int(round(1.0 / (fps * dt))))
    steps = np.arange(0, n_steps, render_every, dtype=int)
    if steps[-1] != n_steps - 1:
        steps = np.append(steps, n_steps - 1)
    return steps


def simulate_rigid(args: argparse.Namespace) -> SimulationBundle:
    cfg = apply_overrides(load_config(args.config), args.overrides)
    model_path = ROOT / cfg["simulation"]["model"]
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    key_name = cfg["simulation"].get("keyframe", "stand")
    key_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_KEY, key_name
    )
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)

    controller = GaitController(model, cfg)
    floor_geom_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "floor"
    )
    if floor_geom_id < 0:
        raise ValueError("Rigid model is missing floor geom")
    component_body_ids = {
        name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        for name in BODIES
    }
    if any(value < 0 for value in component_body_ids.values()):
        raise ValueError("Rigid model is missing one or more component bodies")

    dt = float(model.opt.timestep)
    n_steps = max(1, int(round(args.duration / dt)))
    frame_steps = simulation_frame_steps(n_steps, dt, args.fps)
    frame_lookup = set(int(value) for value in frame_steps)

    times = np.empty(n_steps)
    com = np.empty((n_steps, 3))
    base_pos = np.empty((n_steps, 3))
    contact = np.zeros((n_steps, len(BODIES)), dtype=np.uint8)
    qpos_frames = []
    saved_steps = []

    print(f"Simulating rigid ground: {n_steps} steps", flush=True)
    for step in range(n_steps):
        controller.update(data)
        mujoco.mj_step(model, data)
        times[step] = data.time
        com[step] = robot_com(model, data)
        base_pos[step] = data.qpos[:3]
        contact[step] = rigid_component_contact(
            model,
            data,
            floor_geom_id,
            component_body_ids,
        )
        if step in frame_lookup:
            qpos_frames.append(np.asarray(data.qpos).copy())
            saved_steps.append(step)

    return SimulationBundle(
        model=model,
        time=times,
        com=com,
        base_pos=base_pos,
        contact=contact,
        render_qpos=np.asarray(qpos_frames),
        render_steps=np.asarray(saved_steps, dtype=int),
        extra={},
    )


def simulate_sand(args: argparse.Namespace) -> SimulationBundle:
    setup_args = SimpleNamespace(
        config=str(Path(args.config).resolve()),
        model=str((ROOT / "Lizard_Sand.xml").resolve()),
        duration=args.duration,
        view=False,
        video=None,
        camera="track_side",
        fps=args.fps,
        alpha=args.alpha,
        rft_coeff=args.rft_coeff,
        show_force_sites=None,
        overrides=args.overrides,
        no_save=True,
    )
    (
        model,
        data,
        controller,
        _,
        sand_z,
        body_dict,
        site_ids,
        _,
        body_ids,
    ) = setup_sand(setup_args)
    smoothed = make_force_buffers(site_ids)

    dt = float(model.opt.timestep)
    n_steps = max(1, int(round(args.duration / dt)))
    frame_steps = simulation_frame_steps(n_steps, dt, args.fps)
    frame_lookup = set(int(value) for value in frame_steps)

    times = np.empty(n_steps)
    com = np.empty((n_steps, 3))
    base_pos = np.empty((n_steps, 3))
    contact = np.zeros((n_steps, len(BODIES)), dtype=np.uint8)
    submerged = np.zeros((n_steps, len(BODIES)), dtype=np.int32)
    active_triangles = np.zeros((n_steps, len(BODIES)), dtype=np.int32)
    max_penetration = np.zeros((n_steps, len(BODIES)), dtype=float)
    qpos_frames = []
    saved_steps = []

    print(f"Simulating RFT sand: {n_steps} steps", flush=True)
    for step in range(n_steps):
        controller.update(data)
        (
            _,
            submerged_counts,
            active_counts,
            _,
            _,
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
        times[step] = data.time
        com[step] = robot_com(model, data)
        base_pos[step] = data.qpos[:3]
        submerged[step] = submerged_counts
        active_triangles[step] = active_counts
        contact[step] = active_counts > 0
        for body_index, name in enumerate(BODIES):
            max_penetration[step, body_index] = max(
                0.0,
                sand_z - float(np.min(data.site_xpos[site_ids[name], 2])),
            )
        if step in frame_lookup:
            qpos_frames.append(np.asarray(data.qpos).copy())
            saved_steps.append(step)

    return SimulationBundle(
        model=model,
        time=times,
        com=com,
        base_pos=base_pos,
        contact=contact,
        render_qpos=np.asarray(qpos_frames),
        render_steps=np.asarray(saved_steps, dtype=int),
        extra={
            "submerged_triangles": submerged,
            "active_triangles": active_triangles,
            "max_penetration_m": max_penetration,
            "sand_surface_z": np.asarray(sand_z),
        },
        force_site_ids={
            name: np.asarray(ids, dtype=int) for name, ids in site_ids.items()
        },
    )


def _draw_text(
    image: np.ndarray,
    text: str,
    origin: tuple[int, int],
    scale: float = 0.55,
    color: tuple[int, int, int] = (235, 235, 235),
    thickness: int = 1,
) -> None:
    cv2.putText(
        image,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def _project_to_rect(
    values: np.ndarray,
    rect: tuple[int, int, int, int],
) -> np.ndarray:
    x0, y0, width, height = rect
    values = np.asarray(values, dtype=float)
    low = np.min(values, axis=0)
    high = np.max(values, axis=0)
    span = np.maximum(high - low, 1e-6)
    low -= 0.08 * span
    high += 0.08 * span
    normalized = (values - low) / np.maximum(high - low, 1e-9)
    pixels = np.empty_like(normalized)
    pixels[:, 0] = x0 + normalized[:, 0] * width
    pixels[:, 1] = y0 + (1.0 - normalized[:, 1]) * height
    return np.rint(pixels).astype(np.int32)


def compose_dashboard(
    rendered_rgb: np.ndarray,
    *,
    scenario_label: str,
    view: str,
    current_step: int,
    time_s: np.ndarray,
    com: np.ndarray,
    contact: np.ndarray,
    panel_width: int,
    sand_metrics: tuple[float, int] | None = None,
) -> np.ndarray:
    """Attach COM trail and binary contact diagram to one RGB frame."""
    height, width = rendered_rgb.shape[:2]
    panel = np.full((height, panel_width, 3), (18, 22, 30), dtype=np.uint8)
    label_parts = scenario_label.split(" | ")
    _draw_text(panel, label_parts[0], (18, 27), scale=0.56, thickness=1)
    _draw_text(
        panel,
        " | ".join(label_parts[1:]),
        (18, 50),
        scale=0.44,
        color=(195, 215, 230),
    )
    current_time = float(time_s[current_step])
    _draw_text(
        panel,
        f"view={view}   t={current_time:.3f} s",
        (18, 75),
        scale=0.48,
        color=(180, 205, 230),
    )

    trajectory_top = 101
    trajectory_height = max(115, int(height * 0.30))
    trajectory_rect = (
        54,
        trajectory_top + 18,
        panel_width - 78,
        trajectory_height - 36,
    )
    cv2.rectangle(
        panel,
        (trajectory_rect[0], trajectory_rect[1]),
        (
            trajectory_rect[0] + trajectory_rect[2],
            trajectory_rect[1] + trajectory_rect[3],
        ),
        (75, 85, 100),
        1,
    )
    axes = (0, 2) if view == "side" else (0, 1)
    axis_label = "COM X-Z trajectory" if view == "side" else "COM X-Y trajectory"
    _draw_text(panel, axis_label, (18, trajectory_top), scale=0.47)
    points = _project_to_rect(com[:, axes], trajectory_rect)
    if len(points) > 1:
        cv2.polylines(panel, [points], False, (75, 90, 110), 1, cv2.LINE_AA)
        cv2.polylines(
            panel,
            [points[: current_step + 1]],
            False,
            (75, 220, 255),
            2,
            cv2.LINE_AA,
        )
    cv2.circle(
        panel,
        tuple(points[current_step]),
        5,
        (255, 90, 80),
        -1,
        cv2.LINE_AA,
    )
    displacement = com[current_step] - com[0]
    _draw_text(
        panel,
        f"dCOM=({1000*displacement[0]:+.1f}, "
        f"{1000*displacement[1]:+.1f}, "
        f"{1000*displacement[2]:+.1f}) mm",
        (18, trajectory_top + trajectory_height + 4),
        scale=0.43,
        color=(175, 205, 220),
    )

    chart_top = trajectory_top + trajectory_height + 34
    if sand_metrics is not None:
        penetration_mm, active_triangle_count = sand_metrics
        _draw_text(
            panel,
            f"max penetration={penetration_mm:.2f} mm | "
            f"active triangles={active_triangle_count}",
            (18, chart_top - 8),
            scale=0.40,
            color=(255, 200, 120),
        )
        chart_top += 24
    chart_bottom = height - 30
    chart_left = 64
    chart_right = panel_width - 18
    chart_width = max(1, chart_right - chart_left)
    row_height = max(13, (chart_bottom - chart_top) // len(BODIES))
    _draw_text(
        panel,
        "Component contact (0/1; colored = 1)",
        (18, chart_top - 10),
        scale=0.43,
    )
    duration = max(float(time_s[-1]), 1e-9)
    timeline_indices = np.rint(
        np.linspace(0, len(time_s) - 1, chart_width)
    ).astype(int)
    for body_index, body in enumerate(BODIES):
        y0 = chart_top + body_index * row_height
        y1 = y0 + row_height - 3
        _draw_text(
            panel,
            body,
            (10, y1 - 1),
            scale=0.37,
            color=tuple(int(v) for v in CONTACT_COLORS[body_index]),
        )
        cv2.rectangle(
            panel,
            (chart_left, y0),
            (chart_right, y1),
            (42, 48, 58),
            -1,
        )
        active = np.flatnonzero(
            contact[timeline_indices, body_index] > 0
        )
        if active.size:
            color = tuple(int(v) for v in CONTACT_COLORS[body_index])
            panel[y0 : y1 + 1, chart_left + active] = color
    cursor_x = chart_left + int(
        round(current_time / duration * (chart_width - 1))
    )
    cv2.line(
        panel,
        (cursor_x, chart_top),
        (cursor_x, chart_top + len(BODIES) * row_height - 3),
        (255, 255, 255),
        1,
    )
    _draw_text(panel, "0", (chart_left, height - 8), scale=0.34)
    _draw_text(
        panel,
        f"{duration:.2f}s",
        (chart_right - 42, height - 8),
        scale=0.34,
    )
    return np.concatenate((rendered_rgb, panel), axis=1)


def render_video(
    bundle: SimulationBundle,
    *,
    output_path: Path,
    scenario: str,
    view: str,
    camera: str,
    show_force_sites: bool,
    fps: int,
    width: int,
    height: int,
    panel_width: int,
) -> None:
    if bundle.force_site_ids is not None:
        group = 0 if show_force_sites else FORCE_SITE_GROUP
        for ids in bundle.force_site_ids.values():
            bundle.model.site_group[ids] = group

    output_path.parent.mkdir(parents=True, exist_ok=True)
    data = mujoco.MjData(bundle.model)
    max_geom = max(
        10_000,
        bundle.model.nsite + bundle.model.ngeom + 2_000,
    )
    renderer = mujoco.Renderer(
        bundle.model,
        height=height,
        width=width,
        max_geom=max_geom,
    )
    writer = imageio.get_writer(
        str(output_path),
        fps=fps,
        codec="libx264",
        quality=8,
        macro_block_size=2,
    )
    try:
        for frame_index, qpos in enumerate(bundle.render_qpos):
            data.qpos[:] = qpos
            mujoco.mj_forward(bundle.model, data)
            renderer.update_scene(data, camera=camera)
            rendered = renderer.render()
            step = int(bundle.render_steps[frame_index])
            dashboard = compose_dashboard(
                rendered,
                scenario_label=SCENARIO_LABELS[scenario],
                view=view,
                current_step=step,
                time_s=bundle.time,
                com=bundle.com,
                contact=bundle.contact,
                panel_width=panel_width,
                sand_metrics=(
                    (
                        1000.0
                        * float(
                            np.max(
                                bundle.extra["max_penetration_m"][step]
                            )
                        ),
                        int(
                            np.sum(
                                bundle.extra["active_triangles"][step]
                            )
                        ),
                    )
                    if "max_penetration_m" in bundle.extra
                    else None
                ),
            )
            writer.append_data(dashboard)
    finally:
        writer.close()
        renderer.close()


def write_timeline_csv(path: Path, bundle: SimulationBundle) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_s", *BODIES])
        for index, time_value in enumerate(bundle.time):
            writer.writerow(
                [
                    f"{time_value:.9f}",
                    *[int(value) for value in bundle.contact[index]],
                ]
            )


def write_com_csv(path: Path, bundle: SimulationBundle) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "time_s",
                "com_x_m",
                "com_y_m",
                "com_z_m",
                "base_x_m",
                "base_y_m",
                "base_z_m",
            ]
        )
        for index, time_value in enumerate(bundle.time):
            writer.writerow(
                [
                    f"{time_value:.9f}",
                    *[f"{value:.9f}" for value in bundle.com[index]],
                    *[f"{value:.9f}" for value in bundle.base_pos[index]],
                ]
            )


def write_penetration_csv(path: Path, bundle: SimulationBundle) -> None:
    penetration = bundle.extra["max_penetration_m"]
    active = bundle.extra["active_triangles"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        header = ["time_s"]
        for body in BODIES:
            header.extend(
                (
                    f"{body}_max_penetration_mm",
                    f"{body}_active_triangles",
                )
            )
        writer.writerow(header)
        for step, time_value in enumerate(bundle.time):
            row: list[str | int] = [f"{time_value:.9f}"]
            for body_index in range(len(BODIES)):
                row.extend(
                    (
                        f"{1000.0 * penetration[step, body_index]:.6f}",
                        int(active[step, body_index]),
                    )
                )
            writer.writerow(row)


def write_event_rows(
    writer: csv.writer,
    scenario: str,
    bundle: SimulationBundle,
) -> None:
    dt = (
        float(np.median(np.diff(bundle.time)))
        if len(bundle.time) > 1
        else 0.0
    )
    for body_index, body in enumerate(BODIES):
        intervals = binary_intervals(
            bundle.time,
            bundle.contact[:, body_index],
            dt,
        )
        for event_index, (start, end) in enumerate(intervals):
            writer.writerow(
                [
                    scenario,
                    body,
                    event_index,
                    f"{start:.9f}",
                    f"{end:.9f}",
                    f"{end - start:.9f}",
                ]
            )


def write_static_contact_diagram(
    path: Path,
    scenario_label: str,
    bundle: SimulationBundle,
) -> None:
    width = 1600
    height = 520
    image = np.full((height, width, 3), (248, 249, 252), dtype=np.uint8)
    _draw_text(
        image,
        scenario_label,
        (28, 36),
        scale=0.8,
        color=(20, 30, 45),
        thickness=2,
    )
    _draw_text(
        image,
        "Per-component contact state: 0 = no contact, 1 = contact",
        (28, 68),
        scale=0.55,
        color=(60, 70, 85),
    )
    left = 115
    right = width - 35
    top = 92
    row_height = 45
    duration = max(float(bundle.time[-1]), 1e-9)
    timeline_width = right - left
    timeline_indices = np.rint(
        np.linspace(0, len(bundle.time) - 1, timeline_width)
    ).astype(int)
    for body_index, body in enumerate(BODIES):
        y0 = top + body_index * row_height
        y1 = y0 + row_height - 8
        _draw_text(
            image,
            body,
            (28, y0 + 24),
            scale=0.55,
            color=tuple(int(v) for v in CONTACT_COLORS[body_index]),
            thickness=2,
        )
        cv2.rectangle(image, (left, y0), (right, y1), (220, 224, 231), -1)
        active = np.flatnonzero(
            bundle.contact[timeline_indices, body_index] > 0
        )
        if active.size:
            color = tuple(int(v) for v in CONTACT_COLORS[body_index])
            image[y0 : y1 + 1, left + active] = color
    for tick in np.linspace(0.0, duration, 7):
        x = left + int(round(tick / duration * (right - left)))
        cv2.line(image, (x, top), (x, top + 8 * row_height), (190, 195, 205), 1)
        _draw_text(
            image,
            f"{tick:.2f}s",
            (x - 18, top + 8 * row_height + 26),
            scale=0.38,
            color=(65, 70, 80),
        )
    if not cv2.imwrite(str(path), cv2.cvtColor(image, cv2.COLOR_RGB2BGR)):
        raise OSError(f"Failed to write contact diagram: {path}")


def bundle_summary(bundle: SimulationBundle) -> dict:
    displacement = bundle.com[-1] - bundle.com[0]
    result = {
        "steps": int(len(bundle.time)),
        "duration_s": float(bundle.time[-1]),
        "com_displacement_m": [float(value) for value in displacement],
        "contact_duty_by_component": {
            body: float(np.mean(bundle.contact[:, index]))
            for index, body in enumerate(BODIES)
        },
    }
    if "max_penetration_m" in bundle.extra:
        result["maximum_penetration_mm_by_component"] = {
            body: float(
                1000.0
                * np.max(bundle.extra["max_penetration_m"][:, index])
            )
            for index, body in enumerate(BODIES)
        }
    return result


def save_bundle(path: Path, bundle: SimulationBundle) -> None:
    np.savez_compressed(
        path,
        time=bundle.time,
        com=bundle.com,
        base_pos=bundle.base_pos,
        contact=bundle.contact,
        body_order=np.asarray(BODIES),
        render_steps=bundle.render_steps,
        **bundle.extra,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default=str(ROOT / "configs" / "gait.yaml"),
    )
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--panel-width", type=int, default=480)
    parser.add_argument("--alpha", type=float, default=ALPHA)
    parser.add_argument("--rft-coeff", type=float, default=RFTCOEFF)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument(
        "--set",
        dest="overrides",
        action="append",
        default=[],
        metavar="KEY=VALUE",
    )
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.fps <= 0:
        parser.error("--fps must be positive")
    if args.width < 480 or args.height < 480 or args.panel_width < 420:
        parser.error("video and panel dimensions are too small for annotations")
    if not 0 < args.alpha <= 1:
        parser.error("--alpha must satisfy 0 < alpha <= 1")
    if args.rft_coeff <= 0:
        parser.error("--rft-coeff must be positive")
    return args


def git_metadata() -> dict:
    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=ROOT,
            text=True,
        ).strip()
        dirty = bool(
            subprocess.check_output(
                ["git", "status", "--porcelain"],
                cwd=ROOT,
                text=True,
            ).strip()
        )
        return {"commit": commit, "dirty": dirty}
    except (OSError, subprocess.CalledProcessError):
        return {"commit": None, "dirty": None}


def main() -> None:
    args = parse_args()
    output_dir = args.output_dir
    if output_dir is None:
        output_dir = (
            ROOT
            / "outputs"
            / "video_matrix"
            / datetime.now().strftime("run_%Y-%m-%d_%H%M%S_%f")
        )
    output_dir = output_dir.resolve()
    if output_dir.exists():
        raise FileExistsError(output_dir)
    video_dir = output_dir / "videos"
    analysis_dir = output_dir / "analysis"
    video_dir.mkdir(parents=True)
    analysis_dir.mkdir()

    rigid = simulate_rigid(args)
    sand = simulate_sand(args)
    bundles = {
        "rigid_original": rigid,
        "sand_simplified": sand,
        "sand_simplified_sites": sand,
    }

    save_bundle(analysis_dir / "rigid_original.npz", rigid)
    save_bundle(analysis_dir / "sand_rft.npz", sand)
    write_timeline_csv(
        analysis_dir / "rigid_original_contact_timeline.csv", rigid
    )
    write_timeline_csv(analysis_dir / "sand_rft_contact_timeline.csv", sand)
    write_com_csv(analysis_dir / "rigid_original_com.csv", rigid)
    write_com_csv(analysis_dir / "sand_rft_com.csv", sand)
    write_penetration_csv(analysis_dir / "sand_rft_penetration.csv", sand)
    write_static_contact_diagram(
        analysis_dir / "rigid_original_contact_diagram.png",
        SCENARIO_LABELS["rigid_original"],
        rigid,
    )
    write_static_contact_diagram(
        analysis_dir / "sand_rft_contact_diagram.png",
        SCENARIO_LABELS["sand_simplified"],
        sand,
    )
    with (analysis_dir / "contact_events.csv").open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "scenario",
                "component",
                "event_index",
                "start_s",
                "end_s",
                "duration_s",
            ]
        )
        write_event_rows(writer, "rigid_original", rigid)
        write_event_rows(writer, "sand_rft", sand)

    video_paths = []
    for scenario, bundle in bundles.items():
        for view, camera in VIEW_CAMERAS.items():
            output_path = video_dir / scenario / f"{view}.mp4"
            print(f"Rendering {scenario}/{view}", flush=True)
            render_video(
                bundle,
                output_path=output_path,
                scenario=scenario,
                view=view,
                camera=camera,
                show_force_sites=scenario == "sand_simplified_sites",
                fps=args.fps,
                width=args.width,
                height=args.height,
                panel_width=args.panel_width,
            )
            video_paths.append(output_path)

    overview_path = video_dir / "video_matrix_overview.mp4"
    print("Composing 3x3 overview + 3 analysis panels", flush=True)
    overview_layout = compose_master_video(
        video_dir=video_dir,
        output_path=overview_path,
        render_width=args.width,
        render_height=args.height,
        panel_width=args.panel_width,
        fps=args.fps,
    )

    config = apply_overrides(load_config(args.config), args.overrides)
    with (output_dir / "resolved_gait_config.yaml").open(
        "w", encoding="utf-8"
    ) as handle:
        yaml.safe_dump(config, handle, sort_keys=False)

    artifacts = {}
    for path in sorted(output_dir.rglob("*")):
        if path.is_file():
            relative = path.relative_to(output_dir).as_posix()
            artifacts[relative] = {
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
    manifest = {
        "schema_version": 1,
        "created_at": datetime.now().isoformat(),
        "command": [sys.executable, *sys.argv],
        "git": git_metadata(),
        "configuration": {
            "duration_s": args.duration,
            "fps": args.fps,
            "render_width": args.width,
            "render_height": args.height,
            "panel_width": args.panel_width,
            "views": VIEW_CAMERAS,
            "rft_coefficient": args.rft_coeff,
            "force_smoothing_alpha": args.alpha,
            "body_order": BODIES,
            "sand_contact_definition": "active_triangles > 0",
            "rigid_contact_definition": "MuJoCo geom contact with floor",
        },
        "scenarios": {
            "rigid_original": bundle_summary(rigid),
            "sand_rft_shared_replay": bundle_summary(sand),
        },
        "videos": [
            path.relative_to(output_dir).as_posix() for path in video_paths
        ],
        "overview_video": overview_path.relative_to(output_dir).as_posix(),
        "overview_layout": overview_layout,
        "artifacts": artifacts,
    }
    manifest_path = output_dir / "matrix_manifest.json"
    with manifest_path.open("w", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=2)
    print(f"\nVideo matrix: {output_dir}")
    print(f"Manifest: {manifest_path}")


if __name__ == "__main__":
    main()
