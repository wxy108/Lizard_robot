"""Verify and summarize one generated locomotion video-matrix run."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def contact_intervals(
    time_s: np.ndarray,
    values: np.ndarray,
) -> list[tuple[float, float]]:
    binary = np.asarray(values, dtype=bool)
    dt = (
        float(np.median(np.diff(time_s))) if len(time_s) > 1 else 0.0
    )
    padded = np.concatenate(([False], binary, [False])).astype(np.int8)
    changes = np.diff(padded)
    starts = np.flatnonzero(changes == 1)
    ends = np.flatnonzero(changes == -1)
    intervals = []
    for start, end in zip(starts, ends, strict=True):
        end_time = (
            float(time_s[end])
            if end < len(time_s)
            else float(time_s[-1] + dt)
        )
        intervals.append((float(time_s[start]), end_time))
    return intervals


def component_metrics(
    time_s: np.ndarray,
    contact: np.ndarray,
    body_order: list[str],
    *,
    max_penetration_m: np.ndarray | None = None,
    active_triangles: np.ndarray | None = None,
) -> list[dict]:
    rows = []
    for body_index, body in enumerate(body_order):
        intervals = contact_intervals(time_s, contact[:, body_index])
        durations = [end - start for start, end in intervals]
        active_mask = contact[:, body_index] > 0
        row = {
            "component": body,
            "contact_duty": float(np.mean(active_mask)),
            "contact_events": len(intervals),
            "first_contact_s": intervals[0][0] if intervals else None,
            "last_release_s": intervals[-1][1] if intervals else None,
            "mean_event_duration_s": (
                float(np.mean(durations)) if durations else 0.0
            ),
            "max_event_duration_s": max(durations, default=0.0),
        }
        if max_penetration_m is not None:
            row["maximum_penetration_mm"] = float(
                1000.0 * np.max(max_penetration_m[:, body_index])
            )
        if active_triangles is not None:
            row["mean_active_triangles_when_contacting"] = (
                float(np.mean(active_triangles[active_mask, body_index]))
                if np.any(active_mask)
                else 0.0
            )
            row["maximum_active_triangles"] = int(
                np.max(active_triangles[:, body_index])
            )
        rows.append(row)
    return rows


def trajectory_metrics(time_s: np.ndarray, com: np.ndarray) -> dict:
    increments = np.diff(com, axis=0)
    displacement = com[-1] - com[0]
    return {
        "duration_s": float(time_s[-1]),
        "com_displacement_m": [float(value) for value in displacement],
        "com_path_length_m": float(
            np.sum(np.linalg.norm(increments, axis=1))
        ),
        "mean_com_speed_m_s": float(
            np.sum(np.linalg.norm(increments, axis=1))
            / max(float(time_s[-1] - time_s[0]), 1e-12)
        ),
    }


def verify_manifest(run_dir: Path, manifest: dict) -> None:
    failures = []
    for relative, expected in manifest["artifacts"].items():
        path = run_dir / relative
        if not path.is_file():
            failures.append(f"missing: {relative}")
            continue
        actual_hash = sha256(path)
        if actual_hash != expected["sha256"]:
            failures.append(f"hash mismatch: {relative}")
        if path.stat().st_size != expected["bytes"]:
            failures.append(f"size mismatch: {relative}")
    if failures:
        raise ValueError("Artifact verification failed:\n" + "\n".join(failures))


def load_npz(path: Path) -> dict[str, np.ndarray]:
    with np.load(path) as archive:
        return {key: archive[key] for key in archive.files}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument(
        "--skip-hash-verification",
        action="store_true",
        help="analyze even if manifest artifacts are not being verified",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run_dir = args.run_dir.resolve()
    manifest_path = run_dir / "matrix_manifest.json"
    with manifest_path.open(encoding="utf-8") as handle:
        manifest = json.load(handle)
    if not args.skip_hash_verification:
        verify_manifest(run_dir, manifest)

    scenario_files = {
        "rigid_original": run_dir / "analysis" / "rigid_original.npz",
        "sand_rft": run_dir / "analysis" / "sand_rft.npz",
    }
    summaries = {}
    all_rows = []
    for scenario, path in scenario_files.items():
        data = load_npz(path)
        body_order = [str(value) for value in data["body_order"]]
        rows = component_metrics(
            data["time"],
            data["contact"],
            body_order,
            max_penetration_m=data.get("max_penetration_m"),
            active_triangles=data.get("active_triangles"),
        )
        for row in rows:
            row["scenario"] = scenario
            all_rows.append(row)
        summaries[scenario] = {
            "trajectory": trajectory_metrics(data["time"], data["com"]),
            "components": rows,
        }

    output_json = run_dir / "analysis" / "derived_metrics.json"
    with output_json.open("w", encoding="utf-8") as handle:
        json.dump(summaries, handle, indent=2)
    output_csv = run_dir / "analysis" / "component_metrics.csv"
    fieldnames = []
    for row in all_rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(all_rows)

    for scenario, summary in summaries.items():
        displacement = summary["trajectory"]["com_displacement_m"]
        print(
            f"{scenario}: dCOM=({displacement[0]:+.4f}, "
            f"{displacement[1]:+.4f}, {displacement[2]:+.4f}) m, "
            f"path={summary['trajectory']['com_path_length_m']:.4f} m"
        )
        for row in summary["components"]:
            print(
                f"  {row['component']:<5} duty={row['contact_duty']:.3f} "
                f"events={row['contact_events']}"
            )
    print(f"Derived JSON: {output_json}")
    print(f"Component CSV: {output_csv}")


if __name__ == "__main__":
    main()
