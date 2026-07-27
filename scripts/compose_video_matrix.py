"""Compose the nine locomotion views and three row dashboards into one MP4."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Mapping

import cv2
import imageio.v2 as imageio
import numpy as np


SCENARIOS = (
    ("rigid_original", "Original CAD | rigid ground"),
    ("sand_simplified", "Simplified RFT sand | sites hidden"),
    ("sand_simplified_sites", "Simplified RFT sand | sites visible"),
)
VIEWS = ("top", "side", "diag45")
COLUMN_LABELS = ("TOP", "SIDE", "45 DEG", "ANALYSIS")


def _even(value: float) -> int:
    return max(2, int(round(value)) // 2 * 2)


def _label(
    image: np.ndarray,
    text: str,
    origin: tuple[int, int],
    *,
    scale: float,
    color: tuple[int, int, int] = (245, 245, 245),
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


def compose_master_frame(
    frames: Mapping[tuple[str, str], np.ndarray],
    *,
    render_width: int,
    render_height: int,
    panel_width: int,
    row_height: int | None = None,
    header_height: int = 48,
) -> np.ndarray:
    """Build one frame: three scenario rows x three views + one dashboard."""
    if row_height is None:
        row_height = _even(render_height / 2)
    row_height = _even(row_height)
    header_height = _even(header_height)
    view_width = _even(render_width * row_height / render_height)
    dashboard_width = _even(panel_width * row_height / render_height)
    output_width = 3 * view_width + dashboard_width
    output_height = header_height + len(SCENARIOS) * row_height
    canvas = np.full(
        (output_height, output_width, 3),
        18,
        dtype=np.uint8,
    )

    column_lefts = (
        0,
        view_width,
        2 * view_width,
        3 * view_width,
    )
    column_widths = (
        view_width,
        view_width,
        view_width,
        dashboard_width,
    )
    for text, left, width in zip(
        COLUMN_LABELS,
        column_lefts,
        column_widths,
        strict=True,
    ):
        size = cv2.getTextSize(
            text,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            1,
        )[0]
        _label(
            canvas,
            text,
            (left + max(8, (width - size[0]) // 2), header_height - 15),
            scale=0.58,
        )

    expected_width = render_width + panel_width
    for row_index, (scenario, scenario_label) in enumerate(SCENARIOS):
        top = header_height + row_index * row_height
        bottom = top + row_height
        for column_index, view in enumerate(VIEWS):
            frame = np.asarray(frames[(scenario, view)])
            if (
                frame.ndim != 3
                or frame.shape[0] != render_height
                or frame.shape[1] < expected_width
            ):
                raise ValueError(
                    f"unexpected {scenario}/{view} frame shape: "
                    f"{frame.shape}"
                )
            view_frame = cv2.resize(
                frame[:, :render_width],
                (view_width, row_height),
                interpolation=cv2.INTER_AREA,
            )
            left = column_index * view_width
            canvas[top:bottom, left : left + view_width] = view_frame

        side_frame = np.asarray(frames[(scenario, "side")])
        dashboard = cv2.resize(
            side_frame[:, render_width : render_width + panel_width],
            (dashboard_width, row_height),
            interpolation=cv2.INTER_AREA,
        )
        canvas[
            top:bottom,
            3 * view_width : 3 * view_width + dashboard_width,
        ] = dashboard

        overlay = canvas[top : top + 32, :view_width]
        overlay[:] = (
            0.35 * overlay.astype(np.float32)
        ).astype(np.uint8)
        _label(
            canvas,
            scenario_label,
            (10, top + 22),
            scale=0.47,
            thickness=1,
        )

        cv2.line(
            canvas,
            (0, top),
            (output_width - 1, top),
            (90, 90, 90),
            1,
        )
    for boundary in (
        view_width,
        2 * view_width,
        3 * view_width,
    ):
        cv2.line(
            canvas,
            (boundary, 0),
            (boundary, output_height - 1),
            (90, 90, 90),
            1,
        )
    return canvas


def compose_master_video(
    *,
    video_dir: Path,
    output_path: Path,
    render_width: int,
    render_height: int,
    panel_width: int,
    fps: int,
) -> dict:
    """Read the nine synchronized MP4 files and write one overview MP4."""
    readers = {}
    for scenario, _ in SCENARIOS:
        for view in VIEWS:
            path = video_dir / scenario / f"{view}.mp4"
            if not path.is_file():
                raise FileNotFoundError(path)
            readers[(scenario, view)] = imageio.get_reader(str(path))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = imageio.get_writer(
        str(output_path),
        fps=fps,
        codec="libx264",
        quality=8,
        macro_block_size=2,
    )
    frame_count = 0
    output_shape = None
    try:
        while True:
            frames = {}
            ended = []
            for key, reader in readers.items():
                try:
                    frames[key] = reader.get_next_data()
                    ended.append(False)
                except (IndexError, StopIteration):
                    ended.append(True)
            if all(ended):
                break
            if any(ended):
                raise ValueError("the nine source videos have unequal lengths")
            overview = compose_master_frame(
                frames,
                render_width=render_width,
                render_height=render_height,
                panel_width=panel_width,
            )
            writer.append_data(overview)
            output_shape = overview.shape
            frame_count += 1
    finally:
        writer.close()
        for reader in readers.values():
            reader.close()

    if frame_count == 0 or output_shape is None:
        raise ValueError("source videos contain no frames")
    return {
        "frames": frame_count,
        "fps": fps,
        "duration_s": frame_count / fps,
        "width": int(output_shape[1]),
        "height": int(output_shape[0]),
        "layout": "3 scenario rows x (top | side | 45-degree | analysis)",
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run_dir = args.run_dir.resolve()
    manifest_path = run_dir / "matrix_manifest.json"
    with manifest_path.open(encoding="utf-8") as handle:
        manifest = json.load(handle)
    configuration = manifest["configuration"]
    output_path = (
        args.output.resolve()
        if args.output is not None
        else run_dir / "videos" / "video_matrix_overview.mp4"
    )
    if output_path.exists():
        raise FileExistsError(output_path)
    result = compose_master_video(
        video_dir=run_dir / "videos",
        output_path=output_path,
        render_width=int(configuration["render_width"]),
        render_height=int(configuration["render_height"]),
        panel_width=int(configuration["panel_width"]),
        fps=int(configuration["fps"]),
    )
    relative = output_path.relative_to(run_dir).as_posix()
    manifest["overview_video"] = relative
    manifest["overview_layout"] = result
    manifest.setdefault("artifacts", {})[relative] = {
        "bytes": output_path.stat().st_size,
        "sha256": _sha256(output_path),
    }
    with manifest_path.open("w", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=2)
    print(f"Overview video: {output_path}")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
