"""Generate short diagnostic videos for historically rejected RFT meshes.

The videos are deliberately geometry-only.  They do not run the rejected
meshes through the RFT solver because a numerically stable animation would not
make invalid topology physically meaningful.  Each rejected/source-only mesh
is shown beside the accepted active mesh for the same body.  Every dot is one
triangle centroid, which is also one RFT force application site in the active
pipeline.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import textwrap
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path

import cv2
import imageio.v2 as imageio
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from audit_mesh_quality import audit_mesh


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_FPS = 24
DEFAULT_DURATION = 4.0


@dataclass(frozen=True)
class CaseDefinition:
    slug: str
    title: str
    body: str
    rejected_path: Path
    accepted_path: Path
    rejected_label: str
    diagnosis: str


@dataclass
class SiteCloud:
    centroids: np.ndarray
    spacing_m: np.ndarray
    colors_bgr: np.ndarray
    metrics: dict


def case_definitions() -> tuple[CaseDefinition, ...]:
    """Return the three bounded historical comparisons."""

    return (
        CaseDefinition(
            slug="01_raw_cad_assembly_vs_accepted",
            title="CASE A - RAW CAD ASSEMBLY USED AS AN RFT SHELL",
            body="Back",
            rejected_path=ROOT / "models" / "meshes" / "Back.STL",
            accepted_path=ROOT / "asset" / "Back.STL",
            rejected_label="REJECTED: RAW ASSEMBLY EXPORT",
            diagnosis=(
                "Internal servo, screw, housing, and exterior surfaces overlap. "
                "The 13 components create severe point clustering and large "
                "empty regions; this is not one contact shell."
            ),
        ),
        CaseDefinition(
            slug="02_legacy_vertex_cluster_vs_accepted",
            title="CASE B - LEGACY VERTEX-CLUSTERING REMESH",
            body="Back",
            rejected_path=(
                ROOT
                / "reference"
                / "rejected_meshes"
                / "legacy_vertex_cluster"
                / "Back.STL"
            ),
            accepted_path=ROOT / "asset" / "Back.STL",
            rejected_label="REJECTED: VERTEX-CLUSTERED",
            diagnosis=(
                "Reducing the triangle count did not recover the exterior "
                "union. The remesh kept 13 components, non-manifold topology, "
                "self-intersections, and uneven centroid spacing."
            ),
        ),
        CaseDefinition(
            slug="03_fixed_count_fusion_vs_accepted",
            title="CASE C - DIRECT FIXED-COUNT FUSION SOURCE",
            body="FR",
            rejected_path=(
                ROOT
                / "models"
                / "mesh_sources"
                / "fusion_external_envelope"
                / "FR.STL"
            ),
            accepted_path=ROOT / "asset" / "FR.STL",
            rejected_label="REJECTED AS ACTIVE: 1,500 FACES",
            diagnosis=(
                "The exterior envelope is a better source, but forcing the "
                "same 1,500-face budget onto every body leaves slivers, six "
                "self-intersections, and strongly unequal point density."
            ),
        ),
    )


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def git_state() -> dict:
    def run(*arguments: str) -> str:
        result = subprocess.run(
            ["git", *arguments],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        return result.stdout.strip()

    return {
        "commit": run("rev-parse", "HEAD"),
        "dirty": bool(run("status", "--porcelain")),
    }


def load_site_cloud(path: Path, label: str, body: str) -> SiteCloud:
    mesh = o3d.io.read_triangle_mesh(
        str(path),
        enable_post_processing=False,
    )
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    triangles = np.asarray(mesh.triangles, dtype=np.int64)
    if len(vertices) == 0 or len(triangles) == 0:
        raise ValueError(f"Empty mesh: {path}")

    centroids = vertices[triangles].mean(axis=1)
    spacing = cKDTree(centroids).query(
        centroids,
        k=2,
        workers=-1,
    )[0][:, 1]
    median = max(float(np.median(spacing)), np.finfo(float).tiny)
    normalized = np.clip((np.log2(spacing / median) + 2.0) / 4.0, 0.0, 1.0)
    color_indices = np.rint(normalized * 255.0).astype(np.uint8)
    colors = cv2.applyColorMap(color_indices[:, None], cv2.COLORMAP_TURBO)
    colors = colors[:, 0, :]

    metrics = audit_mesh(
        path,
        mesh_set=label,
        body=body,
        check_self=True,
    )
    metrics["path"] = path.resolve().relative_to(ROOT).as_posix()
    return SiteCloud(
        centroids=centroids,
        spacing_m=spacing,
        colors_bgr=colors,
        metrics=metrics,
    )


def projection_center_radius(
    first: np.ndarray,
    second: np.ndarray,
) -> tuple[np.ndarray, float]:
    combined_min = np.minimum(first.min(axis=0), second.min(axis=0))
    combined_max = np.maximum(first.max(axis=0), second.max(axis=0))
    center = 0.5 * (combined_min + combined_max)
    radius = float(
        max(
            np.linalg.norm(first - center, axis=1).max(),
            np.linalg.norm(second - center, axis=1).max(),
        )
    )
    if not np.isfinite(radius) or radius <= 0:
        raise ValueError("Mesh centroids have zero or invalid extent")
    return center, radius


def project_centroids(
    points: np.ndarray,
    *,
    center: np.ndarray,
    radius: float,
    azimuth_rad: float,
    elevation_rad: float,
    rectangle: tuple[int, int, int, int],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Orthographically project a rotating centroid cloud into a rectangle."""

    x0, y0, width, height = rectangle
    centered = points - center
    cosine = np.cos(azimuth_rad)
    sine = np.sin(azimuth_rad)

    horizontal = cosine * centered[:, 0] + sine * centered[:, 1]
    depth = -sine * centered[:, 0] + cosine * centered[:, 1]
    vertical = (
        np.cos(elevation_rad) * centered[:, 2]
        - np.sin(elevation_rad) * depth
    )
    camera_depth = (
        np.sin(elevation_rad) * centered[:, 2]
        + np.cos(elevation_rad) * depth
    )

    scale = 0.43 * min(width, height) / radius
    x_pixels = np.rint(x0 + width / 2.0 + horizontal * scale).astype(int)
    y_pixels = np.rint(y0 + height / 2.0 - vertical * scale).astype(int)
    return x_pixels, y_pixels, camera_depth


def draw_text(
    image: np.ndarray,
    text: str,
    origin: tuple[int, int],
    *,
    color: tuple[int, int, int] = (235, 240, 245),
    scale: float = 0.55,
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


def draw_site_cloud(
    image: np.ndarray,
    cloud: SiteCloud,
    *,
    rectangle: tuple[int, int, int, int],
    center: np.ndarray,
    radius: float,
    azimuth_rad: float,
    elevation_rad: float,
) -> None:
    x0, y0, width, height = rectangle
    cv2.rectangle(
        image,
        (x0, y0),
        (x0 + width, y0 + height),
        (38, 45, 55),
        1,
    )
    x_pixels, y_pixels, depth = project_centroids(
        cloud.centroids,
        center=center,
        radius=radius,
        azimuth_rad=azimuth_rad,
        elevation_rad=elevation_rad,
        rectangle=rectangle,
    )
    order = np.argsort(depth)
    x_pixels = x_pixels[order]
    y_pixels = y_pixels[order]
    colors = cloud.colors_bgr[order]

    for offset_x, offset_y in ((0, 0), (1, 0), (0, 1), (1, 1)):
        xs = x_pixels + offset_x
        ys = y_pixels + offset_y
        valid = (
            (xs >= x0 + 1)
            & (xs < x0 + width)
            & (ys >= y0 + 1)
            & (ys < y0 + height)
        )
        image[ys[valid], xs[valid]] = colors[valid]

    axis_origin = (x0 + 32, y0 + height - 28)
    cv2.arrowedLine(
        image,
        axis_origin,
        (axis_origin[0] + 34, axis_origin[1]),
        (180, 180, 180),
        1,
        tipLength=0.2,
    )
    cv2.arrowedLine(
        image,
        axis_origin,
        (axis_origin[0], axis_origin[1] - 34),
        (180, 180, 180),
        1,
        tipLength=0.2,
    )
    draw_text(image, "view H", (axis_origin[0] + 38, axis_origin[1] + 4), scale=0.34)
    draw_text(image, "view V", (axis_origin[0] - 20, axis_origin[1] - 40), scale=0.34)


def draw_spacing_legend(
    image: np.ndarray,
    *,
    origin: tuple[int, int],
    width: int,
) -> None:
    x0, y0 = origin
    gradient = np.arange(256, dtype=np.uint8)[None, :]
    colorbar = cv2.applyColorMap(gradient, cv2.COLORMAP_TURBO)
    colorbar = cv2.resize(colorbar, (width, 12), interpolation=cv2.INTER_LINEAR)
    image[y0 : y0 + 12, x0 : x0 + width] = colorbar
    draw_text(image, "dense", (x0, y0 + 28), scale=0.34)
    draw_text(
        image,
        "local centroid spacing / median",
        (x0 + max(45, width // 2 - 90), y0 + 28),
        scale=0.34,
    )
    draw_text(image, "sparse", (x0 + width - 42, y0 + 28), scale=0.34)


def metric_lines(metrics: dict) -> tuple[str, str, str]:
    topology = (
        f"points {metrics['triangles']:,}  |  components "
        f"{metrics['connected_components']}  |  watertight "
        f"{'YES' if metrics['watertight'] else 'NO'}"
    )
    defects = (
        f"self-X pairs {metrics['self_intersecting_triangle_pairs']:,}  |  "
        f"non-manifold edges {metrics['nonmanifold_edges']:,}"
    )
    distribution = (
        f"area P95/P05 {metrics['area_p95_p05']:.1f}  |  spacing "
        f"P95/P05 {metrics['centroid_nn_p95_p05']:.1f}  |  slivers "
        f"{100.0 * metrics['sliver_fraction_lt_10deg']:.1f}%"
    )
    return topology, defects, distribution


def draw_metric_card(
    image: np.ndarray,
    *,
    rectangle: tuple[int, int, int, int],
    label: str,
    metrics: dict,
    accent: tuple[int, int, int],
) -> None:
    x0, y0, width, height = rectangle
    cv2.rectangle(
        image,
        (x0, y0),
        (x0 + width, y0 + height),
        (24, 29, 37),
        -1,
    )
    cv2.rectangle(
        image,
        (x0, y0),
        (x0 + width, y0 + height),
        accent,
        2,
    )
    draw_text(
        image,
        label,
        (x0 + 12, y0 + 24),
        color=accent,
        scale=0.5,
        thickness=1,
    )
    for index, line in enumerate(metric_lines(metrics)):
        draw_text(
            image,
            line,
            (x0 + 12, y0 + 49 + 22 * index),
            scale=0.42,
        )


def render_case_frame(
    case: CaseDefinition,
    rejected: SiteCloud,
    accepted: SiteCloud,
    *,
    frame_index: int,
    frame_count: int,
    width: int,
    height: int,
) -> np.ndarray:
    if width < 960 or height < 540:
        raise ValueError("Diagnostic frame must be at least 960x540")

    image = np.full((height, width, 3), (13, 17, 23), dtype=np.uint8)
    progress = frame_index / max(frame_count - 1, 1)
    azimuth = np.deg2rad(20.0 + 360.0 * progress)
    elevation = np.deg2rad(23.0)
    center, radius = projection_center_radius(
        rejected.centroids,
        accepted.centroids,
    )

    draw_text(
        image,
        case.title,
        (28, 34),
        color=(245, 245, 245),
        scale=0.75,
        thickness=2,
    )
    draw_text(
        image,
        (
            f"{case.body} body - every dot is one triangle centroid / "
            "RFT force site"
        ),
        (28, 62),
        color=(175, 185, 200),
        scale=0.5,
    )

    margin = 28
    gap = 24
    panel_width = (width - 2 * margin - gap) // 2
    panel_top = 82
    card_height = 110
    footer_height = 48
    card_top = height - card_height - footer_height - 12
    panel_height = card_top - panel_top - 12
    left_panel = (margin, panel_top, panel_width, panel_height)
    right_panel = (margin + panel_width + gap, panel_top, panel_width, panel_height)

    draw_site_cloud(
        image,
        rejected,
        rectangle=left_panel,
        center=center,
        radius=radius,
        azimuth_rad=azimuth,
        elevation_rad=elevation,
    )
    draw_site_cloud(
        image,
        accepted,
        rectangle=right_panel,
        center=center,
        radius=radius,
        azimuth_rad=azimuth,
        elevation_rad=elevation,
    )
    draw_text(
        image,
        case.rejected_label,
        (left_panel[0] + 12, left_panel[1] + 24),
        color=(70, 90, 255),
        scale=0.5,
        thickness=2,
    )
    draw_text(
        image,
        "ACCEPTED: GATED ACTIVE SURFACE",
        (right_panel[0] + 12, right_panel[1] + 24),
        color=(105, 225, 125),
        scale=0.5,
        thickness=2,
    )
    draw_spacing_legend(
        image,
        origin=(left_panel[0] + left_panel[2] - 245, left_panel[1] + 36),
        width=220,
    )
    draw_spacing_legend(
        image,
        origin=(right_panel[0] + right_panel[2] - 245, right_panel[1] + 36),
        width=220,
    )

    draw_metric_card(
        image,
        rectangle=(left_panel[0], card_top, panel_width, card_height),
        label="REJECTED / SOURCE-ONLY METRICS",
        metrics=rejected.metrics,
        accent=(70, 90, 255),
    )
    draw_metric_card(
        image,
        rectangle=(right_panel[0], card_top, panel_width, card_height),
        label="CURRENT ACCEPTED METRICS",
        metrics=accepted.metrics,
        accent=(105, 225, 125),
    )

    footer_y = height - footer_height + 3
    wrapped = textwrap.wrap(
        f"DIAGNOSTIC ONLY - {case.diagnosis}",
        width=145,
    )
    for index, line in enumerate(wrapped[:2]):
        draw_text(
            image,
            line,
            (28, footer_y + 19 * index),
            color=(120, 200, 255),
            scale=0.43,
            thickness=1,
        )
    return image


def write_case_video(
    case: CaseDefinition,
    rejected: SiteCloud,
    accepted: SiteCloud,
    *,
    output_path: Path,
    fps: int,
    duration: float,
    width: int,
    height: int,
) -> tuple[int, np.ndarray]:
    frame_count = int(round(duration * fps)) + 1
    writer = imageio.get_writer(
        str(output_path),
        fps=fps,
        codec="libx264",
        quality=8,
        macro_block_size=2,
        ffmpeg_log_level="error",
        output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"],
    )
    midpoint = None
    try:
        for frame_index in range(frame_count):
            frame_bgr = render_case_frame(
                case,
                rejected,
                accepted,
                frame_index=frame_index,
                frame_count=frame_count,
                width=width,
                height=height,
            )
            if frame_index == frame_count // 2:
                midpoint = frame_bgr.copy()
            writer.append_data(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
    finally:
        writer.close()
    if midpoint is None:
        raise RuntimeError("No midpoint frame was produced")
    return frame_count, midpoint


def make_contact_sheet(
    frames: list[tuple[CaseDefinition, np.ndarray]],
    output_path: Path,
) -> None:
    width, height = 1280, 720
    canvas = np.full((height, width, 3), (11, 15, 20), dtype=np.uint8)
    cell_width, cell_height = 640, 360
    for index, (case, frame) in enumerate(frames):
        row, column = divmod(index, 2)
        resized = cv2.resize(
            frame,
            (cell_width, cell_height),
            interpolation=cv2.INTER_AREA,
        )
        y0, x0 = row * cell_height, column * cell_width
        canvas[y0 : y0 + cell_height, x0 : x0 + cell_width] = resized

    x0, y0 = cell_width, cell_height
    cv2.rectangle(
        canvas,
        (x0, y0),
        (width - 1, height - 1),
        (24, 29, 37),
        -1,
    )
    draw_text(
        canvas,
        "WHY THESE MODELS WERE REJECTED",
        (x0 + 32, y0 + 52),
        color=(90, 120, 255),
        scale=0.72,
        thickness=2,
    )
    summary = (
        "A  Raw CAD: overlapping internal assembly surfaces",
        "B  Legacy remesh: topology defects survived decimation",
        "C  Fixed count: uneven density and residual intersections",
        "",
        "Dots are triangle centroids / RFT force sites.",
        "Red side = rejected or source-only.",
        "Green side = current topology-gated active mesh.",
        "",
        "These are geometry diagnostics, not locomotion results.",
    )
    for index, line in enumerate(summary):
        draw_text(
            canvas,
            line,
            (x0 + 34, y0 + 94 + 28 * index),
            color=(215, 222, 232),
            scale=0.48,
        )
    cv2.imwrite(str(output_path), canvas)


def make_preview_gif(
    video_paths: list[Path],
    output_path: Path,
    *,
    source_fps: int,
) -> None:
    preview_fps = 6
    sample_rate = 3
    writer = imageio.get_writer(
        str(output_path),
        mode="I",
        fps=preview_fps,
        loop=0,
    )
    stride = max(source_fps // sample_rate, 1)
    try:
        for path in video_paths:
            reader = imageio.get_reader(str(path))
            try:
                for frame_index, frame_rgb in enumerate(reader):
                    if frame_index % stride:
                        continue
                    resized = cv2.resize(
                        frame_rgb,
                        (640, 360),
                        interpolation=cv2.INTER_AREA,
                    )
                    writer.append_data(resized)
            finally:
                reader.close()
    finally:
        writer.close()


def relative_to_root(path: Path) -> str:
    return path.resolve().relative_to(ROOT).as_posix()


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Render three historically rejected/source-only RFT mesh "
            "comparisons without running invalid physics."
        )
    )
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION)
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS)
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="default: outputs/failed_mesh_videos/run_<timestamp>",
    )
    args = parser.parse_args()
    if args.duration <= 0 or args.fps <= 0:
        raise ValueError("duration and fps must be positive")
    if args.width % 2 or args.height % 2:
        raise ValueError("width and height must be even for H.264 output")

    output_dir = args.output_dir
    if output_dir is None:
        output_dir = (
            ROOT
            / "outputs"
            / "failed_mesh_videos"
            / datetime.now().strftime("run_%Y-%m-%d_%H%M%S_%f")
        )
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=False)

    source_git = git_state()
    generated_at = datetime.now(timezone.utc).isoformat()
    cases_manifest = []
    video_paths: list[Path] = []
    midpoint_frames: list[tuple[CaseDefinition, np.ndarray]] = []

    for case in case_definitions():
        for path in (case.rejected_path, case.accepted_path):
            if not path.is_file():
                raise FileNotFoundError(path)
        print(f"Loading/auditing {case.slug}...", flush=True)
        rejected = load_site_cloud(
            case.rejected_path,
            f"{case.slug}_rejected",
            case.body,
        )
        accepted = load_site_cloud(
            case.accepted_path,
            f"{case.slug}_accepted",
            case.body,
        )
        video_path = output_dir / f"{case.slug}.mp4"
        print(f"Rendering {video_path.name}...", flush=True)
        frame_count, midpoint = write_case_video(
            case,
            rejected,
            accepted,
            output_path=video_path,
            fps=args.fps,
            duration=args.duration,
            width=args.width,
            height=args.height,
        )
        video_paths.append(video_path)
        midpoint_frames.append((case, midpoint))
        cases_manifest.append(
            {
                "definition": {
                    **asdict(case),
                    "rejected_path": relative_to_root(case.rejected_path),
                    "accepted_path": relative_to_root(case.accepted_path),
                },
                "rejected_metrics": rejected.metrics,
                "accepted_metrics": accepted.metrics,
                "video": video_path.name,
                "frames": frame_count,
            }
        )

    still_path = output_dir / "failed_mesh_diagnostics_contact_sheet.png"
    preview_path = output_dir / "failed_mesh_diagnostics_preview.gif"
    make_contact_sheet(midpoint_frames, still_path)
    make_preview_gif(video_paths, preview_path, source_fps=args.fps)

    artifacts = []
    for path in (*video_paths, still_path, preview_path):
        artifacts.append(
            {
                "path": path.name,
                "bytes": path.stat().st_size,
                "sha256": sha256_file(path),
            }
        )
    manifest = {
        "schema_version": 1,
        "generated_at_utc": generated_at,
        "purpose": (
            "Geometry-only diagnostics for rejected/source-only RFT meshes. "
            "No rejected mesh was used for locomotion or force calculation."
        ),
        "git": source_git,
        "video": {
            "fps": args.fps,
            "requested_duration_s": args.duration,
            "frames": int(round(args.duration * args.fps)) + 1,
            "width": args.width,
            "height": args.height,
            "codec": "H.264 / yuv420p",
        },
        "color_encoding": (
            "Each point is a triangle centroid. Color is log2(local nearest "
            "centroid spacing / median), clipped to [1/4, 4]; blue is dense "
            "and red is sparse."
        ),
        "cases": cases_manifest,
        "artifacts": artifacts,
    }
    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, allow_nan=False),
        encoding="utf-8",
    )
    print(f"Wrote {output_dir}")


if __name__ == "__main__":
    main()
