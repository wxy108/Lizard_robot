"""Recover and present the historical invalid-RFT locomotion recording.

The archived ``legacy_sand.mp4`` was interrupted before its MP4 ``moov`` atom
was written. Its ``mdat`` payload still contains complete, length-prefixed
H.264 NAL units. This tool restores the matching SPS/PPS headers preserved in
``reference/rejected_media/``, decodes only the complete historical frames,
and writes:

1. a lossless valid-container recovery;
2. an explicitly labelled, dynamically zoomed diagnostic copy;
3. a six-frame contact sheet;
4. a provenance and SHA-256 manifest.

No simulation is run. The output is historical failure evidence, not a valid
RFT or locomotion baseline.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import struct
import subprocess
import sys
import tempfile
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import cv2
import imageio_ffmpeg
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = (
    ROOT / "reference" / "rejected_media" / "legacy_sand_truncated.mp4"
)
DEFAULT_HEADERS = (
    ROOT / "reference" / "rejected_media" / "legacy_sand_h264_sps_pps.bin"
)
RECOVERED_NAME = "legacy_incorrect_rft_locomotion_recovered.mp4"
ZOOMED_NAME = "legacy_incorrect_rft_locomotion_zoomed.mp4"
CONTACT_SHEET_NAME = "legacy_incorrect_rft_locomotion_contact_sheet.png"
MANIFEST_NAME = "manifest.json"
START_CODE = b"\x00\x00\x00\x01"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def repository_path(path: Path) -> str:
    try:
        return path.resolve().relative_to(ROOT.resolve()).as_posix()
    except ValueError:
        return str(path.resolve())


def split_annex_b(data: bytes) -> list[bytes]:
    """Return NAL payloads from a three- or four-byte start-code stream."""

    starts: list[tuple[int, int]] = []
    index = 0
    while index < len(data) - 2:
        if data[index : index + 4] == START_CODE:
            starts.append((index, 4))
            index += 4
        elif data[index : index + 3] == b"\x00\x00\x01":
            starts.append((index, 3))
            index += 3
        else:
            index += 1

    units: list[bytes] = []
    for unit_index, (start, prefix_length) in enumerate(starts):
        end = starts[unit_index + 1][0] if unit_index + 1 < len(starts) else len(data)
        unit = data[start + prefix_length : end]
        if unit:
            units.append(unit)
    return units


def find_mdat_payload(data: bytes) -> tuple[bytes, dict[str, int | bool]]:
    """Locate the first MP4 ``mdat`` atom, including a truncated declared atom."""

    offset = 0
    while offset + 8 <= len(data):
        declared_size = struct.unpack(">I", data[offset : offset + 4])[0]
        atom_type = data[offset + 4 : offset + 8]
        header_size = 8
        if declared_size == 1:
            if offset + 16 > len(data):
                raise ValueError("Truncated extended MP4 atom header")
            declared_size = struct.unpack(">Q", data[offset + 8 : offset + 16])[0]
            header_size = 16

        if atom_type == b"mdat":
            payload_start = offset + header_size
            if payload_start > len(data):
                raise ValueError("mdat payload begins beyond end of file")
            declared_payload = max(int(declared_size) - header_size, 0)
            available_payload = len(data) - payload_start
            return data[payload_start:], {
                "atom_offset": offset,
                "declared_atom_bytes": int(declared_size),
                "declared_payload_bytes": declared_payload,
                "available_payload_bytes": available_payload,
                "atom_truncated": declared_size > len(data) - offset,
            }

        if declared_size == 0:
            break
        if declared_size < header_size:
            raise ValueError(f"Invalid MP4 atom size {declared_size} at byte {offset}")
        offset += int(declared_size)

    raise ValueError("No mdat atom found")


def extract_complete_nals(payload: bytes) -> tuple[list[bytes], int]:
    """Extract complete four-byte-length-prefixed H.264 NAL units."""

    units: list[bytes] = []
    offset = 0
    while offset + 4 <= len(payload):
        length = struct.unpack(">I", payload[offset : offset + 4])[0]
        if length <= 0 or offset + 4 + length > len(payload):
            break
        units.append(payload[offset + 4 : offset + 4 + length])
        offset += 4 + length
    return units, len(payload) - offset


def build_recovery_stream(source: bytes, header_stream: bytes) -> tuple[bytes, dict]:
    headers = split_annex_b(header_stream)
    header_types = [unit[0] & 31 for unit in headers]
    if header_types != [7, 8]:
        raise ValueError(
            "Header file must contain exactly one SPS (type 7) followed by "
            "one PPS (type 8)"
        )

    payload, atom = find_mdat_payload(source)
    units, trailing_bytes = extract_complete_nals(payload)
    if not units:
        raise ValueError("No complete H.264 NAL units found in mdat")
    nal_types = Counter(unit[0] & 31 for unit in units)
    if nal_types[5] < 1:
        raise ValueError("Recovered stream has no IDR frame")

    annex_b = b"".join(START_CODE + unit for unit in headers + units)
    return annex_b, {
        **atom,
        "header_nal_types": header_types,
        "complete_nal_units": len(units),
        "nal_type_counts": {str(key): value for key, value in sorted(nal_types.items())},
        "discarded_trailing_bytes": trailing_bytes,
    }


def run_ffmpeg(command: list[str], *, stdin_frames=None) -> None:
    if stdin_frames is None:
        subprocess.run(command, check=True)
        return

    process = subprocess.Popen(command, stdin=subprocess.PIPE)
    if process.stdin is None:
        raise RuntimeError("Could not open FFmpeg stdin")
    try:
        for frame in stdin_frames:
            process.stdin.write(np.ascontiguousarray(frame).tobytes())
    finally:
        process.stdin.close()
    return_code = process.wait()
    if return_code:
        raise subprocess.CalledProcessError(return_code, command)


def probe_video(path: Path) -> dict[str, int | float | str]:
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError(f"Could not open generated video: {path}")
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    decoded = 0
    while True:
        ok, _ = capture.read()
        if not ok:
            break
        decoded += 1
    capture.release()
    if decoded != frame_count:
        raise RuntimeError(
            f"Video metadata reports {frame_count} frames but {decoded} decoded"
        )
    return {
        "path": repository_path(path),
        "bytes": path.stat().st_size,
        "sha256": sha256_file(path),
        "frame_count": frame_count,
        "decoded_frame_count": decoded,
        "fps": fps,
        "duration_seconds": frame_count / fps if fps else 0.0,
        "width": width,
        "height": height,
    }


def detect_robot_box(frame: np.ndarray) -> tuple[int, int, int, int]:
    """Detect the robot against the original recording's uniform sand color."""

    height, width = frame.shape[:2]
    border = np.concatenate(
        [
            frame[:16].reshape(-1, 3),
            frame[-16:].reshape(-1, 3),
            frame[:, :16].reshape(-1, 3),
            frame[:, -16:].reshape(-1, 3),
        ]
    )
    background = np.median(border, axis=0)
    distance = np.linalg.norm(frame.astype(np.float32) - background, axis=2)
    mask = (distance > 35).astype(np.uint8)
    mask[:12] = 0
    mask[-12:] = 0
    mask[:, :12] = 0
    mask[:, -12:] = 0
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)),
    )
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if count <= 1:
        return 0, 0, width, height
    x, y, box_width, box_height, _ = max(stats[1:], key=lambda row: row[4])
    return int(x), int(y), int(box_width), int(box_height)


def moving_average(values: np.ndarray, window: int = 9) -> np.ndarray:
    if len(values) <= 1:
        return values.copy()
    window = min(window, len(values))
    if window % 2 == 0:
        window -= 1
    if window <= 1:
        return values.copy()
    radius = window // 2
    padded = np.pad(values, (radius, radius), mode="edge")
    return np.convolve(padded, np.ones(window) / window, mode="valid")


def zoom_plan(path: Path) -> tuple[np.ndarray, np.ndarray, int, int, tuple[int, int]]:
    capture = cv2.VideoCapture(str(path))
    boxes = []
    source_size = None
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        source_size = (frame.shape[1], frame.shape[0])
        boxes.append(detect_robot_box(frame))
    capture.release()
    if not boxes or source_size is None:
        raise RuntimeError("No frames available for zoom planning")

    boxes_array = np.asarray(boxes, dtype=float)
    centers_x = moving_average(boxes_array[:, 0] + boxes_array[:, 2] / 2)
    centers_y = moving_average(boxes_array[:, 1] + boxes_array[:, 3] / 2)
    crop_width = int(np.ceil(np.percentile(boxes_array[:, 2], 98) * 1.28 / 16) * 16)
    crop_width = int(np.clip(crop_width, 640, source_size[0]))
    crop_height = int(round(crop_width * 9 / 16))
    required_height = int(np.ceil(np.percentile(boxes_array[:, 3], 98) * 1.65))
    if crop_height < required_height:
        crop_height = int(np.ceil(required_height / 9) * 9)
        crop_width = int(round(crop_height * 16 / 9))
    crop_width = min(crop_width, source_size[0])
    crop_height = min(crop_height, source_size[1])
    return centers_x, centers_y, crop_width, crop_height, source_size


def crop_with_padding(
    frame: np.ndarray,
    center_x: float,
    center_y: float,
    crop_width: int,
    crop_height: int,
) -> np.ndarray:
    height, width = frame.shape[:2]
    x0 = int(round(center_x - crop_width / 2))
    y0 = int(round(center_y - crop_height / 2))
    x0 = min(max(x0, 0), max(width - crop_width, 0))
    y0 = min(max(y0, 0), max(height - crop_height, 0))
    return frame[y0 : y0 + crop_height, x0 : x0 + crop_width]


def zoom_frames(
    source: Path,
    centers_x: np.ndarray,
    centers_y: np.ndarray,
    crop_width: int,
    crop_height: int,
    fps: float,
):
    capture = cv2.VideoCapture(str(source))
    frame_index = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        crop = crop_with_padding(
            frame,
            centers_x[frame_index],
            centers_y[frame_index],
            crop_width,
            crop_height,
        )
        output = cv2.resize(crop, (1280, 720), interpolation=cv2.INTER_LANCZOS4)

        overlay = output.copy()
        cv2.rectangle(overlay, (0, 0), (1280, 72), (22, 37, 156), -1)
        output = cv2.addWeighted(overlay, 0.82, output, 0.18, 0)
        cv2.putText(
            output,
            "HISTORICAL INVALID RFT MESH - UNEVEN FORCE-SITE DISTRIBUTION",
            (24, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            output,
            f"Recovered original frames; no resimulation; not a physics baseline  "
            f"t={frame_index / fps:.2f}s",
            (24, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.54,
            (232, 238, 255),
            1,
            cv2.LINE_AA,
        )
        yield output
        frame_index += 1
    capture.release()


def make_contact_sheet(video: Path, destination: Path) -> None:
    capture = cv2.VideoCapture(str(video))
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    selected = np.linspace(0, max(frame_count - 1, 0), 6, dtype=int)
    frames = []
    for index in selected:
        capture.set(cv2.CAP_PROP_POS_FRAMES, int(index))
        ok, frame = capture.read()
        if not ok:
            raise RuntimeError(f"Could not decode contact-sheet frame {index}")
        frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)
        cv2.putText(
            frame,
            f"frame {index}/{frame_count - 1}",
            (18, 342),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        frames.append(frame)
    capture.release()
    sheet = np.vstack([np.hstack(frames[:3]), np.hstack(frames[3:])])
    if not cv2.imwrite(str(destination), sheet, [cv2.IMWRITE_PNG_COMPRESSION, 6]):
        raise RuntimeError(f"Could not write contact sheet: {destination}")


def git_state() -> dict[str, str | bool | None]:
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        status = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        return {"commit": commit, "dirty": bool(status.strip())}
    except (OSError, subprocess.CalledProcessError):
        return {"commit": None, "dirty": None}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--headers", type=Path, default=DEFAULT_HEADERS)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="new destination directory (default: timestamped ignored output)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.fps <= 0:
        raise ValueError("--fps must be positive")
    source = args.source.resolve()
    headers = args.headers.resolve()
    if not source.is_file():
        raise FileNotFoundError(source)
    if not headers.is_file():
        raise FileNotFoundError(headers)

    output_dir = args.output_dir
    if output_dir is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = ROOT / "outputs" / "legacy_rft_locomotion" / f"run_{stamp}"
    output_dir = output_dir.resolve()
    if output_dir.exists():
        raise FileExistsError(f"Output directory already exists: {output_dir}")
    output_dir.mkdir(parents=True)

    recovered = output_dir / RECOVERED_NAME
    zoomed = output_dir / ZOOMED_NAME
    contact_sheet = output_dir / CONTACT_SHEET_NAME
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()

    stream, recovery = build_recovery_stream(source.read_bytes(), headers.read_bytes())
    with tempfile.TemporaryDirectory(prefix="lizard_legacy_rft_") as temp_dir:
        raw_stream = Path(temp_dir) / "recovered.h264"
        raw_stream.write_bytes(stream)
        run_ffmpeg(
            [
                ffmpeg,
                "-y",
                "-loglevel",
                "error",
                "-r",
                str(args.fps),
                "-i",
                str(raw_stream),
                "-an",
                "-c:v",
                "libx264",
                "-preset",
                "medium",
                "-crf",
                "0",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(recovered),
            ]
        )

    recovered_probe = probe_video(recovered)
    centers_x, centers_y, crop_width, crop_height, source_size = zoom_plan(recovered)
    run_ffmpeg(
        [
            ffmpeg,
            "-y",
            "-loglevel",
            "error",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "bgr24",
            "-s:v",
            "1280x720",
            "-r",
            str(args.fps),
            "-i",
            "pipe:0",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "12",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(zoomed),
        ],
        stdin_frames=zoom_frames(
            recovered,
            centers_x,
            centers_y,
            crop_width,
            crop_height,
            args.fps,
        ),
    )
    zoomed_probe = probe_video(zoomed)
    if recovered_probe["frame_count"] != zoomed_probe["frame_count"]:
        raise RuntimeError("Recovered and zoomed frame counts do not match")

    make_contact_sheet(zoomed, contact_sheet)
    manifest = {
        "schema_version": 1,
        "title": "Historical invalid-RFT locomotion recovery",
        "scientific_status": (
            "visual failure evidence only; invalid mesh/RFT configuration; "
            "not a locomotion or physics baseline"
        ),
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "git": git_state(),
        "source": {
            "path": repository_path(source),
            "bytes": source.stat().st_size,
            "sha256": sha256_file(source),
        },
        "h264_headers": {
            "path": repository_path(headers),
            "bytes": headers.stat().st_size,
            "sha256": sha256_file(headers),
        },
        "recovery": recovery,
        "presentation": {
            "source_size": list(source_size),
            "zoom_crop_size": [crop_width, crop_height],
            "output_size": [1280, 720],
            "zoom_is_derived": True,
            "resimulation": False,
        },
        "artifacts": {
            RECOVERED_NAME: recovered_probe,
            ZOOMED_NAME: zoomed_probe,
            CONTACT_SHEET_NAME: {
                "path": repository_path(contact_sheet),
                "bytes": contact_sheet.stat().st_size,
                "sha256": sha256_file(contact_sheet),
                "width": 1920,
                "height": 720,
            },
        },
    }
    manifest_path = output_dir / MANIFEST_NAME
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
