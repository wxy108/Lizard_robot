# Historical rejected-media source

This directory preserves the exact damaged recording used to recover the
historical invalid-RFT locomotion video. It is evidence only and is not an
active simulation input.

## Files

| File | Bytes | SHA-256 | Meaning |
| --- | ---: | --- | --- |
| `legacy_sand_truncated.mp4` | 1,499,129 | `627F98081BBD6B988F1DF30973C9845E314C604D46D2CCBDC8DB0DBF8B3E54F1` | Original archived recording; interrupted before the MP4 `moov` atom was written |
| `legacy_sand_h264_sps_pps.bin` | 39 | `71612059CF519E13130C1F1D717AAB90BAC3B63597380F456430AFFCB961F8F4` | Matching Annex-B SPS/PPS headers recovered from the same historical x264/1280x720 recording batch |

The source recording came from:

```text
Lizard_Robot_Archive/2026-07-27/
  Lizard_Robot_RFT/Lizard_robot-main/outputs/videos/legacy_sand.mp4
```

Its archived sibling `optimized.mp4` supplied the matching 39-byte H.264
headers. The large sibling video is not required after preserving those exact
headers.

The truncated `mdat` atom declares 4,187,277 payload bytes but only 1,499,081
bytes are present. Those bytes contain 111 complete NAL units and 9,400 bytes
of one incomplete trailing unit. The recovery tool discards only that
incomplete tail and never simulates new states.

Reproduce the reviewed output from a fresh clone:

```bash
python scripts/recover_legacy_rft_video.py
```

See `docs/media/legacy_incorrect_rft_locomotion/` for the playable reviewed
recovery and `docs/FAILED_MESH_DIAGNOSTICS.md` for interpretation.

