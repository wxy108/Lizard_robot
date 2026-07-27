import struct
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from recover_legacy_rft_video import (  # noqa: E402
    build_recovery_stream,
    extract_complete_nals,
    find_mdat_payload,
    split_annex_b,
)


class LegacyRftVideoRecoveryTests(unittest.TestCase):
    def test_annex_b_split_accepts_three_and_four_byte_start_codes(self):
        stream = b"\x00\x00\x00\x01\x67abc\x00\x00\x01\x68de"
        self.assertEqual(split_annex_b(stream), [b"\x67abc", b"\x68de"])

    def test_truncated_mdat_keeps_complete_nals_only(self):
        first = b"\x65abc"
        second = b"\x01defgh"
        partial = struct.pack(">I", 10) + b"\x01xx"
        payload = (
            struct.pack(">I", len(first))
            + first
            + struct.pack(">I", len(second))
            + second
            + partial
        )
        declared_atom_size = len(payload) + 100
        source = (
            struct.pack(">I", 8)
            + b"free"
            + struct.pack(">I", declared_atom_size)
            + b"mdat"
            + payload
        )
        extracted, atom = find_mdat_payload(source)
        units, trailing = extract_complete_nals(extracted)
        self.assertEqual(units, [first, second])
        self.assertEqual(trailing, len(partial))
        self.assertTrue(atom["atom_truncated"])

    def test_tracked_source_recovers_expected_structure(self):
        source = (
            ROOT / "reference" / "rejected_media" / "legacy_sand_truncated.mp4"
        )
        headers = (
            ROOT
            / "reference"
            / "rejected_media"
            / "legacy_sand_h264_sps_pps.bin"
        )
        stream, report = build_recovery_stream(
            source.read_bytes(), headers.read_bytes()
        )
        self.assertTrue(stream.startswith(b"\x00\x00\x00\x01\x67"))
        self.assertEqual(report["complete_nal_units"], 111)
        self.assertEqual(report["discarded_trailing_bytes"], 9400)
        self.assertEqual(report["nal_type_counts"], {"1": 109, "5": 1, "6": 1})
        self.assertTrue(report["atom_truncated"])


if __name__ == "__main__":
    unittest.main()
