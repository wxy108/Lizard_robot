# Checked-in production videos

These are the canonical 6 s / 30 FPS outputs generated from clean source commit
`ff1dda2f954dc27e7d01499f5200497ad30c91f7`.

Start with:

- [master 3×3 + three-panel overview](video_matrix_overview.mp4);
- [animated overview preview](video_matrix_overview_preview.gif);
- [full production manifest and numerical evidence](../../regressions/2026-07-27-video-matrix-production-6s/MANIFEST.md).

## Videos

| Scenario | Top | Side | 45-degree |
| --- | --- | --- | --- |
| Original CAD, rigid ground | [MP4](rigid_original/top.mp4) | [MP4](rigid_original/side.mp4) | [MP4](rigid_original/diag45.mp4) |
| Simplified RFT, sites hidden | [MP4](sand_simplified/top.mp4) | [MP4](sand_simplified/side.mp4) | [MP4](sand_simplified/diag45.mp4) |
| Simplified RFT, sites visible | [MP4](sand_simplified_sites/top.mp4) | [MP4](sand_simplified_sites/side.mp4) | [MP4](sand_simplified_sites/diag45.mp4) |

## Integrity

| File | Bytes | SHA-256 |
| --- | ---: | --- |
| `rigid_original/diag45.mp4` | 2,092,307 | `1AB7D730587D32697CF73B65AE436D9B18938478FBA5ECF17F56A4F97B783BE6` |
| `rigid_original/side.mp4` | 1,656,638 | `D6616550325428B4BDC6DA999688FC6409F7C82F876B91F5E1E82FF2216C77F0` |
| `rigid_original/top.mp4` | 1,491,787 | `1887CE7CA9528D706486703D92FB13DDCC6396A9F8C30879A6CD6C974CAE79FD` |
| `sand_simplified/diag45.mp4` | 1,263,098 | `3360B64D55D762E6074C73FD6A5B055EB707BE64D3A591585E3F7BE7DADE0D6F` |
| `sand_simplified/side.mp4` | 1,085,524 | `7A50A4577841A432149EF2EE60AD0209996108452FA88CD55CE4EF9D9CA1434F` |
| `sand_simplified/top.mp4` | 1,501,829 | `8699C9F7A6A6DAF6258A77CE981F8655F73817DAA39A8B73314B65D924F36108` |
| `sand_simplified_sites/diag45.mp4` | 2,578,303 | `4EB68A311CD36F2A4FA704ABE779FCFCB7DDD74DB9BA04F5AB5DDB104CFD3B55` |
| `sand_simplified_sites/side.mp4` | 2,101,944 | `8FB2460DF85A9AFA80C2DCAE132597997178C4ECC65AF173162FC58AFF550252` |
| `sand_simplified_sites/top.mp4` | 2,210,794 | `BB78A827E29E05C84CBB42D6F3FE28CEDB8836E36E1B48A9D377AA06561A2FA3` |
| `video_matrix_overview.mp4` | 4,539,623 | `7341225A5E79EA6FB34A1CDF0D02F22F914C727D0A9D7DAC779EC61BBA6BDB90` |
| `video_matrix_overview_mid.png` | 429,591 | `09AEAD4C8B1E7E37A375C2F944D5409AF678BE0A00DDB93702070DB334697CE2` |
| `video_matrix_overview_preview.gif` | 1,589,742 | `CA6BFDDF91A99287FBA1F65474F1F2E0FD8A7BE90DF9E7D4377DCBD6E50A6121` |

The MP4 files total approximately 19.6 MiB. Raw NPZ/CSV runs remain
reproducible local outputs and are not duplicated here.
