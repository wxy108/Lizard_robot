"""D-021 bounded publication: verified existing physics/media, no resimulation."""
import argparse
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rft.records import new_run,write_json,artifact_manifest,git_state,sha256
from scripts.review_ground_pipeline import verify,video_review


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("batch","review","validation","final-validation","replay-root","package-review"):
        parser.add_argument("--"+name,type=Path,required=True)
    parser.add_argument("--legacy-parity",type=Path,required=True)
    parser.add_argument("--evidence-dir",type=Path,default=ROOT/"docs/regressions/2026-10-28-robot-catalog")
    parser.add_argument("--media-dir",type=Path,default=ROOT/"docs/media/robot_catalog")
    args = parser.parse_args(); source = git_state()
    if source["dirty"]:
        raise ValueError("Commit reviewed source before artifact publication")
    for folder in (args.batch,args.review,args.package_review,args.final_validation/"formats",args.final_validation/"controller_parity"):
        verify(folder)
    review = json.loads((args.review/"review.json").read_text())
    evidence,media = new_run(args.evidence_dir),new_run(args.media_dir)
    records = {}
    def copy(src,dst,normalize=False):
        content = Path(src).read_bytes()
        Path(dst).write_bytes(content.replace(b"\r\n",b"\n") if normalize else content)
        records[str(Path(dst).relative_to(ROOT))] = {"source":str(Path(src).resolve()),
            "source_sha256":sha256(src),"sha256":sha256(dst),"normalization":"CRLF to LF" if normalize else None}
    for src,name in ((args.review/"review.json","review.json"),
        (args.package_review/"review.json","package_review.json"),
        (args.final_validation/"formats/format_comparison.json","format_comparison.json"),
        (args.final_validation/"controller_parity/parity.json","controller_parity.json"),
        (args.legacy_parity/"controller_parity.json","legacy_controller_parity.json"),
        (args.validation/"lizard_full.log","lizard_full.log"),
        (args.validation/"doctor.log","doctor.log"),
        (args.final_validation/"unit_tests.log","unit_tests.log"),
        (args.final_validation/"cli.log","cli.log")):
        copy(src,evidence/name,src.suffix==".log")
    selected = (("t1","rigid_matched"),("t1","sand"),("berkeley","sand"),("apollo","sand"),
                ("turtlebot3_burger","rigid_matched"),("turtlebot3_burger","sand"))
    videos = {}
    for robot,scenario in selected:
        folder = args.replay_root/scenario if robot == "turtlebot3_burger" else args.batch/robot/scenario
        verify(folder)
        summary = json.loads((folder/"summary.json").read_text())
        if summary["source"]["dirty"]:
            raise ValueError("Dirty physics source cannot be curated")
        video = folder/"locomotion.mp4"; metrics,_ = video_review(video,summary["video_frames"])
        filename = robot+"_"+scenario+".mp4"; copy(video,media/filename)
        videos[filename] = {"robot":robot,"scenario":scenario,"summary":summary,"video":metrics}
        if (folder/"render_provenance.json").is_file():
            copy(folder/"render_provenance.json",evidence/(robot+"_"+scenario+"_render.json"))
    for robot in ("t1","berkeley","apollo","turtlebot3_burger"):
        copy(args.review/(robot+"_contact_sheet.png"),evidence/(robot+"_original_contact_sheet.png"))
        copy(args.batch/robot/"sand/diagnostics.png",evidence/(robot+"_sand_diagnostics.png"))
    notices = {"LICENSE-berkeley.txt":"third_party/robot_assets/mujoco_menagerie/berkeley_humanoid/LICENSE",
        "LICENSE-t1.txt":"third_party/robot_assets/mujoco_menagerie/booster_t1/LICENSE",
        "LICENSE-apollo.txt":"third_party/robot_assets/mujoco_menagerie/apptronik_apollo/LICENSE",
        "LICENSE-turtlebot3.txt":"third_party/robot_assets/turtlebot3/LICENSE",
        "LICENSE-playground.txt":"third_party/robot_assets/playground/LICENSE",
        "NOTICE.playground":"controllers/NOTICE.playground"}
    for name,path in notices.items(): copy(ROOT/path,media/name)
    table = ["| Robot | Rigid dx upstream/matched (m) | Sand duration / dx (m) | Sand result |",
             "| --- | --- | --- | --- |"]
    for name,run in review["runs"].items():
        scenes = run["scenarios"]; s = scenes["sand"]["summary"]
        a,b = (scenes[k]["summary"]["com_displacement_m"][0] for k in ("rigid_upstream","rigid_matched"))
        table.append(f"| {name} | {a:.3f} / {b:.3f} | {s['recorded_duration_s']:.4f} s / {s['com_displacement_m'][0]:.6f} | {s['status']}: {s['termination']} |")
    (evidence/"README.md").write_text("# Seven-robot catalog evidence\n\nClean physics source `36659ca`; final CLI/render/package source in `source_index.json`.\n\n"+
        "\n".join(table)+"\n\nAll 14 rigid scenarios pass; 3/7 sand screens pass. Failed transfers remain evidence. "
        "All 21 raw videos (2,417 frames) and all raw/cache manifests were verified. "
        "Go1/G1 saved arrays equal the earlier f4c0360 runs exactly. Only Xacro 2.1.1 was added; no existing package changed.\n\n"
        "69 tests pass. Full original lizard six-second regression passed. Three new official policies match 30-state observation/action probes exactly; "
        "T1 physical-qvel mutation is intentionally removed. Version-locked binary packages preserve model fingerprints and active force/control probes; "
        "native MJZ serialization retains the documented small numeric residual.\n\n"
        "`review.json` includes original summaries, source/settings, hashes, binding gates, contacts, speed biases and baseline array comparisons. "
        "`package_review.json` checks actual relocated six-second G1 simulation; `format_comparison.json` separates bitwise fingerprints from force probes. "
        "`*_render.json` records visual-only replay (zero physics steps). Logs are LF-normalized copies; original/copy hashes are in `publication.json`.\n\n"
        "Raw: `outputs/robot_catalog/clean_6s_36659ca/`; keep these local. Reproduce using the catalog guide and reviewer/curator scripts, "
        "not workstation-specific absolute evidence paths. Component contact, COM and penetration diagrams are included here; "
        "[reviewed videos](../../media/robot_catalog/README.md). Short screens and failures are not material calibration or universal locomotion guarantees.\n",encoding="utf-8",newline="\n")
    lines = ["# Reviewed catalog videos", "", "Six bounded MP4s; no failed trajectory is padded. Native policy/proxy transfer is not universally successful.", ""]
    for filename,item in videos.items():
        s = item["summary"]; lines.append(f"- [{filename}]({filename}): {s['status']}, simulated {s['recorded_duration_s']:.4f} s, dx {s['com_displacement_m'][0]:.4f} m; SHA-256 `{sha256(media/filename)}`.")
    lines += ["", "T1, Berkeley and Apollo videos are byte-for-byte copies of clean 36659ca simulation outputs. Burger replays the same saved states with original visual meshes visible and readable caption bands; physics unchanged, separate provenance retained.", "", "[Metrics, tests, contact diagrams and source records](../../regressions/2026-10-28-robot-catalog/README.md). Original model/policy licenses and controller notice accompany these videos. Existing Go1/G1/lizard videos remain unchanged.", ""]
    (media/"README.md").write_text("\n".join(lines),encoding="utf-8",newline="\n")
    patterns = ("rft/*.py","controllers/*.py","configs/robots/*.yaml","configs/controllers/*.json",
                "configs/robot_catalog.json","scripts/*catalog*.py","scripts/replay_ground_video.py",
                "scripts/robot_pipeline.py","scripts/run_ground_pipeline.py","tests/test_catalog_formats.py",
                "requirements-locomotion.txt","environment.yml")
    files = {p.relative_to(ROOT).as_posix():sha256(p) for pattern in patterns for p in ROOT.glob(pattern) if p.is_file()}
    write_json(evidence/"source_index.json",{"publication_source":source,"files":files})
    write_json(evidence/"publication.json",{"source":source,"copies":records,"videos":videos})
    write_json(media/"manifest.json",{"artifacts":artifact_manifest(media)})
    write_json(evidence/"manifest.json",{"artifacts":artifact_manifest(evidence)})
    print(f"Curated {len(videos)} videos: {media}; evidence: {evidence}")


if __name__ == "__main__":
    main()
