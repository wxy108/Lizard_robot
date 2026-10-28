"""Render an immutable saved trajectory into a NEW folder; never resimulate."""
import argparse
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rft.pipeline import import_robot,load_bundle
from rft.ground import render_run
from rft.records import new_run,write_json,artifact_manifest,git_state,sha256
from scripts.review_ground_pipeline import verify,video_review


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run",type=Path,required=True)
    parser.add_argument("--config",type=Path,required=True)
    parser.add_argument("--scenario",choices=("rigid_upstream","rigid_matched","sand"),required=True)
    parser.add_argument("--output-dir",type=Path,required=True)
    args = parser.parse_args()
    verify(args.run); source = args.run/args.scenario; verify(source)
    ledger = json.loads((args.run/"pipeline.json").read_text())
    summary = json.loads((source/"summary.json").read_text())
    output = new_run(args.output_dir)
    model,config,_ = import_robot(args.config,medium=summary["mode"])
    surfaces,_ = load_bundle(model,Path(ledger.get("bundle_directory",args.run/"bundle")))
    model.opt.timestep = summary["physics_timestep_s"]
    shutil.copy2(source/"trajectory.npz",output/"trajectory.npz")
    shutil.copy2(source/"diagnostics.png",output/"diagnostics.png")
    write_json(output/"summary.json",summary)
    frames = render_run(model,config,surfaces,output)
    if frames != summary["video_frames"]:
        raise AssertionError("Replay frame selection changed")
    metrics,_ = video_review(output/"locomotion.mp4",frames)
    write_json(output/"render_provenance.json",{"render_source":git_state(),"simulation_source":summary["source"],
        "source_directory":str(source.resolve()),"source_manifest_sha256":sha256(source/"manifest.json"),
        "source_trajectory_sha256":sha256(source/"trajectory.npz"),
        "copied_trajectory_sha256":sha256(output/"trajectory.npz"),"physics_steps_executed":0,
        "changes":"Hide collision proxies only on URDF bodies with source visuals; white caption bands",
        "video":metrics})
    write_json(output/"manifest.json",{"artifacts":artifact_manifest(output)})
    print(output)


if __name__ == "__main__":
    main()
