"""Verify fresh remote-source runs before removing their temporary checkout."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rft.records import write_json,artifact_manifest,git_state,sha256
from scripts.review_ground_pipeline import verify,video_review


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fresh-root",type=Path,required=True)
    parser.add_argument("--stdout-log",type=Path,required=True)
    parser.add_argument("--evidence-dir",type=Path,required=True)
    args = parser.parse_args(); fresh = args.fresh_root.resolve(); target = args.evidence_dir.resolve()
    verify(target)
    report_path = target/"fresh_github_check.json"; log_path = target/"fresh_github_check.log"
    if report_path.exists() or log_path.exists():
        raise FileExistsError("Fresh check evidence already exists")
    git = lambda *a:subprocess.check_output(["git","-C",str(fresh),*a],text=True).strip()
    if git("status","--porcelain"):
        raise ValueError("Fresh deployment source must be clean")
    report = {"verification_source":git_state(),"verification_script_sha256":sha256(__file__),
        "remote":git("remote","get-url","origin"),"fresh_commit":git("rev-parse","HEAD"),
        "environment":"reused lizard_rft; no new OS/Conda environment tested", "robots":{}}
    for name,subdir,baseline in (("t1","fresh_t1",ROOT/"outputs/robot_catalog/clean_6s_36659ca/t1"),
        ("g1_package","fresh_g1_package",ROOT/"outputs/robot_catalog/clean_6s_36659ca/g1")):
        batch = fresh/"outputs"/subdir; verify(batch)
        job = "package" if name == "g1_package" else name
        run = batch/job; verify(run)
        ledger = json.loads((run/"pipeline.json").read_text())
        if ledger["status"] != "passed_screen":
            raise AssertionError("Fresh deployment failed its locomotion screen")
        verify(Path(ledger["bundle_directory"]))
        scenarios = {}
        for scenario in ("rigid_upstream","rigid_matched","sand"):
            folder = run/scenario; verify(folder)
            summary = json.loads((folder/"summary.json").read_text())
            if summary["source"] != {"commit":report["fresh_commit"],"dirty":False}:
                raise AssertionError("Unexpected fresh-run source")
            with np.load(folder/"trajectory.npz") as a,np.load(baseline/scenario/"trajectory.npz") as b:
                equality = {k:bool(np.array_equal(a[k],b[k])) for k in
                    ("qpos","qvel","ctrl","qfrc_rft","trace","active","rigid_active")}
            if not all(equality.values()):
                raise AssertionError(f"Same-configuration deterministic replay changed: {name}/{scenario}: {equality}")
            video,_ = video_review(folder/"locomotion.mp4",summary["video_frames"])
            scenarios[scenario] = {"summary":summary,"video":video,"exact_array_equality":equality,
                "trajectory_sha256":sha256(folder/"trajectory.npz"),"manifest_sha256":sha256(folder/"manifest.json")}
        report["robots"][name] = {"baseline":str(baseline),"pipeline":ledger,"scenarios":scenarios}
    for model in ("unitree_g1","unitree_go1"):
        if (fresh/"third_party/robot_assets/mujoco_menagerie"/model).exists():
            raise AssertionError("Package-only check unexpectedly has original G1/Go1 assets")
    report["g1_original_assets_present"] = False
    report["stdout_original_sha256"] = sha256(args.stdout_log)
    log_path.write_bytes(args.stdout_log.read_bytes().replace(b"\r\n",b"\n"))
    report["stdout_LF_copy_sha256"] = sha256(log_path)
    write_json(report_path,report)
    write_json(target/"manifest.json",{"artifacts":artifact_manifest(target)})
    print(f"Fresh remote source + automatic T1 assets + offline G1 package verified: {report_path}")


if __name__ == "__main__":
    main()
