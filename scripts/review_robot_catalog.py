"""Verify a catalog batch, including external cache, failed screens and videos."""
import argparse
import importlib.metadata
import json
from pathlib import Path
import re
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rft.records import new_run,write_json,artifact_manifest,git_state,sha256
from scripts.review_ground_pipeline import verify,video_review


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch",type=Path,required=True)
    parser.add_argument("--output-dir",type=Path,required=True)
    parser.add_argument("--environment-before",type=Path)
    parser.add_argument("--compare-go1",type=Path)
    parser.add_argument("--compare-g1",type=Path)
    parser.add_argument("--compare-package",type=Path)
    args = parser.parse_args(); output = new_run(args.output_dir)
    batch = args.batch.resolve(); verify(batch)
    entries = json.loads((batch/"batch.json").read_text())["results"]
    report = {"review_source":git_state(),"review_script_sha256":sha256(__file__),"runs":{}}
    for entry in entries:
        name = entry["robot"]; directory = batch/name
        result = {"pipeline":entry["pipeline"],"returncode":entry["returncode"],"scenarios":{}}
        report["runs"][name] = result
        if not (directory/"manifest.json").is_file():
            result["evidence_limit"] = "No completed run manifest; retain pipeline/preflight failure"
            continue
        manifest = verify(directory)
        result.update({"raw_directory":str(directory),"artifacts":manifest["artifacts"],
                       "original_manifest_sha256":sha256(directory/"manifest.json")})
        if entry["pipeline"].get("bundle_directory"):
            bundle = Path(entry["pipeline"]["bundle_directory"])
            verify(bundle)
            result["bundle"] = json.loads((bundle/"bundle.json").read_text())
        if (directory/"binding_checks.json").is_file():
            result["binding_checks"] = json.loads((directory/"binding_checks.json").read_text())
        rows = []
        for scenario in ("rigid_upstream","rigid_matched","sand"):
            folder = directory/scenario
            if not (folder/"manifest.json").is_file():
                result["scenarios"][scenario] = {"status":"not_completed"}
                continue
            verify(folder)
            summary = json.loads((folder/"summary.json").read_text())
            if summary["source"]["dirty"]:
                raise ValueError("Reviewed release requires clean simulation source")
            with np.load(folder/"trajectory.npz",allow_pickle=False) as data:
                keys = ("qpos","qvel","ctrl","qfrc_rft","trace","active","rigid_active")
                if not all(len(data[k]) == len(data["trace"]) and np.all(np.isfinite(data[k])) for k in keys):
                    raise AssertionError("Nonfinite or unaligned telemetry")
                command = summary["effective_controller"]["command"][0]
                speed = summary["com_displacement_m"][0]/summary["recorded_duration_s"]
                derived = {"mean_forward_speed_m_s":speed,"commanded_forward_speed_m_s":command,
                    "relative_mean_speed_bias":(speed-command)/command if command else None,
                    "component_contact_duty":{str(g):float(np.mean((data["active"][:,i]>0)|(data["rigid_active"][:,i]>0)))
                        for i,g in enumerate(data["groups"])}}
            item = {"summary":summary,"derived":derived}
            result["scenarios"][scenario] = item
            if (folder/"locomotion.mp4").is_file():
                item["video"],frames = video_review(folder/"locomotion.mp4",summary["video_frames"])
                cells = []
                for frame in frames:
                    caption = np.full((24,480,3),255,dtype=np.uint8)
                    label = f"{name}: {scenario} / {summary['status']}"
                    cv2.putText(caption,label,(4,17),cv2.FONT_HERSHEY_SIMPLEX,.35,(15,15,15),1,cv2.LINE_AA)
                    cells.append(np.vstack([frame,caption]))
                rows.append(np.hstack(cells))
        if rows:
            if not cv2.imwrite(str(output/(name+"_contact_sheet.png")),np.vstack(rows)):
                raise OSError("Preview write failed")
        previous = getattr(args,"compare_"+name,None)
        if previous:
            equality = {}
            for scenario in ("rigid_upstream","rigid_matched","sand"):
                with np.load(previous/scenario/"trajectory.npz") as a,np.load(directory/scenario/"trajectory.npz") as b:
                    equality[scenario] = {k:bool(np.array_equal(a[k],b[k])) for k in
                        ("qpos","qvel","ctrl","qfrc_rft","trace","active","rigid_active")}
            result["previous_runtime_comparison"] = {"directory":str(previous),"exact_array_equality":equality}
    if args.environment_before:
        normalize = lambda name:re.sub(r"[-_.]+","-",name).lower()
        before = {normalize(k):v for k,v in json.loads(args.environment_before.read_text()).items()}
        after = {normalize(d.metadata["Name"]):d.version for d in importlib.metadata.distributions()}
        report["environment_delta"] = {"changed_existing":{k:[v,after.get(k)] for k,v in before.items() if after.get(k)!=v},
                                       "added":{k:v for k,v in after.items() if k not in before}}
    write_json(output/"review.json",report)
    write_json(output/"manifest.json",{"artifacts":artifact_manifest(output)})
    print(f"Reviewed {len(entries)} catalog entries: {output}")


if __name__ == "__main__":
    main()
