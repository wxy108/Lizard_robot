"""Verify immutable pipeline artifacts, decode every video frame, derive evidence."""
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
from rft.records import artifact_manifest, git_state, new_run, sha256, write_json


def verify(directory):
    directory = Path(directory).resolve()
    manifest = json.loads((directory/"manifest.json").read_text())
    for filename,expected in manifest["artifacts"].items():
        path = (directory/filename).resolve()
        if not path.is_relative_to(directory) or not path.is_file():
            raise ValueError(f"Missing/unsafe artifact: {filename}")
        if path.stat().st_size != expected["bytes"] or sha256(path) != expected["sha256"]:
            raise ValueError(f"Artifact mismatch: {path}")
    return manifest


def video_review(path,expected_frames):
    capture = cv2.VideoCapture(str(path))
    reported,fps = int(capture.get(cv2.CAP_PROP_FRAME_COUNT)),capture.get(cv2.CAP_PROP_FPS)
    samples,decoded = {},0
    wanted = (0,reported//2,reported-1)
    while True:
        ok,frame = capture.read()
        if not ok:
            break
        if decoded in wanted:
            samples[decoded] = cv2.resize(frame,(480,270))
        decoded += 1
    capture.release()
    if decoded != reported or decoded != expected_frames or len(samples) != 3:
        raise AssertionError(f"Incomplete video {path}: decoded {decoded}, metadata {reported}, expected {expected_frames}")
    return {"frames_decoded":decoded,"fps":fps,"encoded_duration_s":decoded/fps,
            "sha256":sha256(path)},[samples[i] for i in wanted]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs",type=Path,nargs="+",required=True)
    parser.add_argument("--output-dir",type=Path,required=True)
    parser.add_argument("--compare-runs",type=Path,nargs=2)
    parser.add_argument("--environment-before",type=Path)
    args = parser.parse_args()
    output = new_run(args.output_dir)
    report = {"review_source":git_state(),"review_script_sha256":sha256(__file__),"runs":{}}
    for path in args.runs:
        path = path.resolve()
        manifest = verify(path)
        result = {"raw_directory":str(path),"pipeline":json.loads((path/"pipeline.json").read_text()),
                  "bundle":json.loads((path/"bundle/bundle.json").read_text()),
                  "binding_checks":json.loads((path/"binding_checks.json").read_text()),
                  "original_manifest_sha256":sha256(path/"manifest.json"),"artifacts":manifest["artifacts"],
                  "scenarios":{}}
        sheet_rows = []
        for scenario in ("rigid_upstream","rigid_matched","sand"):
            directory = path/scenario
            verify(directory)
            summary = json.loads((directory/"summary.json").read_text())
            if summary["source"]["dirty"]:
                raise ValueError("Release review requires clean simulation source")
            with np.load(directory/"trajectory.npz",allow_pickle=False) as data:
                trace,active,rigid,groups = data["trace"],data["active"],data["rigid_active"],data["groups"]
                if not all(np.all(np.isfinite(data[key])) for key in ("qpos","qvel","ctrl","qfrc_rft","trace")):
                    raise AssertionError("Non-finite saved telemetry")
                if not all(len(data[key]) == len(trace) for key in ("qpos","qvel","ctrl","qfrc_rft","active","rigid_active")):
                    raise AssertionError("Misaligned saved rows")
                desired = float(summary["effective_controller"]["command"][0])
                actual = summary["com_displacement_m"][0]/summary["recorded_duration_s"]
                derived = {"mean_forward_speed_m_s":actual,"desired_forward_speed_m_s":desired,
                           "mean_forward_speed_bias_m_s":actual-desired,
                           "relative_mean_forward_speed_bias":(actual-desired)/desired if desired else None,
                           "component_active_duty":{str(g):float(np.mean((active[:,i]>0)|(rigid[:,i]>0))) for i,g in enumerate(groups)}}
            metrics,frames = video_review(directory/"locomotion.mp4",summary["video_frames"])
            cells = []
            for frame in frames:
                caption = np.full((24,480,3),255,dtype=np.uint8)
                cv2.putText(caption,scenario,(8,17),cv2.FONT_HERSHEY_SIMPLEX,.45,(15,15,15),1,cv2.LINE_AA)
                cells.append(np.vstack([frame,caption]))
            sheet_rows.append(np.hstack(cells))
            result["scenarios"][scenario] = {"summary":summary,"derived":derived,"video":metrics}
        cv2.imwrite(str(output/(path.name+"_contact_sheet.png")),np.vstack(sheet_rows))
        report["runs"][path.name] = result
    if args.compare_runs:
        comparison = {"directories":[str(p) for p in args.compare_runs],"scenarios":{}}
        for scenario in ("rigid_upstream","rigid_matched","sand"):
            with np.load(args.compare_runs[0]/scenario/"trajectory.npz") as a,np.load(args.compare_runs[1]/scenario/"trajectory.npz") as b:
                comparison["scenarios"][scenario] = {key:bool(np.array_equal(a[key],b[key]))
                    for key in ("qpos","qvel","ctrl","qfrc_rft","trace","active","rigid_active")}
        report["guard_change_repeatability"] = comparison
    if args.environment_before:
        normalize = lambda name:re.sub(r"[-_.]+","-",name).lower()
        before = {normalize(k):v for k,v in json.loads(args.environment_before.read_text()).items()}
        after = {normalize(d.metadata["Name"]):d.version for d in importlib.metadata.distributions()}
        report["environment_delta"] = {"changed_existing":{k:[v,after.get(k)] for k,v in before.items() if after.get(k)!=v},
                                       "added":{k:v for k,v in after.items() if k not in before}}
    write_json(output/"review.json",report)
    write_json(output/"manifest.json",{"artifacts":artifact_manifest(output)})
    print(f"Verified {len(report['runs'])} raw runs and all decoded video frames: {output}")


if __name__ == "__main__":
    main()
