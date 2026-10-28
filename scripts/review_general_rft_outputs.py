"""Verify run manifests, decode demo frames and render saved-state floor evidence."""

import argparse
import json
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

import cv2
import mujoco
import numpy as np

from rft.records import artifact_manifest,sha256,write_json


def verify(folder):
    manifest=json.loads((folder/"manifest.json").read_text(encoding="utf-8"))
    for name,entry in manifest["artifacts"].items():
        path=folder/name
        if path.stat().st_size!=entry["bytes"] or sha256(path)!=entry["sha256"]:
            raise ValueError(f"Artifact mismatch: {path}")


def ground_images(folder):
    import lizard_sand as legacy

    args=SimpleNamespace(config=str(ROOT/"configs/gait.yaml"),overrides=[],
                         model=str(ROOT/"Lizard_Sand.xml"),show_force_sites=None)
    model,data,*_=legacy.setup(args)
    with np.load(folder/"legacy_trajectory.npz") as saved:
        frame_index=len(saved["qpos"])//2
        data.qpos[:]=saved["qpos"][frame_index]
        data.qvel[:]=saved["qvel"][frame_index]
        time=float(saved["trace"][frame_index,0])
    mujoco.mj_forward(model,data)
    floor=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,"floor")
    renderer=mujoco.Renderer(model,width=960,height=540,max_geom=18000)
    frames=[]
    try:
        for hidden in (False,True):
            model.geom_rgba[floor,3]=0 if hidden else 1
            option=mujoco.MjvOption()
            option.sitegroup[5]=0
            renderer.update_scene(data,camera="track_side",scene_option=option)
            frame=renderer.render()
            label="Safety floor visually hidden" if hidden else "Safety floor visible at z=-0.25 m"
            cv2.putText(frame,label,(15,30),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1,cv2.LINE_AA)
            cv2.putText(frame,f"Same saved pose t={time:.3f}s; rendering only",(15,55),
                        cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1,cv2.LINE_AA)
            frames.append(frame.copy())
    finally:
        renderer.close()
    cv2.imwrite(str(folder/"same_pose_floor_visibility.png"),
                cv2.cvtColor(np.concatenate(frames,axis=1),cv2.COLOR_RGB2BGR))
    return {"same_pose_time_s":time,"frame_index":frame_index,
            "rendering_only":True,"simulation_repeated":False}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("run_dir",type=Path)
    parser.add_argument("--ground-images",action="store_true")
    args=parser.parse_args()
    folder=args.run_dir.resolve()
    verify(folder)
    review={"original_manifest_sha256":sha256(folder/"manifest.json"),"verified":True}
    video=folder/"robot_rft.mp4"
    if video.is_file():
        cap=cv2.VideoCapture(str(video))
        count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fps=float(cap.get(cv2.CAP_PROP_FPS))
        frames=[]
        for index in (0,count//2,count-1):
            cap.set(cv2.CAP_PROP_POS_FRAMES,index)
            ok,frame=cap.read()
            if not ok or frame.shape!=(540,960,3):
                raise ValueError(f"Video frame decode failed: {index}")
            frames.append(frame)
        cap.release()
        cv2.imwrite(str(folder/"video_contact_sheet.png"),np.concatenate(frames,axis=0))
        review.update(video_frames=count,fps=fps,encoded_duration_s=count/fps,
                      decoded_frames=[0,count//2,count-1])
    if args.ground_images:
        review.update(ground_images(folder))
    write_json(folder/"review.json",review)
    # Regenerate after adding derived review artifacts, retaining each hash.
    write_json(folder/"manifest.json",{"artifacts":artifact_manifest(folder)})
    print(json.dumps(review,indent=2))


if __name__=="__main__":
    main()
