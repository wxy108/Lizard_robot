"""Preserve a legacy trajectory; compare equal-state provider forces; audit ground.

No original model, gait, mesh or legacy implementation is changed. Canonical
snapshots are forward-synchronized for BOTH force evaluators, independently
of the legacy trajectory generator's cached data timing.
"""

import argparse
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

import mujoco
import numpy as np

import lizard_sand as legacy
from controllers import GaitController, load_config
from rft.binding import load_surfaces
from rft.provider import RFTForceProvider
from rft.records import artifact_manifest, git_state, new_run, sha256, write_json
from sim_fxn_lib import rft_3D_body_full_mat


FROZEN = ["lizard_sand.py","sim_fxn_lib.py","Lizard_Sand.xml","models/lizard.xml",
          "configs/gait.yaml","environment.yml"]


def ground_inventory(model,data):
    rows=[]
    for gid in range(model.ngeom):
        if model.geom_bodyid[gid] != 0:
            continue
        rows.append({"geom_id":gid,"name":mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,gid),
                     "type":int(model.geom_type[gid]),"world_pos_m":data.geom_xpos[gid].tolist(),
                     "world_normal":data.geom_xmat[gid].reshape(3,3)[:,2].tolist(),
                     "contype":int(model.geom_contype[gid]),"conaffinity":int(model.geom_conaffinity[gid]),
                     "rgba":model.geom_rgba[gid].tolist()})
    return rows


def contact_stats(model,data,world_ids):
    count=0
    normal=0.0
    minimum=0.0
    for index in range(data.ncon):
        pair=data.contact[index]
        if int(pair.geom1) not in world_ids and int(pair.geom2) not in world_ids:
            continue
        wrench=np.empty(6)
        mujoco.mj_contactForce(model,data,index,wrench)
        count+=1
        normal+=abs(float(wrench[0]))
        minimum=min(minimum,float(pair.dist))
    return count,normal,minimum


def maximum_error(a,b):
    return float(np.max(np.abs(np.asarray(a)-np.asarray(b)))) if np.size(a) else 0.0


def compare_snapshot(model,data,body_dict,sites,body_ids,history,providers):
    copy=mujoco.MjData(model)
    mujoco.mj_copyData(copy,model,data)
    mujoco.mj_forward(model,copy)
    copy.qfrc_applied[:]=0
    old_history={name:buffer.copy() for name,buffer in history.items()}
    legacy.apply_sand_forces(model,copy,0.0,body_dict,sites,body_ids,old_history,
                             alpha=legacy.ALPHA,rft_coeff=legacy.RFTCOEFF)
    expected_qfrc=copy.qfrc_applied.copy()
    old_raw=[]
    old_masks=[]
    for name in legacy.BODIES:
        bid=body_ids[name]
        velocity=np.empty(6)
        mujoco.mj_objectVelocity(model,copy,mujoco.mjtObj.mjOBJ_BODY,bid,velocity,0)
        result=rft_3D_body_full_mat(body_dict[name],copy.xpos[bid],None,velocity[3:],velocity[:3],
                                   RFTCOEFF=legacy.RFTCOEFF,sand_height_m=0,
                                   rotation_matrix=copy.xmat[bid].reshape(3,3))
        old_raw.append(-np.nan_to_num(result[5],nan=0,posinf=0,neginf=0))
        old_masks.append(result[4])
    old_raw=np.concatenate(old_raw)
    old_masks=np.concatenate(old_masks)
    old_forces=np.concatenate([old_history[name] for name in legacy.BODIES])
    old_positions=np.concatenate([copy.site_xpos[sites[name]] for name in legacy.BODIES])
    original_ctrl=copy.ctrl.copy()
    for provider in providers.values():
        provider.reset(history)
    new=providers["legacy"].compute(copy)
    corrected=providers["body_frame"].compute(copy)
    flat=lambda result,key:np.concatenate([getattr(surface,key) for surface in result.surfaces])
    new_forces=flat(new,"forces")
    new_raw=flat(new,"raw_reaction")
    passed=(np.array_equal(old_masks,flat(new,"active"))
            and np.allclose(old_positions,flat(new,"positions_world"),atol=1e-12,rtol=1e-12)
            and np.allclose(old_forces,new_forces,atol=1e-10,rtol=1e-10)
            and np.allclose(old_raw,new_raw,atol=1e-10,rtol=1e-10)
            and np.allclose(expected_qfrc,new.qfrc,atol=1e-10,rtol=1e-10)
            and np.array_equal(copy.ctrl,original_ctrl)
            and np.array_equal(copy.qfrc_applied,expected_qfrc))
    metrics={"time_s":float(copy.time),"passed":bool(passed),
             "position_max_abs_m":maximum_error(old_positions,flat(new,"positions_world")),
             "raw_force_max_abs_N":maximum_error(old_raw,new_raw),
             "smoothed_force_max_abs_N":maximum_error(old_forces,new_forces),
             "generalized_force_max_abs":maximum_error(expected_qfrc,new.qfrc),
             "mask_mismatches":int(np.count_nonzero(old_masks != flat(new,"active"))),
             "body_frame_force_change_max_abs_N":maximum_error(old_forces,flat(corrected,"forces")),
             "body_frame_generalized_change_max_abs":maximum_error(expected_qfrc,corrected.qfrc),
             "body_frame_mask_mismatches":int(np.count_nonzero(old_masks != flat(corrected,"active")))}
    raw={"qpos":copy.qpos.copy(),"qvel":copy.qvel.copy(),"ctrl":copy.ctrl.copy(),
         "legacy_raw":old_raw,"new_raw":new_raw,"legacy_force":old_forces,
         "new_force":new_forces,"body_frame_force":flat(corrected,"forces"),
         "legacy_qfrc":expected_qfrc,"new_qfrc":new.qfrc,
         "body_frame_qfrc":corrected.qfrc}
    return metrics,raw


def run_sand(duration, *, floor_enabled=True, compare=False):
    args=SimpleNamespace(config=str(ROOT/"configs/gait.yaml"),overrides=[],
                         model=str(ROOT/"Lizard_Sand.xml"),show_force_sites=None)
    model,data,controller,cfg,sand_z,bodies,sites,tips,body_ids=legacy.setup(args)
    floor=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,"floor")
    if not floor_enabled:
        model.geom_contype[floor]=0
        model.geom_conaffinity[floor]=0
        mujoco.mj_forward(model,data)
    inventory=ground_inventory(model,data)
    history=legacy.make_force_buffers(sites)
    providers={}
    if compare:
        surfaces,_=load_surfaces(model,ROOT/"configs/robots/lizard_rft.yaml")
        providers={"legacy":RFTForceProvider(model,surfaces,convention="legacy_lizard_v1"),
                   "body_frame":RFTForceProvider(model,surfaces,convention="body_frame_v1")}
    steps=int(round(duration/model.opt.timestep))
    sample_steps=set(np.unique(np.r_[0,1,20,200,400,np.linspace(0,steps-1,13).astype(int)]).tolist())
    trace=[]
    comparisons=[]
    snapshots=[]
    states=[]
    velocities=[]
    controls=[]
    world_ids={row["geom_id"] for row in inventory if row["contype"] or row["conaffinity"]}
    for step in range(steps):
        controller.update(data)
        if compare and step in sample_steps:
            metrics,raw=compare_snapshot(model,data,bodies,sites,body_ids,history,providers)
            comparisons.append(metrics)
            snapshots.append(raw)
        diagnostics=legacy.advance_simulation(model,data,sand_z,bodies,sites,body_ids,history,
                                               alpha=legacy.ALPHA,rft_coeff=legacy.RFTCOEFF)
        contacts,normal,min_dist=contact_stats(model,data,world_ids)
        min_site=min(float(np.min(data.site_xpos[ids,2])) for ids in sites.values())
        trace.append([data.time,data.qpos[2],min_site,contacts,normal,min_dist,
                      float(np.sum(diagnostics[0][:,2]))])
        states.append(data.qpos.copy())
        velocities.append(data.qvel.copy())
        controls.append(data.ctrl.copy())
        if step % 2000 == 0:
            print(f"Sand floor={floor_enabled} step={step}/{steps}",flush=True)
    return model,data,np.asarray(trace),np.asarray(states),np.asarray(velocities),np.asarray(controls),inventory,comparisons,snapshots


def run_rigid(duration):
    model=mujoco.MjModel.from_xml_path(str(ROOT/"models/lizard.xml"))
    data=mujoco.MjData(model)
    key=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_KEY,"stand")
    mujoco.mj_resetDataKeyframe(model,data,key)
    mujoco.mj_forward(model,data)
    controller=GaitController(model,load_config(ROOT/"configs/gait.yaml"))
    inventory=ground_inventory(model,data)
    world_ids={row["geom_id"] for row in inventory if row["contype"] or row["conaffinity"]}
    trace=[]
    for _ in range(int(round(duration/model.opt.timestep))):
        controller.update(data)
        mujoco.mj_step(model,data)
        count,normal,dist=contact_stats(model,data,world_ids)
        trace.append([data.time,data.qpos[2],count,normal,dist])
    return inventory,np.asarray(trace)


def replay_saved_fixtures(folder,output_dir):
    """Retest final source against frozen force arrays, without resimulation.

    Active history is reconstructed algebraically from the recorded raw and
    smoothed oracle outputs. Inactive history is immaterial because both
    evaluators clear it. This is a source-regression replay, not a new oracle.
    """
    folder=Path(folder).resolve()
    summary=__import__("json").loads((folder/"summary.json").read_text(encoding="utf-8"))
    args=SimpleNamespace(config=str(ROOT/"configs/gait.yaml"),overrides=[],
                         model=str(ROOT/"Lizard_Sand.xml"),show_force_sites=None)
    model,data,*_=legacy.setup(args)
    surfaces,_=load_surfaces(model,ROOT/"configs/robots/lizard_rft.yaml")
    provider=RFTForceProvider(model,surfaces,convention="legacy_lizard_v1")
    output=new_run(output_dir,"final_source_replay")
    rows=[]
    with np.load(folder/"same_state_forces.npz") as saved:
        for index in range(len(saved["qpos"])):
            data.qpos[:]=saved["qpos"][index]
            data.qvel[:]=saved["qvel"][index]
            data.ctrl[:]=saved["ctrl"][index]
            data.time=summary["comparisons"][index]["time_s"]
            mujoco.mj_forward(model,data)
            previous=(saved["legacy_force"][index]-provider.alpha*saved["legacy_raw"][index])/(1-provider.alpha)
            history={};offset=0
            for surface in surfaces:
                count=len(surface.areas)
                history[surface.surface_id]=previous[offset:offset+count]
                offset+=count
            provider.reset(history)
            result=provider.compute(data)
            raw=np.concatenate([surface.raw_reaction for surface in result.surfaces])
            forces=np.concatenate([surface.forces for surface in result.surfaces])
            rows.append({"fixture":index,"raw_max_abs_N":maximum_error(raw,saved["legacy_raw"][index]),
                         "smoothed_max_abs_N":maximum_error(forces,saved["legacy_force"][index]),
                         "qfrc_max_abs":maximum_error(result.qfrc,saved["legacy_qfrc"][index]),
                         "passed":bool(np.allclose(raw,saved["legacy_raw"][index],atol=1e-10,rtol=1e-10)
                              and np.allclose(forces,saved["legacy_force"][index],atol=1e-10,rtol=1e-10)
                              and np.allclose(result.qfrc,saved["legacy_qfrc"][index],atol=1e-10,rtol=1e-10))})
    payload={"source":git_state(),"oracle_sha256":sha256(folder/"same_state_forces.npz"),
             "method":"frozen oracle; active previous history reconstructed from raw/smoothed forces; no resimulation",
             "passed":all(row["passed"] for row in rows),"fixtures":rows}
    write_json(output/"summary.json",payload)
    write_json(output/"manifest.json",{"artifacts":artifact_manifest(output)})
    print(f"Final-source fixture replay: {payload['passed']}; {output}")
    if not payload["passed"]:
        raise SystemExit("Frozen fixture regression failed")


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--duration",type=float,default=6.0)
    parser.add_argument("--output-dir",type=Path)
    parser.add_argument("--replay-run",type=Path)
    args=parser.parse_args()
    if args.replay_run:
        replay_saved_fixtures(args.replay_run,args.output_dir)
        return
    if not np.isfinite(args.duration) or not 0.2 <= args.duration <= 10:
        parser.error("bounded audit duration must be in [0.2,10] seconds")
    output=new_run(args.output_dir,"same_state_floor_audit")
    start_hashes={name:sha256(ROOT/name) for name in FROZEN}
    first=run_sand(args.duration,compare=True)
    second=run_sand(args.duration,floor_enabled=False)
    rigid_inventory,rigid_trace=run_rigid(args.duration)
    trace,states,velocities,controls=first[2:6]
    off_trace,off_states=second[2:4]
    np.savez_compressed(output/"legacy_trajectory.npz",trace=trace,qpos=states,qvel=velocities,ctrl=controls)
    np.savez_compressed(output/"floor_disabled_control.npz",trace=off_trace,qpos=off_states)
    np.savez_compressed(output/"rigid_ground.npz",trace=rigid_trace)
    np.savez_compressed(output/"same_state_forces.npz",**{
        key:np.stack([sample[key] for sample in first[8]]) for key in first[8][0]})
    summary={"source":git_state(),"baseline":"ccc694677461012d488cb5ad7d7e9db945b74a80",
             "duration_s":args.duration,"versions":{"mujoco":mujoco.__version__,"numpy":np.__version__},
             "snapshot_policy":"legacy trajectory states; mj_forward on independent copy for BOTH evaluators; identical preceding legacy smoothing history",
             "tolerances":{"positions_atol_m":1e-12,"forces_atol":1e-10,"forces_rtol":1e-10,"active_masks":"exact"},
             "comparison_passed":all(row["passed"] for row in first[7]),
             "comparisons":first[7],"triangles":sum(len(b.surface.areas) for b in
                 RFTForceProvider(first[0],load_surfaces(first[0],ROOT/"configs/robots/lizard_rft.yaml")[0]).bindings),
             "sand_ground_inventory":first[6],"floor_disabled_inventory":second[6],
             "rigid_ground_inventory":rigid_inventory,
             "sand_ground_contact_steps":int(np.count_nonzero(trace[:,3])),
             "sand_ground_support_steps":int(np.count_nonzero(trace[:,4]>1e-9)),
             "sand_peak_ground_normal_N":float(trace[:,4].max()),
             "sand_min_triangle_centroid_z_m":float(trace[:,2].min()),
             "sand_min_base_z_m":float(trace[:,1].min()),
             "floor_disabled_qpos_max_abs":maximum_error(states,off_states),
             "floor_disabled_trajectory_equal":bool(np.array_equal(states,off_states)),
             "rigid_ground_contact_steps":int(np.count_nonzero(rigid_trace[:,2])),
             "rigid_min_contact_distance_m":float(rigid_trace[:,4].min()),
             "frozen_hashes":start_hashes,
             "baseline_files_unchanged":start_hashes=={name:sha256(ROOT/name) for name in FROZEN}}
    write_json(output/"summary.json",summary)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
    axes[0].plot(trace[:,0],trace[:,1],label="base z")
    axes[0].plot(trace[:,0],trace[:,2],label="minimum RFT centroid z")
    axes[0].axhline(0,color="orange",label="sand z=0; non-colliding")
    axes[0].axhline(-.25,color="gray",ls="--",label="safety floor z=-0.25 m")
    axes[0].set_ylabel("world z [m]"); axes[0].legend(loc="best")
    axes[1].plot(trace[:,0],trace[:,6],label="RFT vertical force")
    axes[1].plot(trace[:,0],trace[:,4],label="rigid ground normal force (sand run)")
    axes[1].set_ylabel("force [N]"); axes[1].legend(loc="best")
    axes[2].plot(trace[:,0],np.max(np.abs(states-off_states),axis=1))
    axes[2].set_ylabel("floor-on/off max |dqpos|"); axes[2].set_xlabel("time [s]")
    fig.suptitle("Legacy lizard ground audit: same initial state and controller")
    fig.tight_layout(); fig.savefig(output/"ground_audit.png",dpi=150); plt.close(fig)
    write_json(output/"manifest.json",{"summary":"summary.json","artifacts":artifact_manifest(output)})
    print(f"Summary: {output/'summary.json'}",flush=True)
    print(f"Same-state comparison: {summary['comparison_passed']}; sand supporting ground contacts: {summary['sand_ground_support_steps']}; floor-disabled qpos error: {summary['floor_disabled_qpos_max_abs']}",flush=True)
    if not summary["comparison_passed"] or not summary["baseline_files_unchanged"]:
        raise SystemExit("Compatibility validation failed; raw outputs retained")


if __name__=="__main__":
    main()
