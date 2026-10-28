"""Config-driven robot RFT integration demo; controller and provider are separate."""

import argparse
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

import mujoco
import numpy as np
import yaml

from controllers.generic_controller import ActuatorCommandController
from rft import RFTForceProvider,load_model,load_surfaces
from rft.records import artifact_manifest,git_state,new_run,sha256,write_json


def make_controller(model,data,config):
    spec=config.get("controller",{"type":"hold_keyframe"})
    kind=spec.get("type")
    if kind=="lizard":
        # The optional legacy controller is an adapter, not an RFT dependency.
        from controllers import GaitController,load_config
        return GaitController(model,load_config(ROOT/"configs/gait.yaml")),True
    if kind not in ("sine","hold_keyframe"):
        raise ValueError(f"Unknown independent controller: {kind}")
    indices=[]
    for name in spec.get("actuators",[]):
        index=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR,name)
        if index<0:
            raise ValueError(f"Controller actuator not found: {name}")
        indices.append(index)
    return ActuatorCommandController(data.ctrl,indices=indices,
                 amplitude=spec.get("amplitude",0),frequency=spec.get("frequency",0.5)),False


def render(model,states,surfaces,sand_height,config,output,fps,show_points):
    import cv2
    import imageio.v2 as imageio

    model.vis.global_.offwidth=960
    model.vis.global_.offheight=540
    data=mujoco.MjData(model)
    camera=mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    settings=config.get("camera",{})
    camera.lookat[:]=settings.get("lookat",[0,0,0.1])
    camera.distance=settings.get("distance",1)
    camera.azimuth=settings.get("azimuth",135)
    camera.elevation=settings.get("elevation",-20)
    maximum=model.ngeom+sum(len(surface.centers) for surface in surfaces)+model.nsite+100
    renderer=mujoco.Renderer(model,height=540,width=960,max_geom=max(10000,maximum))
    writer=imageio.get_writer(str(output/"robot_rft.mp4"),fps=fps,codec="libx264",macro_block_size=2)
    try:
        for time,qpos in states:
            data.qpos[:]=qpos
            mujoco.mj_forward(model,data)
            option=mujoco.MjvOption()
            option.sitegroup[5]=0
            renderer.update_scene(data,camera=camera,scene_option=option)
            scene=renderer.scene
            geom=scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_PLANE,np.array([2.,2.,.001]),
                               np.array([0.,0.,sand_height]),np.eye(3).ravel(),
                               np.array([0.68,0.6,0.49,0.3],dtype=np.float32))
            scene.ngeom+=1
            if show_points:
                for surface in surfaces:
                    bid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,surface.body_name)
                    positions=surface.centers@data.xmat[bid].reshape(3,3).T+data.xpos[bid]
                    for position in positions:
                        geom=scene.geoms[scene.ngeom]
                        mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_SPHERE,np.array([.0015,0,0]),
                                           position,np.eye(3).ravel(),np.array([1.,.65,.1,1.],dtype=np.float32))
                        scene.ngeom+=1
            frame=renderer.render()
            cv2.putText(frame,f"RFT integration demo | t={time:.2f}s | no rigid sand floor",(15,30),
                        cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1,cv2.LINE_AA)
            cv2.putText(frame,"Selected proxy geometry; controller is hold/sine, not a walking policy",(15,55),
                        cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1,cv2.LINE_AA)
            writer.append_data(frame)
    finally:
        writer.close()
        renderer.close()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--config",type=Path,required=True)
    parser.add_argument("--duration",type=float,default=2.0)
    parser.add_argument("--output-dir",type=Path)
    parser.add_argument("--video",action="store_true")
    parser.add_argument("--show-points",action="store_true")
    parser.add_argument("--fps",type=int,default=20)
    args=parser.parse_args()
    if not 0<args.duration<=30 or args.fps<=0:
        parser.error("Duration must be in (0,30] seconds and FPS positive")
    path=args.config.resolve()
    config=yaml.safe_load(path.read_text(encoding="utf-8"))
    model_path=(path.parent/config["model"]).resolve()
    model=load_model(model_path,package_roots={key:(path.parent/value).resolve()
                                             for key,value in config.get("package_roots",{}).items()})
    dt=float(config.get("timestep",model.opt.timestep))
    if not np.isfinite(dt) or dt<=0:
        raise ValueError("Timestep must be positive")
    model.opt.timestep=dt
    model.opt.integrator=mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    data=mujoco.MjData(model)
    key=config.get("initial_keyframe")
    if key:
        key_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_KEY,key)
        if key_id<0:
            raise ValueError(f"Initial keyframe is missing: {key}")
        mujoco.mj_resetDataKeyframe(model,data,key_id)
    mujoco.mj_forward(model,data)
    surfaces,_=load_surfaces(model,path)
    initial_min=min(float(np.min(surface.centers@data.xmat[
        mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,surface.body_name)].reshape(3,3).T
        +data.xpos[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,surface.body_name)],axis=0)[2])
        for surface in surfaces)
    sand=float(config.get("sand_height",initial_min+config.get("sand_surface_above_initial_min_m",0)))
    provider=RFTForceProvider(model,surfaces,convention=config.get("convention","body_frame_v1"),
                              sand_height=sand,coefficient=config.get("coefficient",3.75),
                              alpha=config.get("alpha",0.5),application=config.get("application","triangles"))
    controller,legacy_adapter=make_controller(model,data,config)
    output=new_run(args.output_dir,path.stem)
    trace=[]; states=[]; frames=[]
    render_every=max(1,round(1/(args.fps*dt)))
    steps=max(1,round(args.duration/dt))
    for step in range(steps):
        mujoco.mj_forward(model,data)
        if legacy_adapter:
            controller.update(data)
        else:
            data.ctrl[:]=controller.update({"qpos":data.qpos.copy(),"qvel":data.qvel.copy()},float(data.time))
        result=provider.compute(data)
        data.qfrc_applied[:]=result.qfrc
        masses=model.body_mass[1:]
        com=np.average(data.xipos[1:],axis=0,weights=masses)
        active=sum(int(np.count_nonzero(surface.active)) for surface in result.surfaces)
        depth=max(max(0,sand-float(np.min(surface.positions_world[:,2]))) for surface in result.surfaces)
        trace.append([data.time,*com,*result.total_force,active,depth,np.linalg.norm(result.qfrc)])
        if step%render_every==0 or step==steps-1:
            frames.append((float(data.time),data.qpos.copy()))
        mujoco.mj_step(model,data)
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            raise FloatingPointError(f"Non-finite state at step {step}")
        states.append(data.qpos.copy())
    trace=np.asarray(trace)
    np.savez_compressed(output/"trajectory.npz",trace=trace,qpos=np.asarray(states),
                        surface_order=np.asarray([s.surface_id for s in surfaces]))
    if args.video:
        render(model,frames,surfaces,sand,config,output,args.fps,args.show_points)
    summary={"source":git_state(),"config":path.relative_to(ROOT).as_posix(),"config_sha256":sha256(path),
             "model":str(model_path),"model_sha256":sha256(model_path),"duration_s":steps*dt,
             "timestep_s":dt,"integrator":"implicitfast","controller":config.get("controller",{}),
             "convention":provider.convention,"sand_height_m":sand,"initial_min_centroid_z_m":initial_min,
             "geometry_status":config.get("geometry_status","explicit selected surfaces"),
             "surface_count":len(surfaces),"triangle_count":sum(len(s.areas) for s in surfaces),
             "bindings":[{"surface":s.surface_id,"body":s.body_name,"triangles":len(s.areas),
                           "source":s.source,"group":s.group} for s in surfaces],
             "active_steps":int(np.count_nonzero(trace[:,7])),"maximum_penetration_m":float(trace[:,8].max()),
             "peak_total_rft_force_N":float(np.linalg.norm(trace[:,4:7],axis=1).max()),
             "state_finite":True,"robot_nsite":model.nsite,
             "video_frames":len(frames) if args.video else 0,
             "warning":"Integration screening, not validated granular locomotion or calibrated material"}
    write_json(output/"summary.json",summary)
    write_json(output/"manifest.json",{"artifacts":artifact_manifest(output)})
    print(f"Summary: {output/'summary.json'}")
    print(f"{len(surfaces)} surfaces / {summary['triangle_count']} triangles; active steps={summary['active_steps']}; finite state={summary['state_finite']}")
    if not summary["active_steps"]:
        raise SystemExit("No active RFT in this screening run; output retained")


if __name__=="__main__":
    main()
