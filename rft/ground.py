"""Bounded ground-controller evaluation, independent of surface preparation."""
from pathlib import Path

import mujoco
import numpy as np

from controllers.catalog_controllers import make_controller
from .provider import RFTForceProvider
from .records import artifact_manifest, git_state, sha256, write_json


def support_ids(model, names, *, allow_explicit_pairs=False):
    if not names:
        raise ValueError("Ground support geoms must be declared explicitly")
    def static_or_kinematic(gid):
        bid = int(model.geom_bodyid[gid])
        return model.body_weldid[bid] == 0 or model.body_mocapid[bid] >= 0
    selected = set()
    for name in names:
        gid = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,name)
        if gid < 0 or not static_or_kinematic(gid):
            raise ValueError(f"Support geom must be a named world geom: {name}")
        selected.add(gid)
    other = [i for i in range(model.ngeom) if static_or_kinematic(i) and i not in selected
             and (model.geom_contype[i] or model.geom_conaffinity[i])]
    if other:
        raise ValueError(f"Undeclared rigid world support/obstacles: {other}; review explicitly")
    if not allow_explicit_pairs and any(int(a) in selected or int(b) in selected for a,b in zip(model.pair_geom1,model.pair_geom2)):
        raise ValueError("Explicit contact pairs override masks; support conversion needs a reviewed pair adapter")
    return selected


def contact_forces(model,data,supports,groups):
    active = np.zeros(len(groups),dtype=int)
    total = np.zeros(3)
    pairs = []
    for i in range(data.ncon):
        contact = data.contact[i]
        g1,g2 = int(contact.geom1),int(contact.geom2)
        if g1 not in supports and g2 not in supports:
            continue
        other = g2 if g1 in supports else g1
        owner = int(model.geom_bodyid[other])
        for j,owners in enumerate(groups):
            if owner in owners:
                active[j] = 1
        local = np.zeros(6)
        mujoco.mj_contactForce(model,data,i,local)
        # Contact frame normal points from geom1 to geom2. This is force on
        # robot geom2 when support is geom1; reverse for robot geom1.
        world = contact.frame.reshape(3,3).T @ local[:3]
        total += world if g1 in supports else -world
        pairs.append((g1,g2,float(contact.dist),float(local[0])))
    return active,total,pairs


def run(model,config,config_path,surfaces,directory,*,mode,duration,dt):
    directory = Path(directory)
    directory.mkdir(parents=True,exist_ok=False)
    controls = config["controller"]
    control_dt = float(controls["control_dt"])
    cadence = round(control_dt/dt)
    if cadence <= 0 or not np.isclose(cadence*dt,control_dt,rtol=0,atol=1e-12):
        raise ValueError("Physics timestep must divide policy period exactly")
    model.opt.timestep = dt
    root = model.body(config["root_body"]).id
    floating = model.body(config.get("floating_body",config["root_body"])).id
    root_joints = range(int(model.body_jntadr[floating]),int(model.body_jntadr[floating]+model.body_jntnum[floating]))
    if not any(model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE for j in root_joints):
        raise ValueError("This walking adapter requires a floating root; fixed/planar roots need an explicit adapter")
    for bid in range(1,model.nbody):
        ancestor = bid
        while ancestor and ancestor != floating:
            ancestor = int(model.body_parentid[ancestor])
        if not ancestor and model.body_weldid[bid] != 0 and model.body_mocapid[bid] < 0:
            raise ValueError("Undeclared dynamic actor outside robot; use an explicit scene adapter")
    supports = support_ids(model,config["support_geoms"],allow_explicit_pairs=(mode == "rigid"))
    original_masks = {gid:(int(model.geom_contype[gid]),int(model.geom_conaffinity[gid])) for gid in supports}
    if mode == "sand":
        for gid in supports:
            model.geom_contype[gid] = model.geom_conaffinity[gid] = 0
    elif mode != "rigid":
        raise ValueError("Medium must be rigid or sand")
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key(config["initial_keyframe"]).id)
    mujoco.mj_forward(model,data)
    policy,policy_path = make_controller(model,data,controls,Path(config_path).parent)
    provider = RFTForceProvider(model,surfaces,convention="body_frame_v1",
                 sand_height=config["sand_height"],coefficient=3.75,alpha=0.5,application="body_wrench")
    groups = list(dict.fromkeys(s.group for s in surfaces))
    owners = [{model.body(s.body_name).id for s in surfaces if s.group == group} for group in groups]
    criteria = config["acceptance"]
    trace,states,velocities,commands,forces,active_list,contacts = [],[],[],[],[],[],[]
    pairs_seen, warning_previous = set(), np.array([w.number for w in data.warning])
    reason = "completed"
    bad_steps = 0
    start_com = None
    try:
        for step in range(round(duration/dt)):
            mujoco.mj_forward(model,data)
            if (step+1) % cadence == 0:
                data.ctrl[:] = policy.update(model,data)
            result = provider.compute(data) if mode == "sand" else None
            data.qfrc_applied[:] = result.qfrc if result is not None else 0
            com = np.average(data.xipos[1:],axis=0,weights=model.body_mass[1:])
            if start_com is None:
                start_com = com.copy()
            rigid_active,rigid_force,pairs = contact_forces(model,data,supports,owners)
            pairs_seen.update((model.geom(a).name,model.geom(b).name) for a,b,_,_ in pairs)
            group_active = np.zeros(len(groups),dtype=int)
            penetration = 0.0
            if result is not None:
                for surface,diagnostic in zip(surfaces,result.surfaces):
                    group_active[groups.index(surface.group)] += int(np.count_nonzero(diagnostic.active))
                    penetration = max(penetration,max(0,config["sand_height"]-float(diagnostic.positions_world[:,2].min())))
            upright = float(data.xmat[root].reshape(3,3)[2,2])
            force = result.total_force if result is not None else np.zeros(3)
            power = float(np.dot(result.qfrc,data.qvel)) if result is not None else 0.0
            trace.append([data.time,*com,data.xpos[root,2],upright,penetration,*force,*rigid_force,power,
                          len(pairs),min((p[2] for p in pairs),default=0)])
            states.append(data.qpos.copy()); velocities.append(data.qvel.copy()); commands.append(data.ctrl.copy())
            forces.append(data.qfrc_applied.copy()); active_list.append(group_active); contacts.append(rigid_active)
            mujoco.mj_step(model,data)
            warnings = np.array([w.number for w in data.warning])
            if np.any(warnings > warning_previous):
                reason = "numerical_warning"
                break
            if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
                reason = "nonfinite_state"
                break
            bad = (upright < criteria["minimum_upright_cosine"] or
                   trace[-1][4] < criteria["minimum_root_height_m"] or
                   penetration > criteria["max_centroid_penetration_m"])
            bad_steps = bad_steps+1 if bad else 0
            if bad_steps*dt >= 0.1:
                reason = "outside_declared_locomotion_envelope_for_100ms"
                break
    except Exception as exc:
        reason = f"exception:{type(exc).__name__}:{exc}"
    finally:
        for gid,(ct,ca) in original_masks.items():
            model.geom_contype[gid],model.geom_conaffinity[gid] = ct,ca
    if not trace:
        raise RuntimeError(f"No recorded state: {reason}")
    trace = np.asarray(trace)
    np.savez_compressed(directory/"trajectory.npz",trace=trace,qpos=np.asarray(states),qvel=np.asarray(velocities),
                        ctrl=np.asarray(commands),qfrc_rft=np.asarray(forces),active=np.asarray(active_list),
                        rigid_active=np.asarray(contacts),groups=np.asarray(groups),final_qpos=data.qpos.copy(),
                        final_qvel=data.qvel.copy())
    displacement = trace[-1,1:4]-start_com
    final_active = np.asarray(active_list)
    minimum_progress = max(criteria["minimum_forward_displacement_m"],
        criteria.get("minimum_commanded_progress_fraction",0)*max(0,float(controls["command"][0]))*duration)
    acceptance_failures = ([reason] if reason != "completed" else [])
    if displacement[0] < minimum_progress:
        acceptance_failures.append("insufficient_forward_displacement")
    if mode == "sand":
        if not final_active.any(): acceptance_failures.append("no_active_rft")
        if np.any(trace[:,14] != 0): acceptance_failures.append("unexpected_rigid_support")
    passed = not acceptance_failures
    summary = {"source":git_state(),"status":"passed_screen" if passed else "failed_screen",
        "termination":reason,"requested_duration_s":duration,"recorded_duration_s":len(trace)*dt,
        "physics_timestep_s":dt,"control_dt_s":control_dt,"policy_updates":policy.updates,
        "first_policy_update_pre_step_time_s":(cadence-1)*dt,
        "mode":mode,"integrator":int(model.opt.integrator),"effective_controller":controls,
        "support_conversion":config.get("effective_support_conversion",{"removed_explicit_pairs":[]}),
        "effective_ground_masks_during_simulation":{model.geom(gid).name:([0,0] if mode == "sand" else list(masks))
                                                   for gid,masks in original_masks.items()},
        "policy_sha256":sha256(policy_path) if policy_path else None,"model_mass_kg":float(model.body_mass.sum()),
        "model_adapter":config.get("effective_model_adapter"),
        "acceptance_failures":acceptance_failures,"minimum_required_forward_progress_m":minimum_progress,
        "geometry_status":config["geometry_status"],"triangle_count":sum(len(s.areas) for s in surfaces),
        "rft_convention":"body_frame_v1","coefficient":3.75,"alpha":0.5,
        "acceptance":criteria,"com_displacement_m":displacement.tolist(),
        "min_root_height_m":float(trace[:,4].min()),"min_upright_cosine":float(trace[:,5].min()),
        "max_centroid_penetration_m":float(trace[:,6].max()),
        "max_total_rft_force_N":float(np.linalg.norm(trace[:,7:10],axis=1).max()),
        "max_rigid_support_force_N":float(np.linalg.norm(trace[:,10:13],axis=1).max()),
        "rigid_support_contact_steps":int(np.count_nonzero(trace[:,14])),
        "rigid_contact_pairs":sorted(pairs_seen),"rft_active_steps":int(np.count_nonzero(final_active.sum(axis=1))),
        "max_smoothed_rft_power_W":float(trace[:,13].max()),
        "positive_smoothed_power_steps":int(np.count_nonzero(trace[:,13] > 1e-8)),
        "trace_columns":["time_s","com_x","com_y","com_z","root_z","upright_cosine","penetration_m",
                          "rft_fx","rft_fy","rft_fz","rigid_fx","rigid_fy","rigid_fz","rft_power_W",
                          "rigid_contact_count","rigid_min_distance_m"],
        "state_alignment":"All rows are pre-step, force/control at same qpos/qvel/time; final state stored separately",
        "boundary":"Bounded engineering screen, not material calibration or guaranteed policy transfer"}
    write_json(directory/"summary.json",summary)
    write_json(directory/"manifest.json",{"artifacts":artifact_manifest(directory)})
    print(f"{directory.name}: {summary['status']} ({reason}), dx={displacement[0]:.3f} m, depth={summary['max_centroid_penetration_m']*1000:.2f} mm",flush=True)
    return summary


def plot_run(directory):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    directory = Path(directory)
    with np.load(directory/"trajectory.npz",allow_pickle=False) as saved:
        trace,active,rigid,groups = saved["trace"],saved["active"],saved["rigid_active"],saved["groups"]
    fig,axes = plt.subplots(4,1,figsize=(10,9),sharex=True)
    t = trace[:,0]
    axes[0].plot(t,trace[:,1]-trace[0,1],label="COM dx (m)")
    axes[0].plot(t,trace[:,2]-trace[0,2],label="COM dy (m)")
    axes[0].plot(t,trace[:,4],label="root height (m)"); axes[0].legend(loc="upper left",ncol=3)
    axes[1].plot(t,trace[:,6]*1000,label="centroid penetration (mm)"); axes[1].legend()
    axes[2].plot(t,trace[:,9],label="RFT Fz (N)"); axes[2].plot(t,trace[:,12],label="rigid Fz (N)"); axes[2].legend()
    diagram = ((active > 0) | (rigid > 0)).T
    axes[3].imshow(diagram,aspect="auto",interpolation="nearest",origin="lower",cmap="Greys",
                   vmin=0,vmax=1,extent=[t[0],t[-1]+1e-9,-.5,len(groups)-.5])
    axes[3].set_yticks(range(len(groups)),groups); axes[3].set_xlabel("simulation time (s)")
    axes[3].set_title("Component contact: 0 white / 1 black; RFT-active or native rigid contact")
    fig.suptitle(directory.name+" | finite motion alone is not locomotion acceptance")
    fig.tight_layout(); fig.savefig(directory/"diagnostics.png",dpi=130); plt.close(fig)


def render_hidden_proxy_ids(model):
    """Hide private URDF collision proxies only on bodies with source visuals.

    Collision-only casters stay visible; native model/physics never change.
    """
    visual_bodies = {int(model.geom_bodyid[i]) for i in range(model.ngeom)
        if model.geom(i).name.startswith(model.body(int(model.geom_bodyid[i])).name+"__visual_")}
    return {i for i in range(model.ngeom) if int(model.geom_bodyid[i]) in visual_bodies
        and model.geom(i).name.startswith(model.body(int(model.geom_bodyid[i])).name+"__collision_")}


def render_run(model,config,surfaces,directory,*,fps=20):
    import cv2
    import imageio.v2 as imageio
    directory = Path(directory)
    with np.load(directory/"trajectory.npz",allow_pickle=False) as saved:
        trace,qpos = saved["trace"],saved["qpos"]
    summary = __import__("json").loads((directory/"summary.json").read_text())
    model.vis.global_.offwidth,model.vis.global_.offheight = 960,540
    model.vis.quality.shadowsize = 2048
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(camera)
    for key,value in config["camera"].items():
        setattr(camera,key,value)
    renderer = mujoco.Renderer(model,height=540,width=960,max_geom=max(10000,model.ngeom+sum(len(s.areas) for s in surfaces)+500))
    writer = imageio.get_writer(str(directory/"locomotion.mp4"),fps=fps,codec="libx264",macro_block_size=2)
    indices = np.unique(np.rint(np.linspace(0,len(trace)-1,max(2,round(len(trace)*model.opt.timestep*fps)+1))).astype(int))
    owners = [model.body(s.body_name).id for s in surfaces]
    option = mujoco.MjvOption(); option.geomgroup[3] = 0
    hidden_proxies = render_hidden_proxy_ids(model)
    try:
        for index in indices:
            data.qpos[:] = qpos[index]; mujoco.mj_forward(model,data)
            camera.lookat[:] = trace[index,1:4]
            renderer.update_scene(data,camera=camera,scene_option=option)
            for i in range(renderer.scene.ngeom):
                geom = renderer.scene.geoms[i]
                if geom.objtype == mujoco.mjtObj.mjOBJ_GEOM and geom.objid in hidden_proxies:
                    geom.rgba[3] = 0
            if summary["mode"] == "sand":
                # Visual only: native rigid floor is hidden in the render copy.
                for i in range(renderer.scene.ngeom):
                    geom = renderer.scene.geoms[i]
                    if geom.objtype == mujoco.mjtObj.mjOBJ_GEOM and geom.objid in support_ids(model,config["support_geoms"],allow_explicit_pairs=True):
                        geom.rgba[3] = 0
                geom = renderer.scene.geoms[renderer.scene.ngeom]
                mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_PLANE,np.array([5.,5.,.001]),
                    np.array([0.,0.,config["sand_height"]]),np.eye(3).ravel(),np.array([.76,.66,.44,.45],dtype=np.float32))
                renderer.scene.ngeom += 1
                for surface,bid in zip(surfaces,owners):
                    positions = surface.centers @ data.xmat[bid].reshape(3,3).T + data.xpos[bid]
                    for position in positions:
                        geom = renderer.scene.geoms[renderer.scene.ngeom]
                        mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_SPHERE,np.array([.0015,0.,0.]),position,
                            np.eye(3).ravel(),np.array([1.,.5,.05,1.],dtype=np.float32))
                        renderer.scene.ngeom += 1
            for position in trace[:index+1:max(1,round(.1/model.opt.timestep)),1:4]:
                geom = renderer.scene.geoms[renderer.scene.ngeom]
                mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_SPHERE,np.array([.003,0.,0.]),position,
                    np.eye(3).ravel(),np.array([.1,.8,.9,1.],dtype=np.float32)); renderer.scene.ngeom += 1
            frame = renderer.render()
            # White caption bands remain legible on dark imported URDF scenes.
            frame[:66] = 255; frame[508:] = 255
            text = f"{config['controller']['family']} | {summary['mode']} | t={trace[index,0]:.2f}s | dx={trace[index,1]-trace[0,1]:.2f}m"
            cv2.putText(frame,text,(15,28),cv2.FONT_HERSHEY_SIMPLEX,.6,(30,30,30),2,cv2.LINE_AA)
            cv2.putText(frame,f"{summary['status']} | {summary['termination']}",(15,53),cv2.FONT_HERSHEY_SIMPLEX,.43,(30,30,30),1,cv2.LINE_AA)
            cv2.putText(frame,"Independent controller | declared contact proxies | uncalibrated RFT" ,(15,520),cv2.FONT_HERSHEY_SIMPLEX,.45,(30,30,30),1,cv2.LINE_AA)
            writer.append_data(frame)
    finally:
        writer.close(); renderer.close()
    return len(indices)
