"""D-022: saved-state rigid/sand pairs and exact contact-proxy remesh reviews.

No mj_step, controller update or remeshing is performed. Source physics and
qualified bundles remain immutable. All native work is pose-only rendering.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import shutil
import sys

import cv2
import imageio.v2 as imageio
import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rft.records import ROOT,artifact_manifest,git_state,new_run,sha256,write_json
from scripts.review_ground_pipeline import verify,video_review

ROBOTS = ("go1","g1","berkeley","t1","apollo","turtlebot3_burger","turtlebot3_waffle_pi")
NAMES = {"go1":"Unitree Go1","g1":"Unitree G1","berkeley":"Berkeley Humanoid",
         "t1":"Booster T1","apollo":"Apptronik Apollo",
         "turtlebot3_burger":"TurtleBot3 Burger","turtlebot3_waffle_pi":"TurtleBot3 Waffle Pi",
         "g1_fast_failure":"Unitree G1 - fast-command negative control"}
INK = (42,49,59)
TEAL = (132,112,0)
BLUE = (173,101,42)
RED = (65,65,192)


def label(image,text,xy,scale=.48,color=INK,thickness=1):
    cv2.putText(image,str(text),tuple(map(int,xy)),cv2.FONT_HERSHEY_SIMPLEX,
                scale,color,thickness,cv2.LINE_AA)


def quadrature(vertices,faces):
    vertices,faces = np.asarray(vertices,dtype=float),np.asarray(faces)
    if vertices.ndim != 2 or vertices.shape[1] != 3 or faces.ndim != 2 or faces.shape[1] != 3:
        raise ValueError("Expected XYZ vertices and triangle indices")
    if faces.dtype.kind not in "iu" or len(faces)<2 or np.any(faces<0) or np.any(faces>=len(vertices)):
        raise ValueError("Expected at least two valid indexed triangles")
    triangles = vertices[faces]
    cross = np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    lengths = np.linalg.norm(cross,axis=1)
    if not np.all(np.isfinite(vertices)) or np.any(lengths<=1e-16):
        raise ValueError("Nonfinite or degenerate comparison mesh")
    return triangles.mean(axis=1),cross/lengths[:,None],lengths/2


def spacing_stats(centers):
    centers = np.asarray(centers,dtype=float)
    if len(centers)<2 or not np.all(np.isfinite(centers)):
        raise ValueError("Need finite 3D centroids")
    distances = cKDTree(centers).query(centers,k=2,workers=1)[0][:,1]
    median = float(np.median(distances))
    if np.any(distances<=0) or median<=0:
        raise ValueError("Coincident triangle centroids: review geometry rather than hide density")
    # Same dimensionless color rule, explicitly normalized within EACH mesh.
    # Absolute median spacing remains a separate reported physical quantity.
    normalized = np.clip((np.log2(distances/median)+1)/2,0,1)
    colors = cv2.applyColorMap(np.rint(255*normalized).astype(np.uint8)[:,None],cv2.COLORMAP_TURBO)[:,0]
    return colors,{"median_spacing_mm":median*1000,
        "spacing_p95_p05":float(np.percentile(distances,95)/np.percentile(distances,5))}


def camera_basis(azimuth,elevation):
    a,e = np.deg2rad([azimuth,elevation])
    return np.array([[-np.sin(a),np.cos(a),0],
        [-np.cos(a)*np.sin(e),-np.sin(a)*np.sin(e),np.cos(e)],
        [np.cos(a)*np.cos(e),np.sin(a)*np.cos(e),np.sin(e)]])


def geometry_frame(vertices_a,vertices_b):
    points = np.vstack([vertices_a,vertices_b])
    center = (points.min(axis=0)+points.max(axis=0))/2
    radius = max(float(np.linalg.norm(points-center,axis=1).max()),1e-9)
    return center,radius


def project(points,center,radius,rect,basis):
    x,y,w,h = rect
    local = (np.asarray(points)-center)@basis.T
    scale = .81*min(w,h)/(2*radius)
    pixels = np.column_stack([x+w/2+local[:,0]*scale,y+h/2-local[:,1]*scale])
    return np.rint(pixels).astype(np.int32),local[:,2]


def draw_surface(canvas,surface,kind,rect,azimuth,elevation):
    before = kind.startswith("source")
    mesh = surface["before_mesh" if before else "after_mesh"]
    vertices,faces,centers,normals,colors = (mesh[k] for k in ("vertices","faces","centers","normals","colors"))
    basis = camera_basis(azimuth,elevation)
    center,radius = surface["frame"]
    pixels,depth = project(vertices,center,radius,rect,basis)
    point_pixels,point_depth = project(centers,center,radius,rect,basis)
    x,y,w,h = rect
    cv2.rectangle(canvas,(x,y),(x+w,y+h),(244,246,247),-1)
    if kind.endswith("mesh"):
        projected = pixels[faces]
        shade = .45+.55*np.maximum(0,normals@np.array([.3,-.4,.866]))
        for index in np.argsort(depth[faces].mean(axis=1)):
            value = int(190+48*shade[index])
            cv2.fillConvexPoly(canvas,projected[index],(value,value,value),cv2.LINE_AA)
            cv2.polylines(canvas,[projected[index]],True,(132,143,150),1,cv2.LINE_AA)
        if not before:
            selected = np.unique(np.rint(np.linspace(0,len(centers)-1,min(14,len(centers)))).astype(int))
            ends,_ = project(centers[selected]+normals[selected]*radius*.14,center,radius,rect,basis)
            for index,end in zip(selected,ends):
                cv2.arrowedLine(canvas,tuple(point_pixels[index]),tuple(end),TEAL,1,cv2.LINE_AA,tipLength=.25)
    else:
        # Both front and rear centroids are deliberately shown, with rear points
        # faded. Counts/spacing are computed in 3D, never from screen pixels.
        for index in np.argsort(point_depth):
            color = colors[index].astype(float)
            if point_depth[index]<0: color = .48*color+.52*np.array([244,246,247])
            cv2.circle(canvas,tuple(point_pixels[index]),2,tuple(map(int,color)),-1,cv2.LINE_AA)
    cv2.rectangle(canvas,(x,y),(x+w,y+h),(208,216,220),1)


def proxy_metrics(surface):
    result = {}
    for which in ("before","after"):
        mesh = surface[which+"_mesh"]
        _,spacing = spacing_stats(mesh["centers"])
        a = mesh["areas"]
        result[which] = {"triangles_and_centroids":len(a),"area_m2":float(a.sum()),
            "area_p95_p05":float(np.percentile(a,95)/np.percentile(a,5)),**spacing}
    return result


def zoom_row(canvas,surface,origin,size,azimuth,elevation):
    x,y = origin; width,height = size
    name = surface["id"]+" | body="+surface["body"]
    label(canvas,name,(x+8,y+23),.49,INK,1)
    label(canvas,f"view elev {elevation:+.0f} deg | fixed shared scale | metres -> mm in metrics",(x+8,y+45),.36)
    columns = (("SOURCE TRIANGLES","source_mesh"),("REMESH + NORMALS","after_mesh"),
               ("SOURCE CENTROIDS","source_points"),("REMESH CENTROIDS","after_points"))
    cell = width//4
    plot_height = height-130
    for i,(title,kind) in enumerate(columns):
        left = x+i*cell+6
        label(canvas,title,(left+4,y+65),.43,BLUE if i in (0,2) else TEAL,1)
        draw_surface(canvas,surface,kind,(left,y+76,cell-12,plot_height),azimuth,elevation)
    m = surface["metrics"]
    for i,which in enumerate(("before","after","before","after")):
        left = x+i*cell+10; item = m[which]
        label(canvas,f"N={item['triangles_and_centroids']}  area ratio={item['area_p95_p05']:.2f}",(left,y+height-32),.36)
        label(canvas,f"spacing ratio={item['spacing_p95_p05']:.2f} | med={item['median_spacing_mm']:.2f} mm",(left,y+height-12),.33)


def geometry_sheet(title,surfaces,context,azimuth=35,selected=None):
    canvas = np.full((960,1600,3),255,dtype=np.uint8)
    label(canvas,title+" | CONTACT PROXY / REMESH REVIEW",(24,35),.85,INK,2)
    label(canvas,"Saved qualified geometry; source = selected collision proxy, NOT rejected whole-body CAD",(24,66),.52)
    cv2.line(canvas,(20,85),(1580,85),(210,218,222),1)
    canvas[105:505,20:380] = cv2.resize(context,(360,400))
    label(canvas,"INITIAL POSE - LOCATION CONTEXT",(22,526),.43,TEAL,1)
    label(canvas,"Grey: unchanged robot. Gold: all RFT sites.",(22,550),.36)
    label(canvas,"SCOPE: selected proxies only",(22,588),.51,INK,1)
    y = 614
    for s in surfaces:
        label(canvas,s["id"],(24,y),.39)
        label(canvas,f"  {s['before']['triangles']} -> {s['after']['triangles']} triangles / sites",(24,y+19),.35)
        y += 46
    if y<852:
        label(canvas,"No contact/policy accuracy claim.",(24,min(866,y+18)),.36)
    if selected is None:
        representatives = list(dict.fromkeys(s["family"] for s in surfaces))
        choices = [next(s for s in surfaces if s["family"]==f) for f in representatives]
        if len(choices)>2:
            raise ValueError("More than two proxy families: generate explicit review pages")
    else:
        choices = [selected]
    if len(choices)==1:
        rows = ((choices[0],25),(choices[0],-75))
    else:
        rows = ((choices[0],25),(choices[1],25))
    for i,(s,elevation) in enumerate(rows):
        zoom_row(canvas,s,(400,98+i*374),(1180,368),azimuth,elevation)
    # Shared dimensionless rule, not a misleading absolute-density comparison.
    bar = cv2.applyColorMap(np.linspace(0,255,260,dtype=np.uint8)[None],cv2.COLORMAP_TURBO)
    canvas[877:895,430:690] = np.repeat(bar,18,axis=0)
    label(canvas,"0.5x dense       1x median       2x sparse",(430,915),.36)
    label(canvas,"Color = local 3D nearest-centroid spacing / THAT mesh's median",(730,892),.43)
    label(canvas,"Absolute medians above; smaller triangles are not automatically more accurate.",(730,916),.38)
    label(canvas,"Same pose, view, scale and normalization rule | arrows = sampled outward normals, not forces",(24,947),.43)
    return canvas


def contact_bins(time,contact,width,duration):
    """Conservative OR within display bins; preserve brief contact events."""
    time,contact = np.asarray(time),np.asarray(contact,dtype=bool)
    if len(time)!=len(contact) or duration<=0 or width<1:
        raise ValueError("Invalid timeline display dimensions")
    edges = np.linspace(0,duration,width+1)
    locations = np.clip(np.searchsorted(edges,time,side="right")-1,0,width-1)
    result = np.zeros(width,dtype=np.uint8)
    np.maximum.at(result,locations,contact.astype(np.uint8))
    valid = edges[:-1]<=time[-1]
    return result,valid


def row_source_index(frame_index,frame_count):
    """Return None after source ends; never clamp to/freeze the last frame."""
    return frame_index if 0<=frame_index<frame_count else None


def frame_sample_indices(rows,frames):
    if rows<1 or frames<1:
        raise ValueError("Empty recorded sequence")
    return np.rint(np.linspace(0,rows-1,frames)).astype(int)


def load_motion(folder):
    verify(folder)
    summary = json.loads((folder/"summary.json").read_text())
    if summary["source"]["dirty"]:
        raise ValueError("Canonical comparisons require clean recorded simulation source")
    with np.load(folder/"trajectory.npz",allow_pickle=False) as data:
        trace,active,rigid,groups = (data[k].copy() for k in ("trace","active","rigid_active","groups"))
    if not all(len(a)==len(trace) and np.all(np.isfinite(a)) for a in (trace,active,rigid)):
        raise ValueError("Nonfinite/unaligned recorded telemetry")
    if summary["trace_columns"][6] != "penetration_m" or summary["trace_columns"][15] != "rigid_min_distance_m":
        raise ValueError("Unknown trace schema")
    cap = cv2.VideoCapture(str(folder/"locomotion.mp4"))
    frames,fps = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)),float(cap.get(cv2.CAP_PROP_FPS)); cap.release()
    if frames!=summary["video_frames"] or fps<=0:
        raise ValueError("Video metadata disagrees with original summary")
    return {"folder":folder,"summary":summary,"trace":trace,"contact":(active>0)|(rigid>0),
        "groups":groups,"frames":frames,"fps":fps,"indices":frame_sample_indices(len(trace),frames)}


def motion_row(frame,item,index,global_time,duration,yrange):
    canvas = np.full((410,1280,3),255,dtype=np.uint8)
    summary,trace = item["summary"],item["trace"]
    sand = summary["mode"]=="sand"
    name = "SAND / RFT" if sand else "RIGID FLAT / MATCHED TIMESTEP"
    color = TEAL if summary["status"]=="passed_screen" else RED
    label(canvas,name,(16,27),.65,color,2)
    label(canvas,f"{summary['status']} | dt={summary['physics_timestep_s']*1000:.2f} ms",(410,26),.40,color)
    if frame is not None:
        # Remove legacy repeated captions, keep the exact saved scene pixels.
        crop = frame[66:508] if frame.shape[:2]==(540,960) else frame
        canvas[42:410,:800] = cv2.resize(crop,(800,368))
    else:
        canvas[42:410,:800] = (234,238,241)
        label(canvas,"NO RECORDED MOTION AFTER STOP",(96,202),.75,RED,2)
        label(canvas,f"Stopped at {summary['recorded_duration_s']:.4f} s; no frozen or invented robot frames",(64,241),.45)
    source_index = item["indices"][index] if index is not None else len(trace)-1
    current = trace[source_index]; source_time = float(current[0])
    label(canvas,f"{'Row time' if index is not None else 'Final recorded'}: {source_time:.4f} s",(820,32),.48)
    label(canvas,f"COM dx={current[1]-trace[0,1]:+.3f} m  dy={current[2]-trace[0,2]:+.3f} m",(820,58),.43)
    if sand:
        depth = current[6]*1000; text = f"RFT centroid depth={depth:.2f} mm"
    else:
        text = f"Native support overlap={max(0,-current[15])*1000:.2f} mm"
    label(canvas,text,(820,84),.43)
    label(canvas,f"Root height={current[4]:.3f} m | upright cos={current[5]:.3f}",(820,108),.40)
    x,y,w,h = 850,148,398,86
    lo,hi = yrange
    def curve(t,values):
        return np.column_stack([x+np.asarray(t)/duration*w,y+h-(np.asarray(values)-lo)/(hi-lo)*h]).astype(np.int32)
    cv2.rectangle(canvas,(x,y),(x+w,y+h),(210,218,222),1)
    label(canvas,f"COM dx vs time | [{lo:.2f}, {hi:.2f}] m",(820,137),.38)
    all_curve = curve(trace[:,0],trace[:,1]-trace[0,1])
    cv2.polylines(canvas,[all_curve],False,(213,220,225),1,cv2.LINE_AA)
    cv2.polylines(canvas,[all_curve[:source_index+1]],False,color,2,cv2.LINE_AA)
    cursor = min(x+w,max(x,int(x+global_time/duration*w)))
    cv2.line(canvas,(cursor,y),(cursor,y+h),(80,80,80),1)
    label(canvas,"0 s",(x,y+h+17),.32); label(canvas,f"{duration:g} s",(x+w-25,y+h+17),.32)
    label(canvas,"COMPONENT CONTACT: 0 white / 1 dark",(820,278),.39)
    for i,group in enumerate(item["groups"]):
        yy = 289+i*21
        label(canvas,str(group),(818,yy+13),.31)
        bits,valid = contact_bins(trace[:,0],item["contact"][:,i],342,duration)
        rgb = np.full((16,342,3),245,dtype=np.uint8); rgb[:,bits.astype(bool)] = (48,68,76)
        rgb[:,~valid] = (205,214,221)
        canvas[yy:yy+16,912:1254] = rgb
        cv2.line(canvas,(min(1253,int(912+342*global_time/duration)),yy),
                 (min(1253,int(912+342*global_time/duration)),yy+15),RED,1)
    label(canvas,"Each pixel: any contact in its time bin; grey = no data",(820,397),.31)
    return canvas


def compose_motion_frame(title,items,frames,indices,global_time,duration,yrange):
    canvas = np.full((944,1280,3),255,dtype=np.uint8)
    label(canvas,title+" | RIGID ABOVE / SAND BELOW",(20,34),.77,INK,2)
    cmd = items[0]["summary"]["effective_controller"]["command"]
    label(canvas,f"Shared output time {global_time:.2f} s | command vx={cmd[0]:g} m/s | saved trajectories, zero new physics steps",(20,63),.47)
    canvas[82:492] = motion_row(frames[0],items[0],indices[0],global_time,duration,yrange)
    canvas[502:912] = motion_row(frames[1],items[1],indices[1],global_time,duration,yrange)
    label(canvas,"Orange = RFT centroids; cyan = COM history | contact proxies/material uncalibrated | failed rows remain failures",(20,936),.40)
    return canvas


def compose_motion(title,folders,output):
    items = [load_motion(f) for f in folders]
    if abs(items[0]["fps"]-items[1]["fps"])>1e-9:
        raise ValueError("Different source FPS requires an explicit alignment adapter")
    if items[0]["summary"]["effective_controller"] != items[1]["summary"]["effective_controller"]:
        raise ValueError("Rows do not share controller parameters")
    if items[0]["summary"]["physics_timestep_s"] != items[1]["summary"]["physics_timestep_s"]:
        raise ValueError("Use matched-timestep rigid, not upstream timestep")
    fps = items[0]["fps"]; count = max(i["frames"] for i in items)
    duration = max(i["summary"]["requested_duration_s"] for i in items)
    values = np.concatenate([i["trace"][:,1]-i["trace"][0,1] for i in items])
    margin = max(.02,float(np.ptp(values))*.08)
    yrange = float(values.min()-margin),float(values.max()+margin)
    caps = [cv2.VideoCapture(str(i["folder"]/"locomotion.mp4")) for i in items]
    writer = imageio.get_writer(str(output/"motion_comparison.mp4"),fps=fps,codec="libx264",
        pixelformat="yuv420p",macro_block_size=2,output_params=["-movflags","+faststart"])
    samples = {}; transitions = {}
    try:
        for index in range(count):
            rows,indices = [],[]
            for cap,item in zip(caps,items):
                selected = row_source_index(index,item["frames"])
                if selected is None: frame = None
                else:
                    ok,frame = cap.read()
                    if not ok: raise ValueError("Video decode ended before its declared frame count")
                rows.append(frame); indices.append(selected)
            frame = compose_motion_frame(title,items,rows,indices,index/fps,duration,yrange)
            writer.append_data(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
            if index in (0,count//2,count-1): samples[index] = cv2.resize(frame,(640,472))
            if index==count//2: cv2.imwrite(str(output/"motion_comparison.png"),frame)
            if any(index==i["frames"] and index<count for i in items):
                cv2.imwrite(str(output/f"stop_transition_{index}.png"),frame)
                transitions[str(index)] = index/fps
    finally:
        writer.close()
        for cap in caps: cap.release()
    metrics,_ = video_review(output/"motion_comparison.mp4",count)
    cv2.imwrite(str(output/"motion_contact_sheet.png"),np.hstack([samples[k] for k in sorted(samples)]))
    return {"video":metrics,"rows":[{"directory":str(i["folder"]),"summary":i["summary"],
        "video_sha256":sha256(i["folder"]/"locomotion.mp4"),
        "trajectory_sha256":sha256(i["folder"]/"trajectory.npz"),"real_frames":i["frames"],
        "visual_replay":json.loads((i["folder"]/"replay.json").read_text()) if (i["folder"]/"replay.json").exists() else None,
        "frame_physical_times_s":i["trace"][i["indices"],0].tolist()} for i in items],
        "after_stop":"neutral no-data scene; no last-frame duplication", "transitions":transitions,
        "physics_steps_executed":0,"contact_display":"conservative OR per time pixel; raw telemetry unchanged",
        "depth_definitions":{"rigid":"max(0,-native support contact.dist)","sand":"max triangle-centroid penetration"}}


def context_image(model,config,surfaces):
    import mujoco
    from rft.ground import render_hidden_proxy_ids,support_ids
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key(config["initial_keyframe"]).id)
    mujoco.mj_forward(model,data)
    model.vis.global_.offwidth = model.vis.global_.offheight = 720
    camera = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(camera)
    for key,value in config["camera"].items(): setattr(camera,key,value)
    # Auto-frame location context from native robot geoms, not the world floor.
    gids = np.flatnonzero(model.geom_bodyid>0)
    radii = model.geom_rbound[gids,None]
    lower = np.min(data.geom_xpos[gids]-radii,axis=0)
    upper = np.max(data.geom_xpos[gids]+radii,axis=0)
    camera.lookat[:] = (lower+upper)/2
    radius = max(.04,float(np.linalg.norm(upper-lower)/2))
    camera.distance = 2.8*radius
    hidden = render_hidden_proxy_ids(model)|support_ids(model,config["support_geoms"],allow_explicit_pairs=True)
    renderer = mujoco.Renderer(model,height=720,width=720,max_geom=10000+sum(len(s.areas) for s in surfaces))
    option = mujoco.MjvOption(); option.geomgroup[3] = 0
    try:
        renderer.update_scene(data,camera=camera,scene_option=option)
        for i in range(renderer.scene.ngeom):
            geom = renderer.scene.geoms[i]
            if geom.objtype != mujoco.mjtObj.mjOBJ_GEOM: continue
            if geom.objid in hidden: geom.rgba[3] = 0
            elif geom.objid>=0:
                geom.matid = -1; geom.rgba[:] = [.60,.66,.72,.25]
        if config.get("effective_model_adapter",{}).get("source_format") in ("urdf","xacro"):
            # Display-only backdrop for dark URDF scenes, not a native support.
            forward = np.mean([c.forward for c in renderer.scene.camera],axis=0)
            forward /= np.linalg.norm(forward)
            up = np.mean([c.up for c in renderer.scene.camera],axis=0)
            z = -forward; x = np.cross(up,z); x /= np.linalg.norm(x); y = np.cross(z,x)
            geom = renderer.scene.geoms[renderer.scene.ngeom]
            mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_PLANE,np.array([radius*10,radius*10,.01]),
                camera.lookat+forward*radius*3,np.column_stack([x,y,z]).ravel(),np.array([1.,1.,1.,1.],dtype=np.float32))
            renderer.scene.ngeom += 1
        for surface in surfaces:
            bid = model.body(surface.body_name).id
            points = surface.centers@data.xmat[bid].reshape(3,3).T+data.xpos[bid]
            for point in points:
                geom = renderer.scene.geoms[renderer.scene.ngeom]
                radius = max(.001,min(.005,camera.distance*.0015))
                mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_SPHERE,np.array([radius,0,0]),point,
                    np.eye(3).ravel(),np.array([1.,.57,.04,1.],dtype=np.float32))
                renderer.scene.ngeom += 1
        frame = cv2.cvtColor(renderer.render(),cv2.COLOR_RGB2BGR)
    finally:
        renderer.close()
    return frame,data


def load_review_surfaces(model,data,bundle,metadata):
    from rft.geometry import read_mesh,mesh_arrays
    result = []
    for report in metadata["surfaces"]:
        folder = bundle/report["id"]
        source_v,source_f = mesh_arrays(read_mesh(folder/"source.ply","m"))
        with np.load(bundle/report["arrays"],allow_pickle=False) as arrays:
            target_v,target_f = arrays["vertices"].copy(),arrays["faces"].copy()
            c,n,a = quadrature(target_v,target_f)
            if not all(np.array_equal(value,arrays[key]) for value,key in ((c,"centers"),(n,"normals"),(a,"areas"))):
                raise ValueError("Recorded quadrature changed")
        record = dict(report)
        rotation = data.xmat[model.body(report["body"]).id].reshape(3,3)
        for which,v,f in (("before",source_v,source_f),("after",target_v,target_f)):
            centers,normals,areas = quadrature(v,f)
            if len(f)!=report[which]["triangles"] or not np.isclose(areas.sum(),report[which]["area_m2"],rtol=1e-11,atol=1e-14):
                raise ValueError("Snapshot disagrees with qualified metrics")
            colors,_ = spacing_stats(centers)
            record[which+"_mesh"] = {"vertices":v@rotation.T,"faces":f,
                "centers":centers@rotation.T,"normals":normals@rotation.T,"areas":areas,"colors":colors}
        record["frame"] = geometry_frame(record["before_mesh"]["vertices"],record["after_mesh"]["vertices"])
        record["metrics"] = proxy_metrics(record)
        if report["source"].get("geom"):
            gid = model.geom(report["source"]["geom"]).id
            family = [int(model.geom_type[gid]),np.round(model.geom_size[gid],12).tolist(),
                      report["before"]["triangles"],report["after"]["triangles"]]
        else: family = [report["id"]]
        record["family"] = json.dumps(family)
        record["snapshot_hashes"] = {"source_ply":sha256(folder/"source.ply"),
            "prepared_ply":sha256(folder/"prepared.ply"),"surface_npz":sha256(bundle/report["arrays"])}
        result.append(record)
    return result


def public_surface_record(surface):
    return {key:value for key,value in surface.items() if key not in ("before_mesh","after_mesh","frame")}


def generate_geometry(title,surfaces,context,output,duration,fps):
    cv2.imwrite(str(output/"proxy_locations.png"),context)
    for surface in surfaces:
        image = geometry_sheet(title,surfaces,context,selected=surface)
        if not cv2.imwrite(str(output/("proxy_"+surface["id"]+".png")),image):
            raise OSError("Proxy zoom write failed")
    writer = imageio.get_writer(str(output/"remesh_comparison.mp4"),fps=fps,codec="libx264",
        pixelformat="yuv420p",macro_block_size=2,output_params=["-movflags","+faststart"])
    count = round(duration*fps)+1
    try:
        for i in range(count):
            frame = geometry_sheet(title,surfaces,context,azimuth=35+360*i/(count-1))
            writer.append_data(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
            if i==0: cv2.imwrite(str(output/"remesh_comparison.png"),frame)
    finally:
        writer.close()
    metrics,_ = video_review(output/"remesh_comparison.mp4",count)
    report = {"video":metrics,"surfaces":[public_surface_record(s) for s in surfaces],
        "color_rule":"clip(log2(3D nearest-centroid spacing / own-mesh median), -1, +1)",
        "whole_robot_remeshed":False,"physics_steps_executed":0,"remesh_operations_executed":0,
        "views":"same initial world orientation, shared before/after bounds, oblique plus underside where one family",
        "normals":"sampled outward unit geometric normals; visualization arrows, not forces"}
    write_json(output/"geometry.json",report)
    with (output/"proxy_metrics.csv").open("w",encoding="utf-8",newline="") as stream:
        fields = ("surface","body","stage","triangles_and_centroids","area_m2","area_p95_p05","median_spacing_mm","spacing_p95_p05")
        writer = csv.DictWriter(stream,fieldnames=fields,lineterminator="\n"); writer.writeheader()
        for surface in surfaces:
            for stage in ("before","after"):
                writer.writerow({"surface":surface["id"],"body":surface["body"],"stage":stage,**surface["metrics"][stage]})
    return report


def replay_visual_only(run,config_path,scenario,output):
    from rft.pipeline import import_robot,load_bundle
    from rft.ground import render_run
    verify(run); source = run/scenario; verify(source)
    ledger = json.loads((run/"pipeline.json").read_text())
    summary = json.loads((source/"summary.json").read_text())
    output.mkdir(parents=True,exist_ok=False)
    model,config,_ = import_robot(config_path,medium=summary["mode"])
    surfaces,_ = load_bundle(model,ledger["bundle_directory"])
    model.opt.timestep = summary["physics_timestep_s"]
    shutil.copy2(source/"trajectory.npz",output/"trajectory.npz")
    write_json(output/"summary.json",summary)
    frames = render_run(model,config,surfaces,output)
    if frames!=summary["video_frames"] or sha256(output/"trajectory.npz")!=sha256(source/"trajectory.npz"):
        raise AssertionError("Visual replay changed state bytes or frame selection")
    write_json(output/"replay.json",{"render_source":git_state(),"simulation_source":summary["source"],
        "source_directory":str(source.resolve()),"trajectory_sha256":sha256(source/"trajectory.npz"),
        "physics_steps_executed":0,"changes":"display-only original URDF visuals and readable captions"})
    write_json(output/"manifest.json",{"artifacts":artifact_manifest(output)})
    return output


def copy_notices(robot,folder):
    from rft.catalog import catalog,entry,selection
    definitions = catalog()
    for source_name in selection(robot):
        base = ROOT/definitions["sources"][source_name]["destination"]
        name = entry(robot).get("menagerie_model")
        original = base/name/"LICENSE" if source_name=="menagerie" else base/"LICENSE"
        (folder/("LICENSE-"+source_name+".txt")).write_bytes(original.read_bytes())
    (folder/"NOTICE.playground").write_bytes((ROOT/"controllers/NOTICE.playground").read_bytes())


def render_catalog(args):
    from rft.catalog import config_path,verify_installed
    from rft.pipeline import import_robot,load_bundle
    from rft.records import git_state
    source = git_state()
    if source["dirty"] and not args.allow_dirty:
        raise ValueError("Commit reporting source before canonical generation (or explicit development --allow-dirty)")
    output = new_run(args.output_dir); batch = args.batch.resolve(); verify(batch)
    report = {"source":source,"generator_sha256":sha256(__file__),"robots":{},
        "physics_steps_executed":0,"remesh_operations_executed":0,"batch_manifest_sha256":sha256(batch/"manifest.json")}
    for robot in args.robots:
        print(f"COMPARISON {robot}: verify -> geometry/proxy zooms -> paired motion",flush=True)
        verify_installed(robot)
        run = batch/robot; verify(run)
        ledger = json.loads((run/"pipeline.json").read_text())
        model,config,_ = import_robot(config_path(robot))
        expected = next(s["config_sha256"] for s in ledger["steps"] if s["stage"]=="import")
        if sha256(config_path(robot))!=expected:
            raise ValueError("Recipe changed since saved simulation; select a matching source/config")
        bundle = Path(ledger["bundle_directory"])
        prepared,metadata = load_bundle(model,bundle)
        destination = output/robot; destination.mkdir()
        context,data = context_image(model,config,prepared)
        surfaces = load_review_surfaces(model,data,bundle,metadata)
        geometry = generate_geometry(NAMES[robot],surfaces,context,destination,args.geometry_duration,args.geometry_fps)
        folders = [run/"rigid_matched",run/"sand"]
        # Only URI-imported models need fresh display-only replay; all source
        # states, native collisions, bundle arrays and prior MP4s remain intact.
        if robot.startswith("turtlebot3_"):
            folders = [replay_visual_only(run,config_path(robot),scenario,destination/("replay_"+scenario))
                       for scenario in ("rigid_matched","sand")]
        motion = compose_motion(NAMES[robot],folders,destination)
        copy_notices(robot,destination)
        record = {"geometry":geometry,"motion":motion,"recipe":str(config_path(robot)),
            "config_sha256":expected,"simulation_source":ledger["source"],
            "bundle_manifest_sha256":sha256(bundle/"manifest.json"),"bundle_directory":str(bundle)}
        write_json(destination/"comparison.json",record)
        write_json(destination/"manifest.json",{"artifacts":artifact_manifest(destination)})
        report["robots"][robot] = record
    if args.fast_run:
        run = args.fast_run.resolve(); verify(run)
        destination = output/"g1_fast_failure"; destination.mkdir()
        motion = compose_motion(NAMES["g1_fast_failure"],[run/"rigid_matched",run/"sand"],destination)
        write_json(destination/"comparison.json",{"motion":motion,"geometry_shared_with":"g1"})
        write_json(destination/"manifest.json",{"artifacts":artifact_manifest(destination)})
        report["g1_fast_failure"] = {"motion":motion,"geometry_shared_with":"g1"}
    # Re-verify immutable source evidence after all replay/composition work.
    verify(batch)
    for robot in args.robots:
        verify(batch/robot)
        verify(Path(report["robots"][robot]["bundle_directory"]))
    write_json(output/"comparison_index.json",report)
    write_json(output/"manifest.json",{"artifacts":artifact_manifest(output)})
    print(f"COMPLETE: {output}",flush=True)


def publish(args):
    raw = args.publish.resolve(); verify(raw)
    index = json.loads((raw/"comparison_index.json").read_text())
    if index["source"]["dirty"]:
        raise ValueError("Do not publish dirty-source development media")
    output = new_run(args.media_dir)
    copied = {}
    for robot in (*index["robots"],*(["g1_fast_failure"] if "g1_fast_failure" in index else [])):
        folder = raw/robot; verify(folder); target = output/robot; target.mkdir()
        patterns = ("motion_comparison.mp4","remesh_comparison.mp4","*.png","*.csv","*.json","LICENSE-*.txt","NOTICE.playground")
        files = {p for pattern in patterns for p in folder.glob(pattern) if p.is_file() and p.name!="manifest.json"}
        for path in files:
            shutil.copy2(path,target/path.name)
            copied[f"{robot}/{path.name}"] = {"source_sha256":sha256(path),"sha256":sha256(target/path.name)}
        write_json(target/"manifest.json",{"artifacts":artifact_manifest(target)})
    write_json(output/"comparison_index.json",index)
    write_json(output/"publication.json",{"source":git_state(),"raw":str(raw),"copies":copied})
    lines = ["# Rigid above / sand below, plus enlarged contact-proxy remesh reviews","",
        "All movies reuse saved physics. Geometry source is the selected contact proxy, not the whole robot CAD. "
        "No missing failure motion is fabricated; early-stop scene rows become labelled no-data cards.","",
        "Motion right panels show mass-weighted COM displacement, current overlap/centroid depth and component 0/1 contact. "
        "Rigid overlap and RFT centroid penetration are different explicitly labelled quantities. "
        "Contact display conservatively ORs each time-pixel bin; full raw telemetry remains unchanged.",""]
    for robot,record in index["robots"].items():
        lines += [f"## {NAMES[robot]}","",f"[Play rigid/sand comparison]({robot}/motion_comparison.mp4) | "
            f"[Rotate mesh/point comparison]({robot}/remesh_comparison.mp4)","",
            f"![Contact proxy mesh/point overview]({robot}/remesh_comparison.png)","",
            "Individual proxy enlargements:",""]
        lines += [f"- [{s['id']}: {s['body']}]({robot}/proxy_{s['id']}.png)" for s in record["geometry"]["surfaces"]]
        lines += ["",f"[Exact metrics CSV]({robot}/proxy_metrics.csv) | [Hashes and provenance]({robot}/comparison.json)",""]
    if "g1_fast_failure" in index:
        lines += ["## G1 faster-command negative control","",
            "[Rigid/sand comparison](g1_fast_failure/motion_comparison.mp4). Same proxy review as G1 above; different command. "
            "Do not count this as an eighth robot.",""]
    lines += ["## Interpretation and reproduction","",
        "Before/after clouds use one centroid per triangle, including all shell faces (not only active contact). "
        "Colors mean local 3D spacing divided by that mesh's own median: 0.5x dense, 1x median, 2x sparse. "
        "Absolute median spacing and actual counts are reported separately. Same camera/scale is used within each comparison. "
        "Sample arrows show outward normals, not force vectors. Source proxies can already be valid; remeshing is not proof "
        "of calibrated physics or improved walking. Whole-model images are pose-only ghost/location context; native geometry is unchanged.","",
        "Raw generator: `scripts/generate_robot_comparisons.py --batch outputs/robot_catalog/clean_6s_36659ca "
        "--fast-run outputs/ground_rft/g1_clean_6s_5078fdc --output-dir outputs/robot_comparisons/NEW_RUN`. "
        "Use `conda run -n lizard_rft python` before that script. See `docs/ROBOT_COMPARISONS_2026-10-28.md` for "
        "decisions, test commands, exact evidence and boundaries. Licenses/notices accompany each robot. Prior media remains untouched.",""]
    (output/"README.md").write_text("\n".join(lines),encoding="utf-8",newline="\n")
    video_files = list(output.rglob("*.mp4"))
    if len(video_files)>15 or sum(p.stat().st_size for p in output.rglob("*") if p.is_file())>64000000:
        raise ValueError("D-022 publication budget exceeded: review before tracking")
    write_json(output/"manifest.json",{"artifacts":artifact_manifest(output)})
    print(f"PUBLISHED {len(video_files)} MP4s: {output}")


def verify_comparisons(directory):
    """Independent full decode/hash and per-surface completeness readback."""
    directory = Path(directory).resolve(); verify(directory)
    index = json.loads((directory/"comparison_index.json").read_text())
    robots = index["robots"]; videos = frames = zooms = checked_states = missing_states = 0
    for name in (*robots,*(["g1_fast_failure"] if "g1_fast_failure" in index else [])):
        root = directory/name; verify(root)
        record = robots[name] if name in robots else index[name]
        motion = record["motion"]
        video_review(root/"motion_comparison.mp4",motion["video"]["frames_decoded"])
        frames += motion["video"]["frames_decoded"]; videos += 1
        if motion["after_stop"] != "neutral no-data scene; no last-frame duplication":
            raise ValueError("Unknown stopped-row semantics")
        for row in motion["rows"]:
            source = Path(row["directory"])
            if source.exists():
                verify(source)
                if sha256(source/"trajectory.npz")!=row["trajectory_sha256"]:
                    raise ValueError("Source state mutated")
                checked_states += 1
            else:
                missing_states += 1
        if name in robots:
            geometry = record["geometry"]
            video_review(root/"remesh_comparison.mp4",geometry["video"]["frames_decoded"])
            videos += 1; frames += geometry["video"]["frames_decoded"]
            for surface in geometry["surfaces"]:
                path = root/("proxy_"+surface["id"]+".png")
                image = cv2.imread(str(path))
                if image is None or image.shape[:2]!=(960,1600):
                    raise ValueError("Missing/invalid enlarged component comparison")
                zooms += 1
            if geometry["physics_steps_executed"] or geometry["remesh_operations_executed"]:
                raise ValueError("Unexpected physical/remesh experiment")
    result = {"robots":len(robots),"videos":videos,"frames_decoded":frames,"individual_proxy_zooms":zooms,
        "source_states_checked":checked_states,"source_states_unavailable":missing_states,"all_hashes_verified":True}
    print(json.dumps(result)); return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch",type=Path,default=ROOT/"outputs/robot_catalog/clean_6s_36659ca")
    parser.add_argument("--robots",nargs="+",choices=ROBOTS,default=list(ROBOTS))
    parser.add_argument("--fast-run",type=Path)
    parser.add_argument("--output-dir",type=Path)
    parser.add_argument("--geometry-duration",type=float,default=4)
    parser.add_argument("--geometry-fps",type=int,default=16)
    parser.add_argument("--allow-dirty",action="store_true",help="Development only, never publish")
    parser.add_argument("--publish",type=Path,help="Copy reviewed bounded artifacts from a completed raw generation")
    parser.add_argument("--media-dir",type=Path,default=ROOT/"docs/media/robot_comparisons")
    parser.add_argument("--verify",type=Path,help="Verify hashes, every frame and every component zoom")
    args = parser.parse_args()
    if args.verify: verify_comparisons(args.verify)
    elif args.publish: publish(args)
    else:
        if not args.output_dir: parser.error("--output-dir NEW_DIRECTORY required")
        if not .1<=args.geometry_duration<=10 or not 4<=args.geometry_fps<=30:
            parser.error("Bounded geometry duration 0.1..10 s and FPS 4..30 required")
        if round(args.geometry_duration*args.geometry_fps)<2:
            parser.error("Geometry review needs at least three decoded frames; increase duration or FPS")
        render_catalog(args)


if __name__ == "__main__":
    main()
