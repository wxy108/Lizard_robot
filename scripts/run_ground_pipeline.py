"""One entry point for import -> qualify/remesh -> verified binding -> walking."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rft.records import artifact_manifest, git_state, new_run, sha256, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",type=Path)
    parser.add_argument("--input",type=Path,help="Inspect raw robot/mesh input without inventing dynamics")
    parser.add_argument("--units",choices=("m","mm"))
    parser.add_argument("--output-dir",type=Path)
    parser.add_argument("--bundle-dir",type=Path)
    parser.add_argument("--prepare-only",action="store_true")
    parser.add_argument("--duration",type=float,default=6)
    parser.add_argument("--video",action="store_true")
    args = parser.parse_args()
    if (args.config is None) == (args.input is None):
        parser.error("Choose exactly one --config or --input")
    if not 0.1 <= args.duration <= 30:
        parser.error("Duration must be 0.1..30 seconds")
    output = new_run(args.output_dir or ROOT/"outputs/ground_rft"/(__import__("datetime").datetime.now().strftime("run_%Y%m%d_%H%M%S_%f")))
    ledger = {"source":git_state(),"status":"running","steps":[]}
    stage = "input"
    def mark(name,**payload):
        ledger["steps"].append({"stage":name,**payload})
        write_json(output/"pipeline.json",ledger)
        print(name,flush=True)
    try:
        from rft.pipeline import build_bundle, import_robot, inventory, load_bundle, verify_artifacts, verify_binding_poses
        if args.input:
            from rft.binding import load_model
            from rft.geometry import audit, read_mesh
            input_path = args.input.resolve()
            if not input_path.is_file():
                raise FileNotFoundError(input_path)
            if input_path.suffix.lower() in (".stl",".obj",".ply"):
                mesh = read_mesh(input_path,args.units)
                mark("geometry_inventory",source=str(input_path),sha256=sha256(input_path),metrics=audit(mesh))
                ledger.update(status="needs_metadata",required=["robot dynamics/joints","explicit units and body owner/exterior recipe","controller contract"])
            elif input_path.suffix.lower() in (".xml",".mjcf",".urdf"):
                model = load_model(input_path)
                write_json(output/"inventory.json",inventory(model))
                ledger.update(status="needs_recipe",required=["surface/exterior selection","initialization","controller","ground support policy"])
            else:
                ledger.update(status="unsupported_input",required=["explicit conversion to MJCF/URDF or supported STL/OBJ/PLY geometry"])
            write_json(output/"pipeline.json",ledger)
            write_json(output/"manifest.json",{"artifacts":artifact_manifest(output)})
            print(f"Stopped safely: {ledger['status']}; {output}")
            return 2
        stage = "import"
        model,config,sources = import_robot(args.config)
        write_json(output/"inventory.json",inventory(model))
        write_json(output/"resolved_config.json",config)
        mark("import",source_files=sources,config_sha256=sha256(args.config))
        stage = "prepare"
        bundle = args.bundle_dir.resolve() if args.bundle_dir else output/"bundle"
        if not args.bundle_dir:
            metadata = build_bundle(model,config,args.config,bundle,sources)
            mark("prepare",surfaces=[{"id":s["id"],"triangles":s["after"]["triangles"]} for s in metadata["surfaces"]])
        stage = "bind"
        surfaces,metadata = load_bundle(model,bundle)
        if metadata["config_sha256"] != sha256(args.config):
            raise ValueError("Bundle config differs; rebuild or explicitly review a new recipe")
        mark("bind",bundle=str(bundle),bundle_metadata_sha256=sha256(bundle/"bundle.json"),surface_count=len(surfaces))
        if config.get("initial_keyframe"):
            checks = verify_binding_poses(model,surfaces,config["initial_keyframe"])
            write_json(output/"binding_checks.json",checks)
            mark("binding_pose_checks",**checks)
        if args.prepare_only:
            ledger["status"] = "prepared"
        else:
            from rft.ground import plot_run, render_run, run
            results = {}
            for name,mode,dt in (("rigid_upstream","rigid",config["upstream_sim_timestep"]),
                                 ("rigid_matched","rigid",config["timestep"]),("sand","sand",config["timestep"])):
                stage = name
                # Reload original immutable model for every scenario. Rebind
                # and verify fingerprint before changing timestep/contact mode.
                m,c,_ = import_robot(args.config,medium=mode)
                prepared,_ = load_bundle(m,bundle)
                result = run(m,c,args.config,prepared,output/name,mode=mode,duration=args.duration,dt=dt)
                plot_run(output/name)
                if args.video:
                    result["video_frames"] = render_run(m,c,prepared,output/name)
                    write_json(output/name/"summary.json",result)
                write_json(output/name/"manifest.json",{"artifacts":artifact_manifest(output/name)})
                verify_artifacts(output/name)
                results[name] = result
                mark(name,status=result["status"],termination=result["termination"])
                if name.startswith("rigid") and result["status"] != "passed_screen":
                    ledger["status"] = "rigid_reference_failed"
                    break
            else:
                ledger["status"] = "passed_screen" if results["sand"]["status"] == "passed_screen" else "sand_transfer_failed"
            write_json(output/"results.json",results)
    except Exception as exc:
        ledger.update(status="rejected",failed_stage=stage,error=str(exc),exception=type(exc).__name__)
        print(f"Pipeline rejected at {stage}: {exc}",flush=True)
    write_json(output/"pipeline.json",ledger)
    write_json(output/"manifest.json",{"artifacts":artifact_manifest(output)})
    print(f"Pipeline {ledger['status']}: {output}",flush=True)
    return 0 if ledger["status"] in ("prepared","passed_screen") else 1


if __name__ == "__main__":
    raise SystemExit(main())
