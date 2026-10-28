"""Single catalog entry point. Run list/doctor/install before costly simulation."""
import argparse
import importlib.metadata
import json
from pathlib import Path
import sys
import subprocess

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rft.catalog import catalog,config_path,install,verify_installed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command",choices=("list","doctor","install","run","batch","inspect","pack"))
    parser.add_argument("robots",nargs="*")
    parser.add_argument("--duration",type=float,default=6)
    parser.add_argument("--video",action="store_true")
    parser.add_argument("--prepare-only",action="store_true")
    parser.add_argument("--output-dir",type=Path)
    parser.add_argument("--output",type=Path,help="New package filename for pack")
    parser.add_argument("--input",type=Path)
    args = parser.parse_args()
    if args.command == "list":
        for name,item in catalog()["robots"].items():
            print(f"{name:24s} {item['kind']:28s} {item['config']}")
        return 0
    if args.command == "doctor":
        print(f"Python: {sys.executable}; environment: {sys.prefix}")
        ready = True
        for name in ("mujoco","numpy","open3d","pymeshlab","onnxruntime","xacro"):
            try: print(f"{name}: {importlib.metadata.version(name)}")
            except importlib.metadata.PackageNotFoundError:
                ready = False
                print(f"{name}: missing; install requirements-locomotion.txt in lizard_rft")
        try:
            import mujoco
            mujoco.MjModel.from_xml_string("<mujoco/>")
            print("MuJoCo native import/compile: passed")
        except Exception as error:
            ready = False; print(f"MuJoCo native import/compile failed: {error}; launch through conda run -n lizard_rft")
        for robot in args.robots:
            try: print(f"{robot}: {verify_installed(robot)} source checks passed")
            except Exception as error:
                ready = False; print(f"{robot}: {error}")
        return 0 if ready else 1
    if args.command == "install":
        if not args.robots:
            parser.error("Choose robot IDs from list")
        install(args.robots)
        return 0
    if args.command == "pack":
        package_output = args.output or args.output_dir
        if len(args.robots) != 1 or not package_output:
            parser.error("pack ROBOT --output NEW_PACKAGE.zip")
        try: verify_installed(args.robots[0])
        except FileNotFoundError: install(args.robots)
        from rft.robot_package import export_package
        export_package(args.robots[0],package_output)
        return 0
    if args.command == "inspect":
        if not args.input: parser.error("inspect requires --input")
        from rft.onboarding import inspect_input
        inspect_input(args.input,args.output_dir)
        return 2
    if args.command in ("run","batch"):
        from rft.records import new_run,write_json,artifact_manifest,git_state
        output = new_run(args.output_dir or ROOT/"outputs/robot_catalog"/(args.command+"_"+__import__("datetime").datetime.now().strftime("%Y%m%d_%H%M%S_%f")))
        jobs = []
        if args.input:
            if args.robots: parser.error("Choose robot IDs or --input, not both")
            from rft.robot_package import import_package
            if args.input.suffix.lower() in (".zip",".rftpkg"):
                from rft.formats import check_archive
                if "robot_package.json" not in check_archive(args.input):
                    from rft.onboarding import inspect_input
                    inspect_input(args.input,output/"inspection")
                    return 2
                jobs = [("package",import_package(args.input))]
            else:
                from rft.onboarding import recognize_input,inspect_input
                robot = recognize_input(args.input)
                if robot:
                    print(f"Recognized exact catalog model: {robot}",flush=True)
                    args.robots = [robot]
                else:
                    inspect_input(args.input,output/"inspection")
                    return 2
        preflight_errors = []
        for robot in args.robots:
            try:
                try: verify_installed(robot)
                except FileNotFoundError: install([robot])
                jobs.append((robot,config_path(robot)))
            except Exception as error:
                preflight_errors.append({"robot":robot,"returncode":1,
                    "pipeline":{"status":"source_verification_failed","error":str(error)}})
        if not jobs and not preflight_errors: parser.error("Choose robot IDs from list or provide --input")
        results = list(preflight_errors)
        for robot,recipe in jobs:
            target = output/robot
            command = [sys.executable,"-u",str(ROOT/"scripts/run_ground_pipeline.py"),"--config",str(recipe),
                       "--duration",str(args.duration),"--cache","--output-dir",str(target)]
            if args.video: command.append("--video")
            if args.prepare_only: command.append("--prepare-only")
            print(f"Starting {robot}: {recipe}",flush=True)
            with (output/(robot+".log")).open("w",encoding="utf-8",newline="\n") as log:
                process = subprocess.Popen(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
                for line in process.stdout:
                    print(line,end="",flush=True); log.write(line); log.flush()
                code = process.wait()
            ledger = json.loads((target/"pipeline.json").read_text()) if (target/"pipeline.json").exists() else {"status":"process_failed"}
            results.append({"robot":robot,"returncode":code,"pipeline":ledger,"recipe":str(recipe)})
            write_json(output/"batch.json",{"source":git_state(),"results":results})
        write_json(output/"batch.json",{"source":git_state(),"results":results})
        write_json(output/"manifest.json",{"artifacts":artifact_manifest(output)})
        print(f"Catalog run evidence: {output}",flush=True)
        return 0 if all(item["returncode"] == 0 for item in results) else 1
    raise AssertionError("Unreachable command")


if __name__ == "__main__":
    raise SystemExit(main())
