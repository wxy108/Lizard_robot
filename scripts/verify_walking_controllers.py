"""Compare adapted obs/actions/phase to only the pinned official controller class.

No upstream imports, viewer, gamepad, training or hardware code are executed.
The complete scripts were reviewed; AST extraction isolates OnnxController.
"""
import ast
import importlib.metadata
import json
from pathlib import Path
import sys
import types

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import mujoco
import numpy as np
import onnxruntime as rt

from controllers.playground_onnx import PlaygroundController
from rft.pipeline import import_robot, verify_artifacts
from rft.records import artifact_manifest, git_state, new_run, sha256, write_json


def bounded_session(path, **kwargs):
    options = rt.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    return rt.InferenceSession(str(path),sess_options=options,providers=["CPUExecutionProvider"])


def main():
    output = new_run(prefix="walking_controller_parity")
    results = {}
    source_root = ROOT/"third_party/robot_assets/playground"
    manifest = json.loads((source_root/"ground_download_manifest.json").read_text())
    for family in ("go1","g1"):
        path = ROOT/f"configs/robots/playground_{family}.yaml"
        model,config,_ = import_robot(path)
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model,data,model.key(config["initial_keyframe"]).id)
        other = mujoco.MjData(model)
        default = data.qpos[7:].copy()
        spec = config["controller"]
        policy = (path.parent/spec["policy"]).resolve()
        filename = f"mujoco_playground/experimental/sim2sim/play_{family}_joystick.py"
        source = source_root/filename
        expected = next(f["sha256"] for f in manifest["files"] if f["path"] == filename)
        if sha256(source) != expected:
            raise ValueError("Official class source differs from pinned download")
        parsed = ast.parse(source.read_text())
        selected = next(n for n in parsed.body if isinstance(n,ast.ClassDef) and n.name == "OnnxController")
        command = np.asarray(spec["command"])
        scope = {"np":np,"mujoco":mujoco,"rt":types.SimpleNamespace(InferenceSession=bounded_session),
                 "Gamepad":lambda **kw:types.SimpleNamespace(get_command=lambda:command.copy())}
        exec(compile(ast.Module(body=[selected],type_ignores=[]),str(source),"exec"),scope)
        kwargs = {"ctrl_dt":.02} if family == "g1" else {}
        original = scope["OnnxController"](str(policy),default,n_substeps=1,**kwargs)
        adapted = PlaygroundController(model,data,family,policy,command=command)
        generator = np.random.default_rng(2841)
        maximum_obs = maximum_command = 0.0
        for _ in range(20):
            data.qpos[7:] = default+generator.normal(0,.03,model.nu)
            data.qvel[:] = generator.normal(0,.1,model.nv)
            data.qpos[3:7] = np.array([.9987502604,0,.0499791693,0])
            data.qpos[3:7] /= np.linalg.norm(data.qpos[3:7])
            other.qpos[:],other.qvel[:] = data.qpos,data.qvel
            mujoco.mj_forward(model,data); mujoco.mj_forward(model,other)
            old_obs,new_obs = original.get_obs(model,other),adapted.observation(model,data)
            maximum_obs = max(maximum_obs,float(np.max(np.abs(old_obs-new_obs))))
            original.get_control(model,other)
            command = adapted.update(model,data)
            maximum_command = max(maximum_command,float(np.max(np.abs(other.ctrl-command))))
            # Fixed command remains independent of the actuator command returned above.
            command = np.asarray(spec["command"])
        if maximum_obs != 0 or maximum_command > 1e-7:
            raise AssertionError(f"{family} parity failed: {maximum_obs}, {maximum_command}")
        results[family] = {"samples":20,"source_sha256":sha256(source),"policy_sha256":sha256(policy),
                           "max_observation_difference":maximum_obs,"max_actuator_target_difference":maximum_command,
                           "phase_compared":family == "g1","status":"passed"}
    write_json(output/"controller_parity.json",{"source":git_state(),"results":results,
               "environment":{d.metadata["Name"]:d.version for d in importlib.metadata.distributions()}})
    write_json(output/"manifest.json",{"artifacts":artifact_manifest(output)})
    print(json.dumps(results,indent=2)); print(output)


if __name__ == "__main__":
    main()
