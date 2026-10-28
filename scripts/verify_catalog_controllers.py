"""Compare catalog adapters with pinned original class math, including source quirks."""
import ast
import json
from pathlib import Path
import sys
import types

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import mujoco
import numpy as np

from controllers.catalog_controllers import ProfilePolicyController
from rft.catalog import catalog,config_path,verify_installed
from rft.pipeline import import_robot
from rft.records import new_run,sha256,write_json,git_state,artifact_manifest
from scripts.verify_walking_controllers import bounded_session


def main():
    output = new_run(ROOT/"outputs/robot_catalog"/("controller_parity_"+__import__("datetime").datetime.now().strftime("%Y%m%d_%H%M%S_%f")))
    results = {}
    base = ROOT/catalog()["sources"]["playground"]["destination"]
    for family,filename in (("berkeley","bh"),("t1","t1"),("apollo","apollo")):
        verify_installed(family)
        model,config,_ = import_robot(config_path(family))
        spec = config["controller"]
        source = base/f"mujoco_playground/experimental/sim2sim/play_{filename}_joystick.py"
        tree = ast.parse(source.read_text())
        node = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name == "OnnxController")
        current_command = np.array(spec["command"],dtype=float)
        scope = {"np":np,"mujoco":mujoco,"rt":types.SimpleNamespace(InferenceSession=bounded_session),
                 "Gamepad":lambda **kw:types.SimpleNamespace(get_command=lambda:current_command.copy())}
        exec(compile(ast.Module(body=[node],type_ignores=[]),str(source),"exec"),scope)
        data,other = mujoco.MjData(model),mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model,data,model.key(config["initial_keyframe"]).id)
        policy = (config_path(family).parent/spec["policy"]).resolve()
        adapter = ProfilePolicyController(model,data,spec,policy)
        original = scope["OnnxController"](str(policy),adapter.default,ctrl_dt=.02,n_substeps=1,
                                           action_scale=adapter.spec["action_scale"])
        rng = np.random.default_rng(2930)
        observation_error = command_error = 0.0
        upstream_changes_state = False
        for index in range(30):
            current_command = np.array(spec["command"]) if index%3 == 0 else (np.zeros(3) if index%3 == 1 else np.array([.005,0,0]))
            adapter.command = current_command.copy()
            data.qpos[7:] = adapter.default+rng.normal(0,.02,model.nu)
            data.qvel[:] = rng.normal(0,.1,model.nv)
            other.qpos[:],other.qvel[:] = data.qpos,data.qvel
            mujoco.mj_forward(model,data); mujoco.mj_forward(model,other)
            before = data.qvel.copy()
            expected = original.get_obs(model,other)
            actual = adapter.observation(model,data)
            upstream_changes_state |= not np.array_equal(other.qvel,before)
            if not np.array_equal(data.qvel,before):
                raise AssertionError("Adapter observation mutated simulator state")
            observation_error = max(observation_error,float(np.max(np.abs(expected-actual))))
            original.get_control(model,other)
            command_error = max(command_error,float(np.max(np.abs(other.ctrl-adapter.update(model,data)))))
        if observation_error != 0 or command_error > 1e-7:
            raise AssertionError(f"{family} controller math differs: {observation_error} / {command_error}")
        results[family] = {"samples":30,"max_observation_difference":observation_error,
            "max_target_difference":command_error,"upstream_observation_mutates_qvel":upstream_changes_state,
            "adapter_observation_mutates_qvel":False,"source_sha256":sha256(source),"policy_sha256":sha256(policy),
            "intentional_difference":"T1 source zeros head qvel through a NumPy view; adapter masks a copy, not physical state" if family == "t1" else None}
    write_json(output/"parity.json",{"source":git_state(),"results":results})
    write_json(output/"manifest.json",{"artifacts":artifact_manifest(output)})
    print(json.dumps(results,indent=2)); print(output)


if __name__ == "__main__":
    main()
