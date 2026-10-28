# Copyright 2026 Google LLC (Playground observation/action/phase conventions).
# Modifications Copyright 2026 Lizard Robot contributors.
# Apache-2.0: see NOTICE.playground. Wheel equations are standard differential kinematics.
"""Reviewed declarative policy contracts and separate differential-drive adapter."""
import json
from pathlib import Path

import mujoco
import numpy as np

from .playground_onnx import PlaygroundController

PROFILES = Path(__file__).resolve().parents[1]/"configs/controllers/playground_profiles.json"


class ProfilePolicyController:
    def __init__(self,model,data,spec,policy_path,session=None):
        self.spec = json.loads(PROFILES.read_text())[spec["profile"]]
        self.command = np.asarray(spec["command"],dtype=float)
        self.control_dt = float(spec["control_dt"])
        if self.command.shape != (3,) or not np.all(np.isfinite(self.command)) or self.control_dt != .02:
            raise ValueError("Policy requires finite 3D command and exact 50 Hz contract")
        joints = [model.joint(name).id for name in self.spec["joints"]]
        self.qadr = np.array([model.jnt_qposadr[j] for j in joints])
        self.vadr = np.array([model.jnt_dofadr[j] for j in joints])
        if model.nu != len(joints) or list(model.actuator_trnid[:,0]) != joints:
            raise ValueError("Official policy actuator/joint contract mismatch")
        if model.nq != 7+model.nu or model.nv != 6+model.nu or not np.array_equal(self.qadr,np.arange(7,model.nq)):
            raise ValueError("Official policy requires one floating root and exactly its declared scalar joints")
        if not np.allclose(model.actuator_biasprm[:,1],-model.actuator_gainprm[:,0]):
            raise ValueError("Policy needs original position servos")
        self.default = model.key(spec["default_keyframe"]).qpos[self.qadr].copy()
        self.sid = model.site(self.spec["site"]).id
        for key in ("linvel","gyro"):
            if model.sensor(self.spec[key]).dim[0] != 3:
                raise ValueError("Policy sensor contract mismatch")
        if session is None:
            import onnxruntime as rt
            options = rt.SessionOptions(); options.intra_op_num_threads = options.inter_op_num_threads = 1
            session = rt.InferenceSession(str(policy_path),sess_options=options,providers=["CPUExecutionProvider"])
        self.session = session
        if session.get_inputs()[0].name != "obs" or session.get_inputs()[0].shape[-1] != 16+3*model.nu:
            raise ValueError("Policy observation shape mismatch")
        if "continuous_actions" not in [o.name for o in session.get_outputs()]:
            raise ValueError("Policy output contract mismatch")
        self.last = np.zeros(model.nu,dtype=np.float32)
        self.phase = np.array([0.,np.pi]); self.updates = 0

    def observation(self,model,data):
        gravity = data.site_xmat[self.sid].reshape(3,3).T @ np.array([0,0,-1])
        angles,velocity = data.qpos[self.qadr]-self.default,data.qvel[self.vadr].copy()
        if self.spec.get("zero_head_observation"):
            angles[:2] *= 0; velocity[:2] *= 0
        phase = self.phase
        if self.spec["phase"] in ("t1_1.5hz","apollo_speed_dependent") and np.linalg.norm(self.command) < .01:
            phase = np.ones(2)*np.pi
        obs = np.hstack([data.sensor(self.spec["linvel"]).data,data.sensor(self.spec["gyro"]).data,
                         gravity,self.command,angles,velocity,self.last,np.cos(phase),np.sin(phase)]).astype(np.float32)
        if not np.all(np.isfinite(obs)):
            raise FloatingPointError("Non-finite policy observation")
        return obs

    def update(self,model,data):
        obs = self.observation(model,data)
        action = self.session.run(["continuous_actions"],{"obs":obs[None]})[0][0]
        if action.shape != self.last.shape or not np.all(np.isfinite(action)):
            raise FloatingPointError("Invalid policy action")
        self.last = action.copy()
        magnitude = np.linalg.norm(self.command)
        frequency = ((1.25 if magnitude < .01 else 1.25+.5*min(magnitude,1.5)/1.5)
                     if self.spec["phase"] == "apollo_speed_dependent" else 1.5)
        self.phase = np.fmod(self.phase+2*np.pi*self.control_dt*frequency+np.pi,2*np.pi)-np.pi
        self.updates += 1
        return self.default+self.spec["action_scale"]*action


class DifferentialDriveController:
    def __init__(self,model,spec):
        self.spec = spec; self.updates = 0
        radius,separation = float(spec["wheel_radius_m"]),float(spec["wheel_separation_m"])
        if not np.isfinite(radius+separation) or min(radius,separation) <= 0:
            raise ValueError("Wheel radius/separation must be positive physical lengths")
        self.radius,self.separation = radius,separation
        self.command = np.asarray(spec["command"],dtype=float)
        if self.command.shape != (3,) or not np.all(np.isfinite(self.command)) or self.command[1] != 0:
            raise ValueError("Differential drive supports finite vx/yaw_rate, not lateral commands")
        joints = [model.joint(name).id for name in spec["joints"]]
        if model.nu != 2 or list(model.actuator_trnid[:,0]) != joints:
            raise ValueError("Wheel actuator mapping mismatch")
        if not np.allclose(model.actuator_biasprm[:,2],-model.actuator_gainprm[:,0]):
            raise ValueError("Wheel adapter needs declared velocity servos")

    def update(self,model,data):
        vx,_,yaw = self.command
        self.updates += 1
        return np.array([(vx-yaw*self.separation/2)/self.radius,(vx+yaw*self.separation/2)/self.radius])


def make_controller(model,data,spec,base):
    kind = spec["type"]
    if kind == "differential_drive":
        return DifferentialDriveController(model,spec),None
    path = (Path(base)/spec["policy"]).resolve()
    if kind == "playground_onnx":
        return PlaygroundController(model,data,spec["family"],path,control_dt=spec["control_dt"],command=spec["command"]),path
    if kind == "profile_onnx":
        return ProfilePolicyController(model,data,spec,path),path
    raise ValueError("No reviewed controller adapter for requested type")
