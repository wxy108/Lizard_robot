# Copyright 2026 Google LLC (upstream observation/action conventions)
# Modifications Copyright 2026 Lizard Robot contributors
# Licensed under the Apache License, Version 2.0. See NOTICE.playground.
"""Named-joint CPU adapters derived from pinned official native sim2sim scripts.

Source: google-deepmind/mujoco_playground 12ad7f3, experimental/sim2sim/
play_go1_joystick.py and play_g1_joystick.py. No simulator stepping or RFT calls.
"""
import mujoco
import numpy as np


def joint_order(kind):
    if kind == "go1":
        return [f"{leg}_{joint}_joint" for leg in ("FR","FL","RR","RL") for joint in ("hip","thigh","calf")]
    if kind == "g1":
        legs = [f"{side}_{joint}_joint" for side in ("left","right")
                for joint in ("hip_pitch","hip_roll","hip_yaw","knee","ankle_pitch","ankle_roll")]
        waist = [f"waist_{joint}_joint" for joint in ("yaw","roll","pitch")]
        arms = [f"{side}_{joint}_joint" for side in ("left","right") for joint in
                ("shoulder_pitch","shoulder_roll","shoulder_yaw","elbow","wrist_roll","wrist_pitch","wrist_yaw")]
        return legs+waist+arms
    raise ValueError("No audited controller contract for this robot family")


class PlaygroundController:
    def __init__(self,model,data,kind,policy_path,*,control_dt=0.02,command=(0.4,0,0),session=None):
        self.kind,self.control_dt = kind,float(control_dt)
        if not np.isclose(self.control_dt,0.02,rtol=0,atol=1e-12):
            raise ValueError("This pinned policy requires a 50 Hz control contract")
        self.joints = [model.joint(name).id for name in joint_order(kind)]
        self.qadr = np.asarray([model.jnt_qposadr[j] for j in self.joints])
        self.vadr = np.asarray([model.jnt_dofadr[j] for j in self.joints])
        if model.nu != len(self.joints) or list(model.actuator_trnid[:,0]) != self.joints:
            raise ValueError("Policy actuator/joint order mismatch")
        if not np.all(model.actuator_trntype == mujoco.mjtTrn.mjTRN_JOINT):
            raise ValueError("Policy needs joint position servos")
        if not np.allclose(model.actuator_gear[:,0],1) or not np.allclose(model.actuator_biasprm[:,1],-model.actuator_gainprm[:,0]):
            raise ValueError("Unexpected position-servo parameterization")
        self.default = data.qpos[self.qadr].copy()
        self.command = np.asarray(command,dtype=float)
        if self.command.shape != (3,) or not np.all(np.isfinite(self.command)):
            raise ValueError("Velocity command must be finite vx, vy, yaw_rate")
        self.site = "imu" if kind == "go1" else "imu_in_pelvis"
        self.linvel = "local_linvel" if kind == "go1" else "local_linvel_pelvis"
        self.gyro = "gyro" if kind == "go1" else "gyro_pelvis"
        self.sid = model.site(self.site).id
        for name in (self.linvel,self.gyro):
            if model.sensor(name).dim[0] != 3:
                raise ValueError("Invalid policy velocity sensor dimension")
        if session is None:
            import onnxruntime as rt
            options = rt.SessionOptions()
            options.intra_op_num_threads = 1
            options.inter_op_num_threads = 1
            session = rt.InferenceSession(str(policy_path),sess_options=options,providers=["CPUExecutionProvider"])
        self.session = session
        expected = 48 if kind == "go1" else 103
        if self.session.get_inputs()[0].name != "obs" or self.session.get_inputs()[0].shape[-1] != expected:
            raise ValueError("Policy observation signature mismatch")
        if "continuous_actions" not in [item.name for item in self.session.get_outputs()]:
            raise ValueError("Policy action signature mismatch")
        self.last = np.zeros(model.nu,dtype=np.float32)
        self.phase = np.array([0.0,np.pi])
        self.updates = 0
        self.last_observation = None

    def observation(self,model,data):
        linvel,gyro = data.sensor(self.linvel).data,data.sensor(self.gyro).data
        gravity = data.site_xmat[self.sid].reshape(3,3).T @ np.array([0,0,-1])
        joints,velocities = data.qpos[self.qadr]-self.default,data.qvel[self.vadr]
        if self.kind == "go1":
            values = [linvel,gyro,gravity,joints,velocities,self.last,self.command]
        else:
            values = [linvel,gyro,gravity,self.command,joints,velocities,self.last,
                      np.cos(self.phase),np.sin(self.phase)]
        obs = np.hstack(values).astype(np.float32)
        if not np.all(np.isfinite(obs)):
            raise FloatingPointError("Non-finite policy observation")
        return obs

    def update(self,model,data):
        self.last_observation = self.observation(model,data)
        action = np.asarray(self.session.run(["continuous_actions"],{"obs":self.last_observation[None]})[0][0])
        if action.shape != self.last.shape or not np.all(np.isfinite(action)):
            raise FloatingPointError("Invalid policy action")
        self.last = action.copy()
        if self.kind == "g1":
            self.phase = np.fmod(self.phase+2*np.pi*1.5*self.control_dt+np.pi,2*np.pi)-np.pi
        self.updates += 1
        return self.default+0.5*action
