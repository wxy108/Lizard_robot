"""Lizard V1 gait controller with TWO selectable laws (config: controller:).

  controller: sine      -> traveling-wave body + antisymmetric sine legs
                           (our optimized straight-line gait)
  controller: hardware  -> port of the real-robot MATLAB law
                           (lizardCode_1DOF.m), body changed to a STANDING C-wave
                           locked to duty-cycle diagonal-trot legs

Both laws share the joint layout (spine [Front,Back,Tail] + legs
FR/FL/HR/HL), the per-joint `offset` block, and gait.settle_time /
gait.ramp_time. Their tunables live in SEPARATE yaml blocks (spine/legs
for sine, hardware for the real-robot law) so the two never mix.

NOTE: the original MATLAB `dutyFunction` helper was not provided with the
reference, so the duty-cycle leg shape below is a documented reconstruction
(foot presses DOWN for the `duty` fraction of each cycle, neutral/up during
swing). Structure (cos body wave, phi_leg L/R, phi_lat front/back diagonal
trot, tail = 2*body_phase) matches the reference exactly.
"""

from __future__ import annotations

import copy
import math

import numpy as np
import yaml

TWO_PI = 2.0 * math.pi


def load_config(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def apply_overrides(config, overrides):
    """Apply CLI overrides like 'controller=hardware' (dot-notation)."""
    config = copy.deepcopy(config)
    for item in overrides:
        key, sep, raw = item.partition("=")
        if not sep:
            raise ValueError("override must be key=value, got: %r" % item)
        node = config
        parts = key.strip().split(".")
        for p in parts[:-1]:
            node = node.setdefault(p, {})
        node[parts[-1]] = yaml.safe_load(raw)
    return config


def duty_wave(u, duty):
    """Reconstructed dutyFunction. u in [0,1) = cycle fraction.
    Foot DOWN (stance) for the first `duty` fraction as a smooth 0->1->0
    bump; UP/neutral (return 0) during swing. Output in [0, 1]."""
    duty = max(1e-3, min(0.999, duty))
    if u < duty:
        return math.sin(math.pi * u / duty)
    return 0.0


class GaitController:
    """Maps time -> position targets (ctrl). Law chosen by config['controller']."""

    def __init__(self, model, config):
        self.model = model
        self.cfg = config
        self.mode = str(config.get("controller", "sine")).lower()
        if self.mode not in ("sine", "hardware"):
            raise ValueError("controller must be 'sine' or 'hardware'")

        g = config["gait"]
        self.settle_time = float(g.get("settle_time", 0.0))
        self.ramp_time = float(g.get("ramp_time", 0.0))

        # ---- shared joint layout ----
        sp = config["spine"]
        self.spine_joints = list(sp["joints"])             # head -> tail
        self.Ns = len(self.spine_joints)
        self.spine_index = {j: i for i, j in enumerate(self.spine_joints)}

        lg = config["legs"]
        self.right = set(lg["right"])
        self.left = set(lg["left"])
        self.girdle = dict(lg["girdle"])
        self.leg_joints = list(self.right | self.left)
        self.front_spine = self.spine_joints[0]            # girdle name = front

        self.offset = {j: float(config["offset"][j])
                       for j in self.spine_joints + self.leg_joints}

        import mujoco
        self.act_id = {}
        for j in self.spine_joints + self.leg_joints:
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, j)
            if aid < 0:
                raise KeyError("actuator '%s' not found in model" % j)
            self.act_id[j] = aid

        if self.mode == "sine":
            self._init_sine(config)
        else:
            self._init_hardware(config)

    # ================= SINE law =================
    def _init_sine(self, config):
        g, sp, lg = config["gait"], config["spine"], config["legs"]
        self.freq = float(g["frequency"])
        self.wave_speed = float(g.get("wave_speed", 1.0))
        self.spine_amp = float(sp["amplitude"])
        self.wavenumber = float(sp["wavenumber"])
        self.turn_trim = float(sp.get("turn_trim", 0.0))
        self.leg_amp = float(lg["amplitude"])
        self.phase_offset = float(lg["phase_offset"])

    def _spatial_phase(self, spine_joint):
        denom = (self.Ns - 1) if self.Ns > 1 else 1
        return (TWO_PI * self.wavenumber * self.wave_speed / denom
                * self.spine_index[spine_joint])

    def _sine_spine(self, joint, t, ramp):
        phase = TWO_PI * self.freq * t + self._spatial_phase(joint)
        return (self.offset[joint] + ramp * self.turn_trim
                + ramp * self.spine_amp * math.sin(phase))

    def _sine_leg(self, leg, t, ramp):
        side = 1.0 if leg in self.right else -1.0
        phase = (TWO_PI * self.freq * t
                 + self._spatial_phase(self.girdle[leg])
                 + TWO_PI * self.phase_offset)
        return self.offset[leg] + ramp * side * self.leg_amp * math.sin(phase)

    # ================= HARDWARE law (real-robot port) =================
    def _init_hardware(self, config):
        hw = config["hardware"]
        # body = STANDING C-wave: all spine joints share one sinusoid (same
        # temporal phase -> the whole body curls into a C, flipping side each
        # half cycle), phase-LOCKED to the leg cycle at leg_freq so the bend
        # alternates with the diagonal foot contacts.
        self.hw_body_amp = float(hw["body_amp"])
        self.hw_body_lock = float(hw.get("body_lock", 0.0))   # bend timing vs contact [rad]
        self.hw_shape = {j: float(hw["body_shape"][j]) for j in self.spine_joints}
        self.hw_leg_amp = float(hw["leg_amp"])
        self.hw_leg_freq = float(hw["leg_freq"])
        self.hw_duty = float(hw["duty"])
        self.hw_phi_leg = float(hw["phi_leg"])             # L/R [rad]
        self.hw_phi_lat = float(hw["phi_lat"])             # front/back [rad]
        self.hw_down_sign = {j: float(hw["down_sign"][j]) for j in self.leg_joints}

    def _hw_spine(self, joint, t, ramp):
        # standing wave: identical temporal sinusoid on every spine joint,
        # spatial pattern set by body_shape (all same sign -> C-shape bend).
        signal = math.sin(TWO_PI * self.hw_leg_freq * t + self.hw_body_lock)
        amp = self.hw_body_amp * self.hw_shape[joint]
        return self.offset[joint] + ramp * amp * signal

    def _hw_leg(self, leg, t, ramp):
        # diagonal trot: +phi_leg if right, +phi_lat if hind
        ph = 0.0
        if leg in self.right:
            ph += self.hw_phi_leg
        if self.girdle[leg] != self.front_spine:           # hind limb
            ph += self.hw_phi_lat
        u = ((TWO_PI * self.hw_leg_freq * t + ph) / TWO_PI) % 1.0
        bump = duty_wave(u, self.hw_duty)
        return self.offset[leg] + ramp * self.hw_down_sign[leg] * self.hw_leg_amp * bump

    # ================= dispatch =================
    def _ramp(self, t):
        if self.ramp_time <= 0.0:
            return 0.0 if t < self.settle_time else 1.0
        if t <= self.settle_time:
            return 0.0
        if t >= self.settle_time + self.ramp_time:
            return 1.0
        x = (t - self.settle_time) / self.ramp_time
        return 0.5 * (1.0 - math.cos(math.pi * x))

    def update(self, data):
        """Write position targets into data.ctrl. Call every physics step."""
        t = data.time
        ramp = self._ramp(t)
        if self.mode == "sine":
            for j in self.spine_joints:
                data.ctrl[self.act_id[j]] = self._sine_spine(j, t, ramp)
            for j in self.leg_joints:
                data.ctrl[self.act_id[j]] = self._sine_leg(j, t, ramp)
        else:
            for j in self.spine_joints:
                data.ctrl[self.act_id[j]] = self._hw_spine(j, t, ramp)
            for j in self.leg_joints:
                data.ctrl[self.act_id[j]] = self._hw_leg(j, t, ramp)
        return data.ctrl
