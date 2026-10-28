"""Per-instance RFT force provider; no controller, stepping or global force writes."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from .binding import bind_surfaces
from .kernel import evaluate_body_raw


@dataclass(frozen=True)
class SurfaceForces:
    surface_id: str
    body_name: str
    positions_world: np.ndarray
    normals_world: np.ndarray
    point_velocities_world: np.ndarray
    raw_reaction: np.ndarray
    forces: np.ndarray
    active: np.ndarray
    submerged: np.ndarray
    nonfinite_count: int


@dataclass(frozen=True)
class ForceResult:
    qfrc: np.ndarray
    surfaces: tuple[SurfaceForces, ...]

    @property
    def total_force(self):
        return sum((s.forces.sum(axis=0) for s in self.surfaces), np.zeros(3))


class RFTForceProvider:
    """Compute generalized forces from current, forward-synchronized mjData.

    legacy_lizard_v1 reproduces the legacy COM/body-origin velocity mixture
    and absolute-z depth law intentionally. body_frame_v1 uses XBODY velocity
    at xpos and shifts the kernel frame to the requested sand plane.
    """

    def __init__(self, model, surfaces, *, convention="body_frame_v1", sand_height=0.0,
                 coefficient=3.75, alpha=0.5, application="triangles"):
        if convention not in ("legacy_lizard_v1", "body_frame_v1"):
            raise ValueError(f"Unknown RFT convention: {convention}")
        if application not in ("triangles", "body_wrench"):
            raise ValueError(f"Unknown force application: {application}")
        if not np.isfinite(coefficient) or coefficient <= 0 or not 0 < alpha <= 1:
            raise ValueError("Coefficient must be positive and alpha in (0,1]")
        if not np.isfinite(sand_height):
            raise ValueError("Sand height must be finite")
        self.model = model
        self.bindings = bind_surfaces(model, surfaces)
        self.convention = convention
        self.sand_height = float(sand_height)
        self.coefficient = float(coefficient)
        self.alpha = float(alpha)
        self.application = application
        self.history = {b.surface.surface_id: np.zeros_like(b.surface.centers) for b in self.bindings}

    def reset(self, history=None):
        if history is not None and set(history) != set(self.history):
            raise ValueError("History keys must match the bound surfaces")
        for key, buffer in self.history.items():
            if history is None:
                buffer.fill(0)
            else:
                value = np.asarray(history[key])
                if value.shape != buffer.shape or not np.all(np.isfinite(value)):
                    raise ValueError(f"Invalid smoothing history: {key}")
                buffer[:] = value

    def compute(self, data) -> ForceResult:
        qfrc = np.zeros(self.model.nv)
        results = []
        wrenches = {}
        for binding in self.bindings:
            surface, bid = binding.surface, binding.body_id
            origin = np.asarray(data.xpos[bid])
            world_from_body = np.asarray(data.xmat[bid]).reshape(3, 3)
            positions = surface.centers @ world_from_body.T + origin
            normals = surface.normals @ world_from_body.T
            velocity = np.empty(6)
            frame = (mujoco.mjtObj.mjOBJ_BODY if self.convention == "legacy_lizard_v1"
                     else mujoco.mjtObj.mjOBJ_XBODY)
            mujoco.mj_objectVelocity(self.model, data, frame, bid, velocity, 0)
            point_velocities = velocity[3:] + np.cross(velocity[:3], positions - origin)
            kernel_origin = origin.copy()
            kernel_height = self.sand_height
            if self.convention == "body_frame_v1":
                kernel_origin[2] -= self.sand_height
                kernel_height = 0.0
            _, _, _, _, include, raw, _ = evaluate_body_raw(
                {"r": surface.centers, "n": surface.normals, "A": surface.areas},
                kernel_origin, None, velocity[3:], velocity[:3],
                RFTCOEFF=self.coefficient, sand_height_m=kernel_height,
                rotation_matrix=world_from_body,
            )
            invalid = int(np.count_nonzero(~np.isfinite(raw)))
            if invalid and self.convention != "legacy_lizard_v1":
                raise FloatingPointError(f"Non-finite RFT force on {surface.surface_id}: {invalid}")
            reaction = -np.nan_to_num(raw, nan=0.0, posinf=0.0, neginf=0.0)
            smoothed = self.history[surface.surface_id]
            smoothed[~include] = 0
            smoothed[include] = self.alpha * reaction[include] + (1-self.alpha) * smoothed[include]
            if self.application == "body_wrench":
                wrench = wrenches.setdefault(bid, np.zeros(6))
                wrench[:3] += smoothed[include].sum(axis=0)
                wrench[3:] += np.cross(positions[include] - data.xipos[bid],
                                       smoothed[include]).sum(axis=0)
            else:
                for triangle in np.flatnonzero(include):
                    force = smoothed[triangle]
                    if not np.any(force):
                        continue
                    mujoco.mj_applyFT(self.model, data, force, np.zeros(3),
                                      positions[triangle], bid, qfrc)
            results.append(SurfaceForces(surface.surface_id, surface.body_name,
                           positions.copy(), normals.copy(), point_velocities.copy(),
                           reaction.copy(), smoothed.copy(), include.copy(),
                           positions[:, 2] < self.sand_height, invalid))
        for bid, wrench in wrenches.items():
            mujoco.mj_applyFT(self.model, data, wrench[:3], wrench[3:], data.xipos[bid], bid, qfrc)
        if not np.all(np.isfinite(qfrc)):
            raise FloatingPointError("Non-finite generalized RFT force")
        return ForceResult(qfrc, tuple(results))
