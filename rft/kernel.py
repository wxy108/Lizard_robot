"""NumPy-only copy of the frozen triangle law; no simulator or controller.

Copied from sim_fxn_lib.py at ccc6946 (SHA-256 recorded in the worklog).
Only the function name changes. Coordinate/convention corrections belong to
an explicit provider mode, so compatibility can be tested independently.
The raw return convention is body-on-sand; providers negate exactly once.
Portions derive from RFT-SiM, Copyright (c) 2026 Ryan Walker Brown, MIT.
See NOTICE in this directory for the retained permission/license text.
"""

import numpy as np

def evaluate_body_raw(
    body_m,
    origin_m,
    orientation_rad,
    vel_mps,
    ang_vel_radps,
    RFTCOEFF=3.75,
    sand_height_m=0.0,
    rotation_matrix=None,
):
    """Evaluate 3D RFT on all mesh triangles.

    Force outputs follow the upstream RFT-SiM body-on-sand convention. A
    simulator applying the sand reaction to the body must negate them.
    ``rotation_matrix`` should be MuJoCo's world-from-body ``data.xmat``;
    the legacy Euler input remains available as [yaw, pitch, roll].
    """
    # Convert inputs to centimeter units at the beginning
    origin_cm = origin_m * 100
    r_cm = body_m['r'] * 100
    area_cm2 = body_m['A'] * 10000
    vel_cmps = vel_mps * 100
    sand_height_cm = sand_height_m * 100
    if rotation_matrix is None:
        if orientation_rad is None or len(orientation_rad) != 3:
            raise ValueError("orientation_rad must be [yaw, pitch, roll]")
        psi, theta, phi = orientation_rad
        Rz = np.array([[np.cos(psi), np.sin(psi), 0],
                       [-np.sin(psi), np.cos(psi), 0],
                       [0, 0, 1]])
        Ry = np.array([[np.cos(theta), 0, -np.sin(theta)],
                       [0, 1, 0],
                       [np.sin(theta), 0, np.cos(theta)]])
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(phi), np.sin(phi)],
                       [0, -np.sin(phi), np.cos(phi)]])
        world_from_body = (Rx @ Ry @ Rz).T
    else:
        world_from_body = np.asarray(rotation_matrix, dtype=float).reshape(3, 3)

    # Rotate position & normal vectors of all faces based on current orientation
    r_rot_vec_cm = (world_from_body @ r_cm.T).T
    depth_cm = origin_cm[2] + r_rot_vec_cm[:, 2]
    n_rot_vec = (world_from_body @ body_m['n'].T).T

    # Calculate velocity of each plate element based on body velocities (in cm/s and cm)
    n_elements = r_cm.shape[0]
    ang_vel_radps_tiled = np.tile(ang_vel_radps, (n_elements, 1))
    v_vec_cmps = vel_cmps + np.cross(ang_vel_radps_tiled, r_rot_vec_cm)

    # Normalize velocity & normal vectors
    v_norm_vec = v_vec_cmps / (np.linalg.norm(v_vec_cmps, axis=1, keepdims=True) + 1e-10)
    n_norm_vec = n_rot_vec / (np.linalg.norm(n_rot_vec, axis=1, keepdims=True) + 1e-10)

    # Logical conditions to count force on element
    leading_edge = np.sum(n_norm_vec * v_norm_vec, axis=1) > 0
    intruding = depth_cm < sand_height_cm
    include = leading_edge & intruding

    if not np.any(include):
        zeros = np.zeros((body_m['r'].shape[0], 3))
        return (
            np.zeros(3),
            np.zeros(3),
            np.zeros((0, 3)),
            np.zeros((0, 3)),
            include,
            zeros,
            RFTCOEFF,
        )

    # Isolate the elements that satisfy this condition
    n_inc = n_norm_vec[include, :]
    v_inc = v_norm_vec[include, :]
    r_inc_cm = r_rot_vec_cm[include, :]




    # generate basis for RFT decomposition
    n_inc_z_abs = np.abs(n_inc[:, 2])
    horizontal_case = n_inc_z_abs >= 1 - 1e-3
    nominal_case = n_inc_z_abs < 1 - 1e-3

    v_inc_modified = v_inc + np.array([1e-5, 0, 0])
    e2_vec = np.zeros_like(n_inc)
    e2_vec[horizontal_case, :] = (v_inc_modified[horizontal_case, :] * np.array([1, 1, 0])) / np.linalg.norm((v_inc_modified[horizontal_case, :] * np.array([1, 1, 0])), axis=1, keepdims=True)
    e2_vec[nominal_case, :] = (n_inc[nominal_case, :] * np.array([1, 1, 0])) / np.linalg.norm((n_inc[nominal_case, :] * np.array([1, 1, 0])), axis=1, keepdims=True)

    e1_vec = np.cross(e2_vec, np.tile([0, 0, 1], (np.sum(include), 1)))

    # decompose velocity vector
    v1_vec = np.sum(v_inc * e1_vec, axis=1, keepdims=True) * e1_vec
    v23_vec = v_inc - v1_vec

    # calculate 2D RFT parameters
    beta_vec = np.arctan2(n_inc[:, 2], np.sum(n_inc * e2_vec, axis=1)) + np.pi / 2
    gamma_vec = -np.arctan2(v23_vec[:, 2], np.sum(v23_vec * e2_vec, axis=1))
    # RFTCOEFF = 3.75
    # RFTCOEFF = 2
    # from Li et al. -> M matrices for each granular medium
    M_generic = np.array([0.206, 0.169, 0.212, 0.358, 0.055, -0.124, 0.253, 0.007, 0.088])
    M_testing = M_generic * RFTCOEFF

    # 2D alpha components (output in N/cm^3)
    def afunc(gamma_vec, beta_vec, M):
        fit_z = 0
        fit_x = 0
        A_0_0, A_1_0, B_1_1, B_0_1, B_neg1_1, C_1_1, C_0_1, C_neg1_1, D_1_0 = M
        for m in range(-1, 2):
            for n in range(2):
                arg = 2 * np.pi * (m * beta_vec / np.pi + n * gamma_vec / (2 * np.pi))
                if m == 0 and n == 0: Amn, Bmn, Cmn, Dmn = A_0_0, 0, 0, 0
                elif m == 1 and n == 0: Amn, Bmn, Cmn, Dmn = A_1_0, 0, 0, D_1_0
                elif m == 1 and n == 1: Amn, Bmn, Cmn, Dmn = 0, B_1_1, C_1_1, 0
                elif m == 0 and n == 1: Amn, Bmn, Cmn, Dmn = 0, B_0_1, C_0_1, 0
                elif m == -1 and n == 1: Amn, Bmn, Cmn, Dmn = 0, B_neg1_1, C_neg1_1, 0
                else: Amn, Bmn, Cmn, Dmn = 0, 0, 0, 0
                fit_z += Amn * np.cos(arg) + Bmn * np.sin(arg)
                fit_x += Cmn * np.cos(arg) + Dmn * np.sin(arg)
        return fit_z, fit_x

    aZ, aX = afunc(gamma_vec, beta_vec, M_testing)  # in N/cm^3
    _, aY = afunc(np.array([0]*len(gamma_vec)), np.array([0]*len(beta_vec)), M_testing)  # in N/cm^3

    # Calculate scaling factors (dimensionless)
    vt = np.linalg.norm(v1_vec, axis=1)
    ct_fit_A = [0.440850096954369, 3.62263982880971, 1.60910808139526, 0.41]
    f1 = ct_fit_A[0] * (np.tanh(ct_fit_A[1] * vt - ct_fit_A[2]) + np.tanh(ct_fit_A[2])) / ct_fit_A[3]

    vn = np.linalg.norm(v23_vec, axis=1)
    cn_fit_B = [1.99392673405210, 1.61146827229181, 0.973746396532650, 4.31]
    f23 = (
        cn_fit_B[0] *
        (np.arctanh(cn_fit_B[1] * vn - cn_fit_B[2]) + np.arctanh(cn_fit_B[2])) /
        cn_fit_B[3])

    area_cm2 = body_m['A'][include] * 10000
    depth_cm_included = depth_cm[include]

    # Calculate forces in Newtons
    f1_magnitude = -f1.reshape(-1, 1) * aY.reshape(-1, 1) * np.sign(np.einsum('ij,ij->i', v_inc, e1_vec)).reshape(-1, 1) * depth_cm_included[:, None] * area_cm2[:, None]
    F1 = f1_magnitude * e1_vec

    f2_magnitude = -f23.reshape(-1, 1) * aX.reshape(-1, 1) * depth_cm_included[:, None] * area_cm2[:, None]
    F2 = f2_magnitude * e2_vec

    f3_magnitude = f23.reshape(-1, 1) * aZ.reshape(-1, 1) * depth_cm_included[:, None] * area_cm2[:, None]
    F3 = f3_magnitude * np.array([0, 0, 1])
    F_i = F1 + F2 + F3

    # Moments use the world-frame lever arm in metres.
    r_inc_m = r_rot_vec_cm[include] / 100.0
    Mi_mat = np.cross(r_inc_m, F_i)
    M_total = np.sum(Mi_mat, axis=0)

    F_mat_full = np.zeros((body_m['r'].shape[0], 3))
    F_mat_full[include] = F_i
    F_total = np.sum(F_i, axis=0)

    return F_total, M_total, F_i, Mi_mat, include, F_mat_full, RFTCOEFF
