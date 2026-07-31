

import math
import numpy as np
from scipy.spatial import cKDTree


last_points = None
last_transform = None


state = [0.0, 0.0, 0.0]


ICP_MAX_ITERS = 25
ICP_TOLERANCE = 1e-5
ICP_MAX_CORR_DIST = 0.5
ICP_MIN_MATCHES = 10




def scan_to_points(scan_data):
    points = []


    num_samples = len(scan_data)
    angle_increment = 2.0 * math.pi / num_samples


    for i, dist in enumerate(scan_data):
        dist /= 100.0  # cm to m


        if dist < 0.10 or dist > 10.0:
            continue


        angle = i * angle_increment


        x = dist * math.cos(angle)
        y = dist * math.sin(angle)


        points.append([x, y])


    return np.asarray(points)




def get_nearest_neighbors(source, target):
    tree = cKDTree(target)
    dists, idx = tree.query(source, k=1)
    return dists, idx




def best_fit_transform(A, B):
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)


    AA = A - centroid_A
    BB = B - centroid_B


    H = AA.T @ BB


    U, _, Vt = np.linalg.svd(H)


    R = Vt.T @ U.T


    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T


    t = centroid_B - R @ centroid_A


    return R, t




def icp(source, target, init_R=None, init_t=None):
    R_total = np.eye(2) if init_R is None else init_R.copy()
    t_total = np.zeros(2) if init_t is None else init_t.copy()


    src = (R_total @ source.T).T + t_total


    prev_error = float("inf")


    for _ in range(ICP_MAX_ITERS):


        dists, idx = get_nearest_neighbors(src, target)


        mask = dists < ICP_MAX_CORR_DIST


        if np.count_nonzero(mask) < ICP_MIN_MATCHES:
            break


        src_match = src[mask]
        tgt_match = target[idx[mask]]


        R_step, t_step = best_fit_transform(src_match, tgt_match)


        src = (R_step @ src.T).T + t_step


        R_total = R_step @ R_total
        t_total = R_step @ t_total + t_step


        error = np.mean(dists[mask])


        if abs(prev_error - error) < ICP_TOLERANCE:
            break


        prev_error = error


    return R_total, t_total, prev_error




def update(scan_data):
    global last_points, last_transform, state


    if scan_data is None:
        return


    current = scan_to_points(scan_data)


    if (
        last_points is None
        or len(current) < ICP_MIN_MATCHES
        or len(last_points) < ICP_MIN_MATCHES
    ):
        last_points = current
        last_transform = None
        return


    init_R = None
    init_t = None


    if last_transform is not None:
        init_R, init_t = last_transform


    R, t, error = icp(
        current,
        last_points,
        init_R,
        init_t,
    )


    last_transform = (R, t)
    dtheta = math.atan2(R[1, 0], R[0, 0])




    dx_robot = t[0]
    dy_robot = t[1]


    theta = state[2]


    dx_world = (
        dx_robot * math.cos(theta)
        - dy_robot * math.sin(theta)
    )


    dy_world = (
        dx_robot * math.sin(theta)
        + dy_robot * math.cos(theta)
    )


    state[0] += dx_world
    state[1] += dy_world
    state[2] += dtheta


    state[2] = math.atan2(
        math.sin(state[2]),
        math.cos(state[2])
    )


    last_points = current


    print(
        f"x={state[0]:.2f} "
        f"y={state[1]:.2f} "
        f"theta={math.degrees(state[2]):.1f}° "
        f"err={error:.4f}"
    )




def get_pose():
    return tuple(state)
