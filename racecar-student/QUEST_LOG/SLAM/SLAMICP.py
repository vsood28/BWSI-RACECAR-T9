import numpy as np
import math


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
    angle_increment = 360.0 / num_samples

    for i, dist in enumerate(scan_data):
        if dist < 0.1 or dist > 10.0:
            continue

        angle = math.radians(i * angle_increment)
        x = dist * math.cos(angle)
        y = dist * math.sin(angle)
        points.append([x, y])

    return np.array(points)


def get_nearest_neighbors(source, target):
    dists = np.zeros(len(source))
    idx = np.zeros(len(source), dtype=int)

    for i, c in enumerate(source):
        min_dist = 1000000000000000000000000
        min_idx = -1
        for j, y in enumerate(target):
            dist = math.sqrt((c[0] - y[0]) ** 2 + (c[1] - y[1]) ** 2)
            if dist < min_dist:
                min_dist = dist
                min_idx = j
        dists[i] = min_dist
        idx[i] = min_idx

    return dists, idx


def best_fit_transform(A, B):
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    AA = A - centroid_A
    BB = B - centroid_B

    H = AA.T @ BB
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = centroid_B - R @ centroid_A
    return R, t


def icp(source, target, init_R=None, init_t=None):
    R_total = np.eye(2) if init_R is None else init_R.copy()
    t_total = np.zeros(2) if init_t is None else init_t.copy()

    src = (R_total @ source.T).T + t_total

    prev_error = None
    mean_error = float("nan")
    iters_run = 0

    for iters_run in range(1, ICP_MAX_ITERS + 1):
        dists, idx = get_nearest_neighbors(src, target)

        inliers = dists < ICP_MAX_CORR_DIST
        if np.count_nonzero(inliers) < ICP_MIN_MATCHES:
            break

        src_matched = src[inliers]
        tgt_matched = target[idx[inliers]]

        R_step, t_step = best_fit_transform(src_matched, tgt_matched)

        src = (R_step @ src.T).T + t_step
        R_total = R_step @ R_total
        t_total = R_step @ t_total + t_step

        mean_error = float(np.mean(dists[inliers]))
        if prev_error is not None and abs(prev_error - mean_error) < ICP_TOLERANCE:
            break
        prev_error = mean_error

    return R_total, t_total, mean_error, iters_run


def update(scan_data):
    global last_points, last_transform, state

    if scan_data is None:
        return

    cur_points = scan_to_points(scan_data)

    if (last_points is not None
            and len(cur_points) > ICP_MIN_MATCHES
            and len(last_points) > ICP_MIN_MATCHES):

        init_R = last_transform[0] if last_transform is not None else None
        init_t = last_transform[1] if last_transform is not None else None

        R, t, mean_error, iters_run = icp(
            cur_points, last_points, init_R=init_R, init_t=init_t
        )
        last_transform = (R, t)

        rotation_rad = math.atan2(R[1, 0], R[0, 0])
        rotation_deg = math.degrees(rotation_rad)

        rad_current = math.radians(state[2])
        global_dx = t[0] * math.cos(rad_current) - t[1] * math.sin(rad_current)
        global_dy = t[0] * math.sin(rad_current) + t[1] * math.cos(rad_current)

        state[0] += global_dx
        state[1] += global_dy
        state[2] = (state[2] + rotation_deg) % 360

        print(
            f"Pose: X={state[0]:.2f}, Y={state[1]:.2f}, Yaw={state[2]:.2f}\u00b0  "
            f"(ICP: {iters_run} iters, mean_err={mean_error:.4f} m)"
        )
    else:
        last_transform = None

    last_points = cur_points
    return global_dx, global_dy, rotation_deg % 360

def get_pose():
    global state
    return state

