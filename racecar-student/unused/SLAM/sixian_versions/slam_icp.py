import numpy as np
import math
from dataclasses import dataclass
from scipy.spatial import cKDTree

@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # radians

    def to_array(self):
        return np.array([self.x, self.y, self.theta])

class ICPScanMatcher:

    ICP_MAX_ITERS = 25
    ICP_TOLERANCE = 1e-5
    ICP_MAX_CORR_DIST = 0.5
    ICP_MIN_MATCHES = 10

    def __init__(self):
        self.last_points = None
        self.last_transform = None
        self.state = Pose(0.0, 0.0, 0.0)

    # ---------- static/pure helpers ----------

    @staticmethod
    def scan_to_points(scan_data, angle_min=-math.pi, angle_increment=None,
                    range_min=0.05, range_max=12):
        """
        Convert scan ranges in meters into Nx2 Cartesian points.

        Parameters
        ----------
        scan_data:
            Iterable of ranges in meters.
        angle_min:
            Angle of the first measurement in radians.
        angle_increment:
            Angular separation between measurements in radians.
        range_min:
            Minimum valid range.
        range_max:
            Maximum valid range.
        """
        points = []

        num_samples = len(scan_data)

        if num_samples == 0:
            return np.empty((0, 2))

        if angle_increment is None:
            angle_increment = (2.0 * math.pi) / num_samples

        for i, dist in enumerate(scan_data):

            if not np.isfinite(dist) or dist < range_min or dist > range_max: #inv becomes 0
                dist = 0.0

            angle = angle_min + i * angle_increment

            x = dist * math.cos(angle)
            y = dist * math.sin(angle)

            points.append([x, y])

        return np.asarray(points, dtype=float)

    @staticmethod
    def get_nearest_neighbors(source, target):
        tree = cKDTree(target)
        dists, idx = tree.query(source, k=1)
        return dists, idx

    @staticmethod
    def best_fit_transform(A, B):
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)

        AA = A - centroid_A
        BB = B - centroid_B

        H = AA.T @ BB

        U, S, Vt = np.linalg.svd(H)

        R = Vt.T @ U.T

        if np.linalg.det(R) < 0:
            Vt[-1] *= -1
            R = Vt.T @ U.T

        t = centroid_B - R @ centroid_A

        return R, t

    @staticmethod
    def _normalize_angle(angle):
        """Wrap an angle (radians) into (-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    # ---------- ICP core ----------

    def icp(self, source, target, init_R=None, init_t=None):
        R_total = np.eye(2) if init_R is None else init_R.copy()
        t_total = np.zeros(2) if init_t is None else init_t.copy()

        src = (R_total @ source.T).T + t_total

        prev_error = None
        mean_error = None

        for iters in range(self.ICP_MAX_ITERS):

            dists, idx = self.get_nearest_neighbors(src, target)

            inliers = dists <= self.ICP_MAX_CORR_DIST

            if np.count_nonzero(inliers) <= self.ICP_MIN_MATCHES:
                break

            src_match = src[inliers]
            tgt_match = target[idx[inliers]]

            R_step, t_step = self.best_fit_transform(src_match, tgt_match)

            src = (R_step @ src.T).T + t_step

            R_total = R_step @ R_total
            t_total = R_step @ t_total + t_step

            mean_error = np.mean(dists[inliers])

            if prev_error is not None and abs(prev_error - mean_error) < self.ICP_TOLERANCE:
                break

            prev_error = mean_error

        return R_total, t_total, mean_error, iters + 1

    # ---------- public API ----------

    def update(self, cur_points):
        if (
            self.last_points is None
            or len(cur_points) <= self.ICP_MIN_MATCHES
            or len(self.last_points) <= self.ICP_MIN_MATCHES
        ):
            self.last_points = cur_points
            self.last_transform = None
            return self.state

        init_R = self.last_transform[0] if self.last_transform is not None else None
        init_t = self.last_transform[1] if self.last_transform is not None else None

        R, t, mean_error, iters = self.icp(
            cur_points,
            self.last_points,
            init_R,
            init_t,
        )

        self.last_points = cur_points

        if mean_error is None:
            self.last_transform = None
            print("ICP match failed (insufficient inliers); pose not updated.")
            return self.state

        self.last_transform = (R, t)

        dtheta = math.atan2(R[1, 0], R[0, 0])
        dx, dy = t[0], t[1]

        theta = self.state.theta
        cos_t, sin_t = math.cos(theta), math.sin(theta)

        new_x = self.state.x + dx * cos_t - dy * sin_t
        new_y = self.state.y + dx * sin_t + dy * cos_t
        new_theta = self._normalize_angle(theta - dtheta)

        self.state = Pose(new_x, new_y, new_theta)

        return self.state


    def get_pose(self):
        return self.state