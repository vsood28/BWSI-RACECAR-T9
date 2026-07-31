import math
import numpy as np
import pytest

from slam_icp import ICPScanMatcher, Pose


@pytest.fixture
def matcher():
    return ICPScanMatcher()


@pytest.fixture
def square_cloud():
    return np.array([
        [0.0, 0.0],
        [1.0, 0.0],
        [1.0, 1.0],
        [0.0, 1.0],
        [0.5, 0.5],
        [2.0, 0.0],
        [2.0, 1.0],
        [0.0, 2.0],
        [1.0, 2.0],
        [2.0, 2.0],
        [0.5, 1.5],
        [1.5, 0.5],
    ])

def test_pose_to_array():
    pose = Pose(1.0, 2.0, 3.0)
    np.testing.assert_array_equal(
        pose.to_array(),
        np.array([1.0, 2.0, 3.0])
    )

def test_scan_to_points_empty():
    pts = ICPScanMatcher.scan_to_points([])
    assert pts.shape == (0, 2)


def test_scan_to_points_known_angles():
    scan = [1.0, 1.0, 1.0, 1.0]

    pts = ICPScanMatcher.scan_to_points(
        scan,
        angle_min=0,
        angle_increment=math.pi / 2,
    )

    expected = np.array([
        [1, 0],
        [0, 1],
        [-1, 0],
        [0, -1],
    ])

    np.testing.assert_allclose(pts, expected, atol=1e-6)


def test_scan_to_points_invalid_ranges():
    scan = [
        np.nan,
        np.inf,
        20.0,
        0.01,
        1.0,
    ]

    pts = ICPScanMatcher.scan_to_points(
        scan,
        angle_min=0,
        angle_increment=0.5,
    )

    assert np.allclose(pts[:4], 0)
    assert not np.allclose(pts[4], 0)


# ---------------------------------------------------------------------
# normalize angle
# ---------------------------------------------------------------------

@pytest.mark.parametrize("angle", [
    0,
    math.pi,
    -math.pi,
    5 * math.pi,
    -7 * math.pi,
    123.4,
])
def test_normalize_angle_range(angle):
    wrapped = ICPScanMatcher._normalize_angle(angle)
    assert -math.pi <= wrapped <= math.pi


# ---------------------------------------------------------------------
# best fit transform
# ---------------------------------------------------------------------

def test_best_fit_transform_identity(square_cloud):
    R, t = ICPScanMatcher.best_fit_transform(square_cloud, square_cloud)

    np.testing.assert_allclose(R, np.eye(2), atol=1e-8)
    np.testing.assert_allclose(t, np.zeros(2), atol=1e-8)


def test_best_fit_transform_known(square_cloud):
    theta = np.deg2rad(30)

    R_true = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)],
    ])

    t_true = np.array([1.2, -0.4])

    transformed = (R_true @ square_cloud.T).T + t_true

    R, t = ICPScanMatcher.best_fit_transform(square_cloud, transformed)

    np.testing.assert_allclose(R, R_true, atol=1e-6)
    np.testing.assert_allclose(t, t_true, atol=1e-6)


# ---------------------------------------------------------------------
# nearest neighbors
# ---------------------------------------------------------------------

def test_nearest_neighbors():
    src = np.array([
        [0.1, 0.1],
        [2.2, 0.0],
    ])

    tgt = np.array([
        [0, 0],
        [2, 0],
    ])

    dists, idx = ICPScanMatcher.get_nearest_neighbors(src, tgt)

    assert idx[0] == 0
    assert idx[1] == 1

    assert pytest.approx(dists[0]) == np.linalg.norm([0.1, 0.1])


# ---------------------------------------------------------------------
# ICP
# ---------------------------------------------------------------------

def test_icp_identity(matcher, square_cloud):
    R, t, err, iters = matcher.icp(square_cloud, square_cloud)

    np.testing.assert_allclose(R, np.eye(2), atol=1e-6)
    np.testing.assert_allclose(t, np.zeros(2), atol=1e-6)

    assert err < 1e-8
    assert iters >= 1


def test_icp_known_transform(matcher, square_cloud):
    theta = np.deg2rad(15)

    R_true = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)],
    ])

    t_true = np.array([0.25, -0.15])

    target = (R_true @ square_cloud.T).T + t_true

    R, t, err, _ = matcher.icp(square_cloud, target)

    np.testing.assert_allclose(R, R_true, atol=1e-3)
    np.testing.assert_allclose(t, t_true, atol=1e-3)

    assert err < 1e-3


# ---------------------------------------------------------------------
# update()
# ---------------------------------------------------------------------

def test_update_initializes(matcher, square_cloud):
    pose = matcher.update(square_cloud)

    assert pose.x == 0
    assert pose.y == 0
    assert pose.theta == 0

    assert matcher.last_points is not None


def test_update_insufficient_matches(matcher):
    pts = np.random.rand(5, 2)

    matcher.update(pts)

    pose = matcher.update(pts)

    assert pose.x == 0
    assert pose.y == 0
    assert pose.theta == 0


# ---------------------------------------------------------------------
# End-to-end noisy ICP pose estimation
# ---------------------------------------------------------------------

def test_icp_recovers_noisy_transform():
    """
    Create a random point cloud.

    Apply a known rigid transform.

    Add Gaussian noise.

    Verify ICP recovers approximately the known transform.
    """

    np.random.seed(42)

    matcher = ICPScanMatcher()

    matcher.ICP_MAX_CORR_DIST = 2.0
    matcher.ICP_MIN_MATCHES = 10

    points = np.random.uniform(-2, 2, size=(200, 2))

    theta = np.deg2rad(20)

    R_true = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)],
    ])

    t_true = np.array([0.7, -0.4])

    transformed = (R_true @ points.T).T + t_true

    noise = np.random.normal(0, 0.01, transformed.shape)

    transformed += noise

    R_est, t_est, err, _ = matcher.icp(points, transformed)

    theta_est = math.atan2(R_est[1, 0], R_est[0, 0])

    assert err < 0.03

    np.testing.assert_allclose(
        t_est,
        t_true,
        atol=0.03,
    )

    assert theta_est == pytest.approx(theta, abs=np.deg2rad(1.0))


# ---------------------------------------------------------------------
# Verify update() returns robot motion
# ---------------------------------------------------------------------

def test_update_pose_estimate_matches_motion():
    """
    update() returns the accumulated robot world pose.
    """

    np.random.seed(1)

    matcher = ICPScanMatcher()
    matcher.ICP_MAX_CORR_DIST = 2.0

    cloud = np.random.uniform(-3, 3, (250, 2))

    # First scan initializes the matcher.
    matcher.update(cloud)

    theta = np.deg2rad(10)

    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)],
    ])

    t = np.array([0.4, 0.2])

    # Second scan (current scan)
    moved = (R @ cloud.T).T + t
    moved += np.random.normal(0, 0.005, moved.shape)

    pose = matcher.update(moved)

    R_inv = R.T
    t_inv = -R_inv @ t
    theta_inv = theta #since robot theta is flipped, +theta in cw instead of ccw, so +theta from coordinate plane equal and opposite = +theta  robot plane

    assert pose.x == pytest.approx(t_inv[0], abs=0.03)
    assert pose.y == pytest.approx(t_inv[1], abs=0.03)
    assert pose.theta == pytest.approx(theta_inv, abs=np.deg2rad(1.0))
