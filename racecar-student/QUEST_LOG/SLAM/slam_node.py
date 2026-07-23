import rclpy as ros2
from rclpy.node import Node
from EKF import ExtendedKalmanFilter
from slam import SLAM
from occupancy_grid import OccupancyGrid


class StateEstimationNode(Node):

    def __init__(self, initial_state, initial_covariance, models, jacobians, grid_params, grid_odds, sys_params):
        super().__init__('state_estimator')

        self.measure_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.measure_callback,
            10
        )

        self.encoder_sub = self.create_subscription(
            Float32,
            '/encoder/speed',
            self.estimate_callback,
            10
        )

        self.control_sub = self.create_subscription(
            Float32,
            '/controller/angle',
            self.controller_callback,
            10
        )

        self.angle_cache = 0

        sc, mc, pc = initial_covariance.values()

        sm, mm, cm = models.values()

        sj, mj, pj = jacobians.values()

        self.controller_model = cm

        ekf = ExtendedKalmanFilter(initial_state, sc, pc, mc, sj, mj, pj, sm, mm)

        w, h, res = grid_params.values()

        po, poh, pom = grid_odds.values()

        occ = OccupancyGrid(w, h, res, po, poh, pom)

        self.slam = SLAM(ekf, occ, sys_params)        

    def measure_callback(self, data):
        pass

    def estimate_callback(self, data): #
        v, omega = self.control_model(data, self.angle_cache)

    def controller_callback(self, data):
        self.angle_cache = data