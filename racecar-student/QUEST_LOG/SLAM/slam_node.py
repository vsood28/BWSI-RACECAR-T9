import rclpy as ros2
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from EKF import ExtendedKalmanFilter
from slam import SLAM
from occupancy_grid import OccupancyGrid as OG
from datetime import datetime


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

        self.servo_sub = self.create_subscription(
            Float32,
            '',
            self.servo_callback,
            10
        )

        self.angle_cache = 0

        sc, mc, pc = initial_covariance.values()

        sm, mm = models.values()

        sj, mj, pj = jacobians.values()
        
        ekf = ExtendedKalmanFilter(initial_state, sc, pc, mc, sj, mj, pj, sm, mm)

        w, h, res = grid_params.values()

        po, poh, pom = grid_odds.values()

        occ = OG(w, h, res, po, poh, pom)

        self.slam = SLAM(ekf, occ, sys_params)        

        self.last_measure_callback = datetime.now()
        self.last_esitmate_callback = datetime.now()

    def initalize(self):
        self.last_measure_callback = datetime.now()
        self.last_esitmate_callback = datetime.now()

    def measure_callback(self, data):
        self.icp.update_state(data)
        t = datetime.now()
        
        self.slam.measurement_callback(self.icp.state_estimate, self.last_measure_callback - t)

        self.last_measure_callback = t

    def estimate_callback(self, data): #
        v = data
        omega = self.angle_cache
        t = datetime.now()

        self.slam.estimate_callback((v, omega), self.last_esitmate_callback - t)

        self.last_esitmate_callback = t

    def servo_callback(self, data):
        self.angle_cache = data

    def get_map_estiamte(self):
        return self.slam.get_map()

    def get_state_estimate(self):
        return self.slam.occupancy_grid.to_ros_occupancy_grid(frame_id="map", origin_x=0.0, origin_y=0.0, stamp=self.get_clock().now().to_msg() )