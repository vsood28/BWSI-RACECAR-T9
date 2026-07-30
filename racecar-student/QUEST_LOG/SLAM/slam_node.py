import rclpy as ros2
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from EKF import ExtendedKalmanFilter
from slam import SLAM
from occupancy_grid import OccupancyGrid as OG
from datetime import datetime
from slam_icp import ICPScanMatcher

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
            '/drive',
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

        self.icp = ICPScanMatcher()

        self.last_measure_callback = datetime.now()
        self.last_estimate_callback = datetime.now()

    def initialize(self):
        self.last_measure_callback = datetime.now()
        self.last_estimate_callback = datetime.now()

    def measure_callback(self, data):
        now = datetime.now()

        dt = (now - self.last_measure_callback).total_seconds()
        self.last_measure_callback = now

        pose = self.icp.update(
            data.ranges,
            angle_min=data.angle_min,
            angle_increment=data.angle_increment,
            range_min=data.range_min,
            range_max=data.range_max
        )

        self.slam.measurement_callback(
            pose.to_array(),
            dt,
        )

    def estimate_callback(self, data):
        v = float(data.data)
        omega = float(self.angle_cache)

        now = datetime.now()

        dt = (now - self.last_estimate_callback).total_seconds()

        self.slam.estimate_callback(
            (v, omega),
            dt,
        )

        self.last_estimate_callback = now

    def servo_callback(self, data):
        self.angle_cache = float(data.data)

    def get_map_estiamte(self):
        return self.slam.get_map()

    def get_state_estimate(self):
        return self.slam.occupancy_grid.to_ros_occupancy_grid(frame_id="map", origin_x=0.0, origin_y=0.0, stamp=self.get_clock().now().to_msg())