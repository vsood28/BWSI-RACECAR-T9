from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from ekf import ExtendedKalmanFilter
from slam import SLAM
from occupancy_grid import OccupancyGrid as OG
from datetime import datetime
from slam_icp import ICPScanMatcher
import numpy as np

from ekf_models import steering_model, velocity_model

#units of meters (from lidar), angular velocity (rads) for encoder,  



def laserscan_to_points(scan_msg: LaserScan) -> np.ndarray: #helper
    ranges = np.asarray(scan_msg.ranges, dtype=np.float32)

    #compute the angle for every beam
    angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment

    #leep only valid measurements
    valid = (
        np.isfinite(ranges)
        & (ranges >= scan_msg.range_min)
        & (ranges <= scan_msg.range_max)
    )

    ranges = ranges[valid]
    angles = angles[valid]

    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)

    return np.column_stack((x, y))

class StateEstimationNode(Node):
    def __init__(self, initial_state, initial_covariance, models, jacobians, grid_params, grid_odds, sys_params, publish_rate=10.0):
        super().__init__('state_estimator')

        self.measure_sub = self.create_subscription( #sub to lidar, for measure update (acucate and lsow)
            LaserScan,
            '/scan',
            self.measure_callback,
            10
        )

        self.encoder_sub = self.create_subscription( #sub to encoder, for measures
            Float32,
            '/encoder/speed',
            self.estimate_callback,
            10
        )

        self.servo_sub = self.create_subscription( #sub to drive, to read servo steering angle. this model assumes servo achives angle instantenly
            AckermannDriveStamped,
            '/drive',
            self.servo_callback,
            10
        )

        self.pose_pub = self.create_publisher( #topic to publish pose est
            Pose2D,
            '/state_estimate/pose', 
            10
        )
        self.map_pub = self.create_publisher( #topic to publish map est
            OccupancyGrid, 
            '/state_estimate/map', 
            10
        )


        self.angle_cache = 0 # cahce for steering angle to coordinate both steering angle and encoder speed, since come in at different tiems

        sc, mc, pc = initial_covariance.values() #unpack everything 

        sm, mm = models.values()

        sj, mj, pj = jacobians.values()
        
        ekf = ExtendedKalmanFilter(initial_state, sc, pc, mc, sj, mj, pj, sm, mm) #create ekf with all the models

        w, h, res = grid_params.values()

        po, poh, pom = grid_odds.values()

        occ = OG(w, h, res, po, poh, pom) #create occupancy

        self.slam = SLAM(ekf, occ, sys_params) #create slam with the ekf and occgrid, as well as sysparams (wheel base etc)

        self.icp = ICPScanMatcher() #create icp, used to turn lidar data to useful measurement model (abs pose)

        self.last_measure_callback = datetime.now() #for dt calculations 
        self.last_estimate_callback = datetime.now()

        self.publish_timer = self.create_timer( #publishtimer that calls publishing at rate
            1.0 / publish_rate,
            self.publish_state_estimate
        )

    def initialize(self): #reintialize everything 
        self.last_measure_callback = datetime.now()
        self.last_estimate_callback = datetime.now()

    def measure_callback(self, data): #run when lidar measurement (accurate) comes in
        now = datetime.now()

        dt = (now - self.last_measure_callback).total_seconds() #get dt
        self.last_measure_callback = now

        point_cloud = laserscan_to_points(data) #turn to useful pointcloud instaed of noisy lidar data

        pose = self.icp.update(point_cloud) #get icp pose 

        self.slam.measurement_callback( #update everything based on measurement (math handled in slam class)
            pose.to_array(),
            dt,
        )

        self.slam.update_occupancy_grid(point_cloud) #use pointcloud to update, as well as current updated pose

    def estimate_callback(self, data): #run when encoder data comes in
        v = velocity_model(float(data.data)) #convert from angular to linear
        omega = float(self.angle_cache) #read from steering angle chache

        now = datetime.now() #for dt

        dt = (now - self.last_estimate_callback).total_seconds() #get delta_t
        self.last_estimate_callback = now

        self.slam.estimate_callback( #slam runs its estiamte callback
            (v, omega),
            dt,
        )

        

    def servo_callback(self, data): #store servo steering when gets
        self.angle_cache = steering_model(float(data.drive.steering_angle))

    def get_map_estimate(self): #ros2 occgrid getter
        return self.slam.occupancy_grid.to_ros_occupancy_grid(frame_id="map", origin_x=0.0, origin_y=0.0, stamp=self.get_clock().now().to_msg())

    def get_state_estimate(self): #ros2 se getter
        state = self.slam.get_state_estimate()

        x = float(state[0])
        y = float(state[1])
        theta = float(state[2])

        pose_msg = Pose2D()
        pose_msg.x = x
        pose_msg.y = y
        pose_msg.theta = theta

        return pose_msg

    def publish_state_estimate(self): #publish both pose map, callback that is on the clock
        self.pose_pub.publish(self.get_state_estimate())
        self.map_pub.publish(self.get_map_estimate())
