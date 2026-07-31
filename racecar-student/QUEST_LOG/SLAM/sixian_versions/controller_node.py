import time
import math

from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid
from ackermann_msgs.msg import AckermannDriveStamped

from astar import AStarPlanner

#units of meters for grid resolution

class PID:  # pid class very simple
    def __init__(self, kP=0, kI=0, kD=0):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.prev_error = 0
        self.cum_i_val = 0
        self.prev_tick_called = 0

    def reset(self):
        self.prev_error = 0
        self.cum_i_val = 0
        self.prev_tick_called = 0

    def tick(self, setpoint, val, reset=False):
        if reset:
            self.reset()

        error = val - setpoint
        dt = time.perf_counter() - self.prev_tick_called

        p = self.kP * error
        self.cum_i_val += self.kI * error * dt
        d = self.kD * (error - self.prev_error) / dt if dt > 0 else 0.0

        self.prev_error = error
        self.prev_tick_called = time.perf_counter()

        return p + self.cum_i_val + d


class SLAMController(Node):
    def __init__(self, goal_xy, lookahead_cells=5, pid_gains=(1.0, 0.0, 0.0),
                 obstacle_threshold=60, control_rate=10.0, base_speed=0.5,
                 max_steering_angle=0.4):
        super().__init__('slam_controller')

        self.pose_sub = self.create_subscription(
            Pose2D,
            '/state_estimate/pose',
            self.pose_callback,
            10
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/state_estimate/map',
            self.map_callback,
            10
        )

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.goal_xy = goal_xy
        self.lookahead_cells = lookahead_cells
        self.obstacle_threshold = obstacle_threshold
        self.base_speed = base_speed
        self.max_steering_angle = max_steering_angle

        kP, kI, kD = pid_gains
        self.pid = PID(kP=kP, kI=kI, kD=kD)

        self.current_pose = None
        self.current_map = None

        self.control_timer = self.create_timer(
            1.0 / control_rate,
            self.control_loop
        )

    def pose_callback(self, data):
        self.current_pose = data

    def map_callback(self, data):
        self.current_map = data

    def _get_lookahead_target(self, path): #walk lookahead along planned path
        idx = min(self.lookahead_cells, len(path) - 1)
        return path[idx]

    def _world_to_body(self, dx, dy, theta): #helper to get to world
        forward = dx * math.cos(theta) + dy * math.sin(theta)
        right = dx * math.sin(theta) - dy * math.cos(theta)
        return right, forward

    def control_loop(self):
        if self.current_pose is None or self.current_map is None:
            return

        try:
            planner = AStarPlanner(
                self.current_map,
                obstacle_threshold=self.obstacle_threshold,
                allow_unknown=False,  # unknown cells treated as impassable
            )

            path = planner.plan(
                (self.current_pose.x, self.current_pose.y),
                self.goal_xy,
            )
        except ValueError as e:
            self.get_logger().warn(f"Planning failed: {e}")
            self._publish_stop()
            return

        if path is None or len(path) < 2:
            self.get_logger().warn("No path found to goal.")
            self._publish_stop()
            return

        target_x, target_y = self._get_lookahead_target(path)

        dx = target_x - self.current_pose.x
        dy = target_y - self.current_pose.y

        right, forward = self._world_to_body(dx, dy, self.current_pose.theta)

        heading_error = math.atan2(right, forward)

        steering_command = self.pid.tick(0.0, heading_error)
        steering_command = max(
            -self.max_steering_angle,
            min(self.max_steering_angle, steering_command)
        )

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = float(steering_command)
        drive_msg.drive.speed = float(self.base_speed)

        self.drive_pub.publish(drive_msg)

    def _publish_stop(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = 0.0
        drive_msg.drive.speed = 0.0
        self.drive_pub.publish(drive_msg)