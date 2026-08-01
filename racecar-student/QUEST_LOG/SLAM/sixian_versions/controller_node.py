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
        dt = time.perf_counter() - self.prev_tick_called #dt

        p = self.kP * error #p term
        self.cum_i_val += self.kI * error * dt #iterm
        d = self.kD * (error - self.prev_error) / dt if dt > 0 else 0.0 #dterm

        self.prev_error = error
        self.prev_tick_called = time.perf_counter() #update for dt calcs

        return p + self.cum_i_val + d


class SLAMController(Node):
    def __init__(self, goal_xy, lookahead_cells=5, pid_gains=(1.0, 0.0, 0.0), #buncha params, configurbale
                 obstacle_threshold=60, control_rate=10.0, base_speed=0.5,
                 max_steering_angle=0.4):
        super().__init__('slam_controller')

        self.pose_sub = self.create_subscription( #read from state estimate pose, so we know where we are
            Pose2D,
            '/state_estimate/pose',
            self.pose_callback,
            10
        )

        self.map_sub = self.create_subscription( #read from state estimate map, so we know where obstacles are 
            OccupancyGrid,
            '/state_estimate/map',
            self.map_callback,
            10
        )

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10) #push commands to drive topic which should control servo/motor (doesnt seem to for now, not sure why)

        self.goal_xy = goal_xy #store params in self
        self.lookahead_cells = lookahead_cells
        self.obstacle_threshold = obstacle_threshold
        self.base_speed = base_speed
        self.max_steering_angle = max_steering_angle

        kP, kI, kD = pid_gains
        self.pid = PID(kP=kP, kI=kI, kD=kD)

        self.current_pose = None
        self.current_map = None

        self.control_timer = self.create_timer( #timer to call control loop at rate
            1.0 / control_rate,
            self.control_loop
        )

    def pose_callback(self, data): #when i get new pose (dtype pose2d)
        self.current_pose = data #update

    def map_callback(self, data): #when i get new map (dtype occupancygrid)
        self.current_map = data #update

    def _get_lookahead_target(self, path): #walk lookahead along planned path
        idx = min(self.lookahead_cells, len(path) - 1) #index of lookahead point, plus bounds protection
        return path[idx] #give that point

    def _world_to_body(self, dx, dy, theta): #helper to get to world coordinates
        forward = dx * math.cos(theta) + dy * math.sin(theta)
        right = dx * math.sin(theta) - dy * math.cos(theta)
        return right, forward

    def control_loop(self): #main loop function that runs at hz rate
        if self.current_pose is None or self.current_map is None: #not enough data
            return

        try: #attempt to plan path
            planner = AStarPlanner( #init astar
                self.current_map,
                obstacle_threshold=self.obstacle_threshold,
                allow_unknown=True,  # unknown cells treated as passable, so it tries to go trhough uknown to explore
            )

            path = planner.plan( #get path
                (self.current_pose.x, self.current_pose.y),
                self.goal_xy,
            )
        except ValueError as e: #path invalid, or other error
            self.get_logger().warn(f"Planning failed: {e}")
            self._publish_stop()
            return

        if path is None or len(path) < 2: #bad path
            self.get_logger().warn("No path found to goal.")
            self._publish_stop()
            return

        target_x, target_y = self._get_lookahead_target(path) #get lookahead target

        dx = target_x - self.current_pose.x
        dy = target_y - self.current_pose.y

        right, forward = self._world_to_body(dx, dy, self.current_pose.theta) #get target in body frame, to calc steering targets

        heading_error = math.atan2(right, forward) #heading error (angle to target)

        steering_command = self.pid.tick(0.0, heading_error) #steering based on heading error and pid
        steering_command = max( #clamp
            -self.max_steering_angle,
            min(self.max_steering_angle, steering_command)
        )

        drive_msg = AckermannDriveStamped() #create msg
        drive_msg.header.stamp = self.get_clock().now().to_msg() #stamp with time
        drive_msg.drive.steering_angle = float(steering_command) #set angle

        s = self.base_speed #temp var

        if abs(dx) < 0.1 and abs(dy) < 0.1: #if close, dont keep moving (lowkey a hack just to test if it works)
            s = 0 #stop

        drive_msg.drive.speed = float(s) #set speed

        self.drive_pub.publish(drive_msg) #publish; note: will use speed controller in addition in future, but just testing basics now

    def _publish_stop(self): #helper to publish a stop moving command
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = 0.0
        drive_msg.drive.speed = 0.0
        self.drive_pub.publish(drive_msg)