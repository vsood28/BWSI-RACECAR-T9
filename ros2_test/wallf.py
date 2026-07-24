    ### Imports ###
import rclpy 
from rclpy.node import Node  

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np

### Classes ###

class WallFollower(Node):
    __PUBLISH_PERIOD_SEC = 0.05

    def __init__(self):
        super().__init__('wall_follower_node')

        self__samples = np.array([])
        
        # copied from drive_real.py
        self.__publisher = self.node.create_publisher(AckermannDriveStamped, self.__TOPIC, qos_profile=1)
        self.__message = AckermannDriveStamped()
        self.__message.header.frame_id = self.__FRAME_ID
        self.__max_speed = 0.50

        # make sure to initialize variables
        self.node.create_timer(self.__PUBLISH_PERIOD_SEC, self.__update)

        self.__attitude_sub = self.create_subscription(Vector3, '/attitude', self.attitude_callback, 10) # print
        self.__velocity_sub = self.create_subscription(Float32, '/velocity', self.velocity_callback, 10) # print
        self.__pose_sub = self.create_subscription(Pose, '/pose_estimate', self.pose_callback, 10) # print

        self.__lidar_sub = self.create_subscription(LaserScan, '/scan       ', self.scan_callback, 10)

        # remove warning flags
        self.__attitude_sub
        self.__velocity_sub
        self.__pose_sub
        self.__lidar_sub

        self.attitude_val = tuple() # should form () 
        self.velocity_val = 0.0
        self.pose_val = tuple()

    # data from the lidar - needs to be transformed
    """
    # LIDAR Scan returns value in meters, multiplying by 100 to be processed in cm
    # LIDAR Scan reversed, flipping order of data entry to correct for CW spin - matches with sim
    # For RPLidar - replace "inf" with 0 to match sim LIDAR data
    # Note: The real LIDAR returns a scan of length 1080.
    """
    def scan_callback(self, data):
        scan_data = np.flip(np.multiply(np.array(data.ranges), 100))
        self.__samples = np.array([0 if str(x) == "inf" else x for x in scan_data])

    # saving values in callbacks if they need to be used later
    def attitude_callback(self, data):
        self.attitude_val = (data.x, data.y, data.z) # these callbacks probably dont work 
    
    def velocity_callback(self, data):
        self.velocity_val = data # .value ?

    def pose_callback(self, data):
        self.pose_val = (data.x, data.y, data.theta)

    def set_speed_angle(self, speed: float, angle: float) -> None:
        assert (
            -1.0 <= speed <= 1.0
        ), f"speed [{speed}] must be between -1.0 and 1.0 inclusive."
        assert (
            -1.0 <= angle <= 1.0
        ), f"angle [{angle}] must be between -1.0 and 1.0 inclusive."

        self.__message.drive.speed = float(speed * self.__max_speed)
        # Negate angle so positive = right in the student API, matching the
        # sim convention; the throttle/pwm chain handles servo-side sign.
        self.__message.drive.steering_angle = float(-angle)

    # def set_max_speed(self, max_speed: float = 0.50) -> None:
    #     assert (
    #         0.0 <= max_speed <= 1.0
    #     ), f"max_speed [{max_speed}] mus  t be between 0.0 and 1.0 inclusive."

    #     self.__max_speed = max_speed

    def __update(self):
        """
        Stamps and publishes the current drive message. Called from the
        node's own 20 Hz timer rather than from racecar_core_real's run
        thread, so the publisher keeps running under both go() and
        go_async() regardless of run-loop pacing issues.
        """
        self.__message.header.stamp = self.node.get_clock().now().to_msg()
        self.__publisher.publish(self.__message)

def main():
    rclpy.init(args=None)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()