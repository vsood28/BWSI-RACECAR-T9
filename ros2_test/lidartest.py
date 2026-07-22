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
        super().__init__('lidar_test_node')

        self__samples = np.array([])

        self.node.create_timer(self.__PUBLISH_PERIOD_SEC, self.__update) # possible that node keyword needs to be removed 

        self.__lidar_sub = self.create_subscription(LaserScan, '/scan', self.__scan_callback, 10)

    def __scan_callback(self, data):
        scan_data = np.flip(np.multiply(np.array(data.ranges), 100))
        self.__samples = np.array([0 if str(x) == "inf" else x for x in scan_data])


def main():
    rclpy.init(args=None)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()