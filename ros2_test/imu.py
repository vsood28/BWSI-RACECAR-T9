### Imports ###

import rclpy 
from rclpy.node import Node  

from geometry_msgs.msg import Pose2D, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

import filters
import math
import time

### Classes ###

# what is the parameter of this class
class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_publisher_node')
        
        self.declare_parameter('pubilsh_rate_hz', 60.0)

        # create_publisher parameters: msg type (Class), topic (str), quality (10)
        self.__attitude_pub = self.create_publisher(Vector3, '/attitude', 10)
        self.__velocity_pub = self.create_publisher(Float32, '/velocity', 10)
        self.__pose_est_pub = self.create_publisher(Pose2D, '/pose_estimate', 10)

        self.__attitude_message = Vector3()
        self.__velocity_message = Float32()
        self.__pose_est_message = Pose() # ?

        self.kf1_velocity = filters.KalmanFilter()
        self.kf2 = filters.KalmanFilter()
        self.kf3 = filters.KalmanFilter()
        
        self.comp1 = filters.ComplementaryFilter()

        self.dt = 0.0

        # im confused
        self.create_subscription(Imu, '/imu/fused', self.imu_fused_callback, 10)


    def imu_fused_callback(self, data): # called every time something is published
        # self = global
        self.dt = time.time() - self.dt

        # linear velocity:
        self.velocity_x = self.velocity_x + data.linear_acceleration.x * self.dt
        # y should be equal to 0 - ignorable?
        self.velocity_z = self.velocity_z + data.linear_acceleration.z * self.dt

        self.velocity_scalar = self.velocity_x ** 2 + self.velocity_z ** 2
        
        self.velocity_final

        self.__velocity_message.data 
        self.__velocity_pub.publish(self.__velocity_message)

        # attitude:
        self.

        # pose: x,y = x,z

        data.angular_velocity.x
        data.angular_velocity.y
        data.angular_velocity.z



        self.__attitude_message.x = 0
        self.__attitude_message.y = 0
        self.__attitude_message.z = 0
        self.__attitude_pub.publish(self.__attitude_message)

def main():
    rclpy.init(args=None)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()