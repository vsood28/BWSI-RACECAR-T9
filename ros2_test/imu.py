### Imports ###

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from geometry_msgs.msg import Pose2D, Vector3
from sensor_msgs.msg import Imu, MagneticField # does magnetometer work like this ?
from std_msgs.msg import Float32

import filters
import time
import math

### Classes ###
# self = global

# what is this parameter
class ImuNode(Node):
    def __init__(self): 
        super().__init__('imu_publisher_node')
        
        # is this the right hz
        self.declare_parameter('pubilsh_rate_hz', 60.0)

        # create_publisher parameters: msg type (Class), topic (str), quality (10)
        self.__attitude_pub = self.create_publisher(Vector3, '/attitude', 10)
        self.__velocity_pub = self.create_publisher(Float32, '/velocity', 10)
        self.__pose_est_pub = self.create_publisher(Pose2D, '/pose_estimate', 10)

        # initializing / type of messages to publish
        self.__attitude_message = Vector3()
        self.__velocity_message = Float32()
        self.__pose_est_message = Pose2D()

        # Filters and extra useful variables + usages
        self.kf1_velocity = filters.KalmanFilter(1, 0) # linear velocity - we can switch to 1 if neededd
        self.kf2_posx = filters.KalmanFilter(0.1, 0) # position (x)
        self.kf3_posz = filters.KalmanFilter(0.1, 0) # position (y)

        # globals initialized here (?)
        self.old_time = 0.0

        self.velocity_x = 0.0 
        self.velocity_z = 0.0   

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0  

        self.position_x = 0.0
        self.position_z = 0.0

        self.mx = 0.0
        self.my = 0.0       
        self.mz = 0.0
                
        # alpha value: trust to put into gyroscope
        self.compf1_att = filters.ComplementaryFilter(0.95, self.roll, self.pitch, self.yaw) # attitude
        self.compf2_theta = filters.ComplementaryFilter(0.95, 0.0, 0.0, 0.0) # theta

        qos = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # create_subscription parameters: 
        # msg_type (Class), topic (str), callback (function), qos_profile (quality: int)
        self.__imu_sub = self.create_subscription(Imu, '/imu/fused', self.imu_fused_callback, qos)
        self.__mag_sub = self.create_subscription(MagneticField, '/mag', self.mag_callback, qos)

        # remove the warning flags ig
        self.__imu_sub
        self.__mag_sub


    def imu_fused_callback(self, data): # called every time something is published
        if self.old_time == 0.0:
            self.old_time = time.time() 
            return

        new_time = time.time()
        dt = new_time - self.old_time # dt for all integration
        self.old_time = new_time
        
        ########################## LINEAR VELOCITY ##########################

        # integrating acceleration values
        self.velocity_x = self.velocity_x + data.linear_acceleration.x * dt
        # self.velocity_y = self.velocity_y + data.linear_acceleration.y * dt 
        # # y should technically be equal to zero - maybe ignorable?
        self.velocity_z = self.velocity_z + data.linear_acceleration.z * dt

        # combines velocity into a scalar 
        self.velocity_scalar = self.velocity_x ** 2 + self.velocity_z ** 2

        # passing velocity into a kalman filter
        self.__velocity_message.data = self.kf1_velocity.update(self.velocity_scalar)

        # publishing final scalar velocity
        self.__velocity_pub.publish(self.__velocity_message)

        ############################## ATTITUDE ###############################

        # integrating angular velocity values
        self.roll = self.roll + data.angular_velocity.x * dt
        self.pitch = self.pitch + data.angular_velocity.y * dt
        self.yaw = self.yaw + data.angular_velocity.z * dt

        # passing values into a complementary filter 
        at_x, at_y, at_z, _ = self.compf1_att.update( 
                           data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z, 
                           data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z,
                           0.0, 0.0, dt)
    
        # taking the values out of complementary filter and assigning them to the message data
        self.__attitude_message.x = at_x
        self.__attitude_message.y = at_y
        self.__attitude_message.z = at_z

        # publishing final attitude components
        self.__attitude_pub.publish(self.__attitude_message)

        ############################### POSE ####################################
        # x,y = x,z because y has the gravity acceleration in it for some reason
        
        # integrating linear velocity values
        self.position_x = self.position_x + self.velocity_x * dt
        self.position_z = self.position_z + self.velocity_z * dt

        # updating mx and mz to account for tilt
        new_mx = (self.mx * math.cos(self.pitch)) 
        + (self.my * math.sin(self.roll) * math.sin(self.pitch)) 
        + (self.mz * math.cos(self.roll) * math.sin(self.pitch)) 

        new_my = (self.my * math.cos(self.roll))
        - (self.mz * math.sin(self.roll))

        # passing theta into complementary filter
        _, _, _, final_theta = self.compf1_att.update( 
                                            0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0,
                                            new_my, new_mx, dt)

        # assigning various values to message data
        self.__pose_est_message.x = self.kf2_posx.update(self.position_x)
        self.__pose_est_message.y = self.kf3_posz.update(self.position_z)
        self.__pose_est_message.theta = final_theta

        # publishing final 2d pose values
        self.__pose_est_pub.publish(self.__pose_est_message)


    def mag_callback(self, data):
        self.mx = data.magnetic_field.x 
        self.my = data.magnetic_field.y 
        self.mz = data.magnetic_field.z 


def main():
    rclpy.init(args=None)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()