# imu node to publish /attitude, /velocity, /pose_estimate
# had help from sixian with math

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
import numpy as np

### Classes ###
# self = global

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
        self.kf1_velocity = filters.KalmanFilter(1, 0) # linear velocity - we can switch to 0.1 if needed
        self.kf2_posx = filters.KalmanFilter(0.1, 0) # position (x)
        self.kf3_posz = filters.KalmanFilter(0.1, 0) # position (y)

        # globals initialized here
        self.old_time = 0.0
        self.velocity_x = 0.0 
        self.velocity_y = 0.0           
        self.velocity_z = 0.0   
        
        self.velocity_vector = np.array([self.velocity_x, self.velocity_y, self.velocity_z])

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

        # remove the warning flags 
        self.__imu_sub
        self.__mag_sub


    # called every time something is published
    def imu_fused_callback(self, data): 
        new_time = time.time()

        if self.old_time == 0.0:
            self.old_time = new_time
            return

        dt = new_time - self.old_time # dt for all integration
        self.old_time = new_time

        self.wx = data.angular_velocity.x + 0.01117
        self.wy = data.angular_velocity.y - 0.01299
        self.wz = data.angular_velocity.z - 0.02923
        
        self.ax = data.linear_acceleration.x - 0.00382
        self.ay = data.linear_acceleration.y - 0.38592 # needs to zero out gravity not go to 9.8
        self.az = data.linear_acceleration.z + 0.30102              

        # attitude 
        
        # integrating angular velocity values
        self.roll = self.roll + self.wx * dt
        self.pitch = self.pitch + self.wy * dt
        self.yaw = self.yaw + self.wz * dt
        
        # passing values into a complementary filter 
        self.roll, self.pitch = self.compf1_att.update(
                              self.ax, self.ay, self.az, 
                              self.wx, self.wy, dt)

        roll = np.array([
                    [1, 0, 0],
                    [0, math.cos(self.roll), -math.sin(self.roll)],
                    [0, math.sin(self.roll), math.cos(self.roll)]
                    ])  
        pitch = np.array([
                    [math.cos(self.pitch), 0, math.sin(self.pitch)],
                    [0, 1, 0],
                    [-math.sin(self.pitch), 0, math.cos(self.pitch)]
                    ])
        yaw = np.array([
                    [math.cos(self.yaw), -math.sin(self.yaw), 0],
                    [math.sin(self.yaw), math.cos(self.yaw), 0],
                    [0, 0, 1]
                    ])  

        # rotating acceleration into the world frame
        rotation = yaw @ pitch @ roll
        acceleration = np.array([self.ax, self.ay, self.az])
        self.new_accel = rotation @ acceleration

        # integrating velocity but like. as vector components
        self.velocity_vector[0] = [self.velocity_x + acceleration[0], 
                                   self.velocity_y,
                                   self.velocity_z]
            
        # self.__attitude_message.x = at_x
        # self.__attitude_message.y = at_y
        # self.__attitude_message.z = at_z
        
        ########################## LINEAR VELOCITY ##########################

        # integrating acceleration values
        self.velocity_x = self.velocity_x + self.ax * dt
        self.velocity_y = self.velocity_y + self.ay * dt # should technically be equal to zero ???
        self.velocity_z = self.velocity_z + self.az * dt

        # combines velocity into a scalar 
        self.velocity_scalar = math.sqrt(self.velocity_x ** 2 + self.velocity_z ** 2)   

        # kalman filter
        self.__velocity_message.data = self.kf1_velocity.update(self.velocity_scalar)

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
    
        # complementary filter
        _, _, _, final_theta = self.compf2_theta.update( 
                                            0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0,
                                            new_my, new_mx, dt)

        # use lidar to fix position ????????????????

        # kalman filters
        self.__pose_est_message.x = self.kf2_posx.update(self.position_x)
        self.__pose_est_message.y = self.kf3_posz.update(self.position_z)
        self.__pose_est_message.theta = final_theta

        ########################### PUBLISHING VALUES ###############################
        
        # linear velocity, attitude, 2d pose
        self.__velocity_pub.publish(self.__velocity_message)
        self.__attitude_pub.publish(self.__attitude_message)
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