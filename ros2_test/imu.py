### Imports ###

import rclpy 
from rclpy.node import Node  

from geometry_msgs.msg import Pose2D, Vector3
from sensor_msgs.msg import Imu, MagneticField # does magnetometer work like this ?
from std_msgs.msg import Float32

import filters
import time

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

        # Filters and extra useful variables ?? + usages
        self.kf1_velocity = filters.KalmanFilter() # linear velocity
        self.kf2_posx = filters.KalmanFilter() # position (x)
        self.kf3_posz = filters.KalmanFilter() # position (y)
        
        self.compf1_att = filters.ComplementaryFilter() # attitude
        self.compf2_theta = filters.ComplementaryFilter() # theta

        # do these get initialized here or in callback
        self.dt = 0.0 # is this initialized with 0 because it should be an integer

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

        # create_subscription parameters: 
        # msg_type (Class), topic (str), callback (function), qos_profile (quality: int)
        self.create_subscription(Imu, '/imu/fused', self.imu_fused_callback, 10)
        self.create_subscription(MagneticField, '/mag', self.mag_callback, 10)


    def imu_fused_callback(self, data): # called every time something is published
        self.dt = time.time() - self.dt # dt for all integration
        
        ########################## LINEAR VELOCITY ##########################

        # integrating acceleration values
        self.velocity_x = self.velocity_x + data.linear_acceleration.x * self.dt
        # self.velocity_y = self.velocity_y + data.linear_acceleration.y * self.dt 
        # # y should technically be equal to zero - maybe ignorable?
        self.velocity_z = self.velocity_z + data.linear_acceleration.z * self.dt

        # combines velocity into a scalar 
        self.velocity_scalar = self.velocity_x ** 2 + self.velocity_z ** 2

        # passing velocity into a kalman filter
        self.kf1_velocity.__init__(self, 0, 0) # is self correct here?
        self.__velocity_message.data = self.kf1_velocity.update(self, self.velocity_scalar)

        # publishing final scalar velocity
        self.__velocity_pub.publish(self.__velocity_message)

        ############################## ATTITUDE ###############################

        # integrating angular velocity values
        self.roll = self.roll + data.angular_velocity.x * self.dt
        self.pitch = self.pitch + data.angular_velocity.y * self.dt
        self.yaw = self.yaw + data.angular_velocity.z * self.dt

        # passing values into a complementary filter 
        # alpha value: trust to put into gyroscope
        self.compf1_att.__init__(self, 0.95, self.roll, self.pitch, self.yaw)
        at_x, at_y, at_z, _ = self.compf1_att.update(self, 
                           data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z, 
                           data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z,
                           None, None)
    
        # taking the values out of complementary filter and assigning them to the message data
        self.__attitude_message.x = at_x
        self.__attitude_message.y = at_y
        self.__attitude_message.z = at_z

        # publishing final attitude components
        self.__attitude_pub.publish(self.__attitude_message)

        ############################### POSE ####################################
        # x,y = x,z because y has the gravity acceleration in it for some reason
        
        # integrating linear velocity values
        self.position_x = self.position_x + self.velocity_x * self.dt
        self.position_z = self.position_z + self.velocity_z * self.dt

        # passing x and z into kalman filters
        self.kf2_posx.__init__(self, 0, 0)
        self.kf3_posz.__init__(self, 0, 0)

        # passing theta into complementary filter
        # alpha value: trust to put into gyroscope
        self.compf2_theta.__init__(self, 0.95, None, None, None)
        _, _, _, final_theta = self.compf1_att.update(self, 
                                            None, None, None,
                                            None, None, None,
                                            self.mz, self.mx)

        # assigning various values to message data
        self.__pose_est_message.data.x = self.kf2_velocity.update(self, self.position_x)
        self.__pose_est_message.data.y = self.kf3_velocity.update(self, self.position_z)
        self.__pose_est_message.data.theta = final_theta

        # publishing final 2d pose values
        self.__pose_est_pub.publish(self.__pose_est_message)


    def mag_callback(self, data):
        self.mx = data.magnetic_field.x 
        # self.my = data.magnetic_field.y 
        self.mz = data.magnetic_field.z 


def main():
    rclpy.init(args=None)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()