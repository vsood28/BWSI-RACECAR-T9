    ### Imports ###
import rclpy 
from rclpy.node import Node  
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Vector3
from ackermann_msgs.msg import AckermannDriveStamped

### Classes ###

class WallFollower(Node):
    def __init__(self):
        # self.node = ros2.create_node("wallf_pub")
        # in case ^ doesn't work:
        super().__init__('wall_follower_node')

        self.create_subscription(Vector3, '/attitude', self.attitude_callback, 10)
        self.create_subscription(Vector3, '/velocity', self.velocity_callback, 10)
        self.create_subscription(Pose, '/pose_estimate', self.pose_callback, 10)

        # self.cur_attitude = None
        # self.cur_velocity = None
        # self.cur_pose = None

    def attitude_callback(self, data):
        pass
    
    def velocity_callback(self, data):
        pass

    def pose_callback(self, data):
        pass

def __update(self):
    self.__publisher.publish(self.__message) # every tick, we want to publish what ?

def main():
    rclpy.init(args=None)
    node = IMU()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()