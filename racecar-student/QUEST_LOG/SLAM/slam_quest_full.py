import rclpy as ros2

from slam_node import StateEstimationNode
from controller_node import ControllerNode

from ekf_models import state_transistion_jacobian, process_noise_jacobian, measurement_jacobian, state_model, measurement_model, control_model

def main(args=None):
    ros2.init(args=args)

    jacobians = {"st":state_transistion_jacobian, "me":measurement_jacobian, "pr":process_noise_jacobian}

    models = {"st":state_model, "me":measurement_model, "ct":control_model}

    se_node = StateEstimationNode()
    c_node = ControllerNode()

    ros2.spin(se_node)
    ros2.spin(c_node)

    se_node.destroy_node()
    c_node.destroy_node()
    ros2.shutdown()


if __name__ == '__main__':
    main()