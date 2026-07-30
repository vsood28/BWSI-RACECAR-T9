import rclpy as ros2
import numpy as np

from slam_node import StateEstimationNode

from ekf_models import state_transistion_jacobian, process_noise_jacobian, measurement_jacobian, state_model, measurement_model, control_model

def main(args=None):
    ros2.init(args=args)

    jacobians = {"st":state_transistion_jacobian, "me":measurement_jacobian, "pr":process_noise_jacobian}

    models = {"st":state_model, "me":measurement_model}

    grid_params = {"w": 10, "h": 12, "res": 0.05}

    grid_odds = {"po": 0.5, "poh": 0.8, "pom": 0.2}

    sys_params = {"wheelbase": 4}

    se_node = StateEstimationNode(
        np.array([0.0, 0.0, 0.0]), 
        np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 0.5]
        ]),
        models,
        jacobians,
        grid_params,
        grid_odds,
        sys_params
        )

    ros2.spin(se_node)

    se_node.destroy_node()
    ros2.shutdown()


if __name__ == '__main__':
    main()