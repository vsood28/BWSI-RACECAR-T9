import rclpy as ros2
from rclpy.executors import MultiThreadedExecutor
import numpy as np

from se_node import StateEstimationNode
from controller_node import SLAMController

#spin up both se and control block for full system loop

from ekf_models import state_transistion_jacobian, process_noise_jacobian, measurement_jacobian, state_model, measurement_model

# --- goal for the controller, set this to whatever target (x, y) you want ---
GOAL_XY = (2.0, 1.0)  # meters, in map frame


def main(args=None):
    ros2.init(args=args)

    jacobians = {"st": state_transistion_jacobian, "me": measurement_jacobian, "pr": process_noise_jacobian}

    models = {"st": state_model, "me": measurement_model}

    grid_params = {"w": 200, "h": 360, "res": 0.05}  # 10m by 18 m

    grid_odds = {"po": 0.5, "poh": 0.8, "pom": 0.2}

    sys_params = {"wheelbase": 20}

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

    controller_node = SLAMController(
        goal_xy=GOAL_XY,
        lookahead_cells=5,
        pid_gains=(1.0, 0.0, 0.0),
        obstacle_threshold=60,
        control_rate=10.0,
        base_speed=0.5,
        max_steering_angle=0.4,
    )

    executor = MultiThreadedExecutor()
    executor.add_node(se_node)
    executor.add_node(controller_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        se_node.destroy_node()
        controller_node.destroy_node()
        ros2.shutdown()


if __name__ == '__main__':
    main()