import rclpy as ros2

class slam(Node):
    def __init__(self, ekf, occupancy_grid, models, jacobians, sys_params):
        self.ekf = ekf
        self.occupancy_grid = occupancy_grid

        s, m = models.values()

        self.state_model = s
        self.measurement_model = m

        s, m, p = jacobians.values()

        self.state_trans_jacobian = s
        self.measurement_jacobian = m
        self.process_noise_jacobian = p

        self.sys_params = sys_params

    def get_state_estimate(self):
        return self.ekf.state_estimate
    
    def get_map(self):
        return self.occupancy_grid.grid
    
    def get_map_scale(self):
        return self.occupancy_grid.resolution
    
    def estimate_callback(self, control_input, delta_t):
        self.ekf.predict_state(control_input, delta_t, **self.sys_params)
        