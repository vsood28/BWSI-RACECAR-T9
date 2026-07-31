import numpy as np

class SLAM:
    def __init__(self, ekf, occupancy_grid, sys_params):
        self.ekf = ekf
        self.occupancy_grid = occupancy_grid

        self.sys_params = sys_params

    def get_state_estimate(self):
        return self.ekf.state_estimate
    
    def get_map(self):
        return self.occupancy_grid.grid
    
    def get_map_scale(self):
        return self.occupancy_grid.resolution

    def update_occupancy_grid(self, pt_cloud): #
        self.occupancy_grid.update_grid(pt_cloud, self.get_state_estimate())
    
    def estimate_callback(self, control_input, delta_t):
        self.ekf.predict_state(control_input, delta_t, **self.sys_params)

    def measurement_callback(self, measured_position, delta_t):
        self.ekf.update_state(measured_position, delta_t, **self.sys_params)