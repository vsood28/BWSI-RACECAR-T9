class SLAM:
    def __init__(self, ekf, occupancy_grid):
        self.ekf = ekf
        self.occupancy_grid = occupancy_grid

    def get_state_estimate(self):
        return self.ekf.state_estimate
    
    def get_map(self):
        return self.occupancy_grid.grid
    
    def get_map_scale(self):
        return self.occupancy_grid.resolution