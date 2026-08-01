# ros implementation of complementary and kalman filters
# kalman is a 1dkf, certain values were removed from complementary update after realizing that i 
# needed to implement ekfs and rotating acceleration into the world frame

import math
import time

class ComplementaryFilter:
    def __init__(self, init_alpha, init_roll, init_pitch):
        self.alpha = init_alpha
        self.roll = init_roll
        self.pitch = init_pitch

    def update(self, ax, ay, az, wx, wy, dt): # accel, gyroscope (angular velocity)
        accel_roll = math.atan2(ay, az)
        accel_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        
        self.roll = self.alpha * (self.roll + wx * dt) + ((1 - self.alpha) * accel_roll)
        self.pitch = self.alpha * (self.pitch + wy * dt) + ((1 - self.alpha) * accel_pitch)
        
        return self.roll, self.pitch

class KalmanFilter:
    # covar_est: covariable EST
    # covar_mea: covariable MEA
    # n: x subscript n, estimate for the robot (in this case, mean)
    # noise: process noise (HOPEFULLY ZERO) !!!
    # kg: kalman gain

    def __init__(self, init_mea, init_state): # init_est ?
        self.covar_est = 1
        self.covar_mea = init_mea
        self.n = init_state
        # self.noise = init_noise
        self.kg = 0.0 # None ?

    # ordering:
    # 1. add process noise 
    # 2. calculate kalman gain
    # 3. estimate x n+1
    # 4. update est
    def update(self, nvalue):
        # self.covar_est = self.covar_est + self.noise
        self.kg = self.covar_est / (self.covar_est + self.covar_mea)
        self.n = self.n + (self.kg * (nvalue - self.n))
        self.covar_est = (1 - self.kg) * self.covar_est
        return self.n