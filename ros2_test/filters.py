import math
import time

class ComplementaryFilter:
    def __init__(self, init_alpha):
        self.alpha = init_alpha
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.dt = 0.0 # None ?

    def update(self, ax, ay, az, gx, gy, gz): # accel and gyroscope (angular velo) values
        self.dt = time.time() - self.dt

        accel_roll = math.atan2(ay, az) 
        accel_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        
        self.roll = self.alpha * (self.roll + gx * self.dt) + ((1 - self.alpha) * accel_roll)
        self.pitch = self.alpha * (self.pitch + gy * self.dt) + ((1 - self.alpha) * accel_pitch)
        self.yaw = self.alpha * (self.yaw + gz * self.dt) # i think the second term is removed because accelerometer would give 0?
        

class KalmanFilter:
    # i hate this system
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
    # 2. calculate kalmain gain
    # 3. estimate x n+1
    # 4. update est
    def update(self, nvalue):
        # self.covar_est = self.covar_est + self.noise
        self.kg = self.covar_est / (self.covar_est + self.covar_mea)
        self.n = self.n + (self.kg * (nvalue - self.n))
        self.covar_est = (1 - self.kg) * self.covar_est