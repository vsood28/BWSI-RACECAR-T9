import math
import time

class ComplementaryFilter:
    def __init__(self, init_alpha, init_roll, init_pitch, init_yaw):
        self.alpha = init_alpha
        self.roll = init_roll
        self.pitch = init_pitch
        self.yaw = init_yaw
        self.theta = 0.0
        self.dt = 0.0

    def update(self, ax, ay, az, wx, wy, wz, mz, mx): # accel, gyroscope (angular velocity), and magnetometer values
        self.dt = time.time() - self.dt

        accel_roll = math.atan2(ay, az) 
        accel_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        theta_mag = math.atan2(mz, mx) # axis convention could be off
        
        self.roll = self.alpha * (self.roll + wx * self.dt) + ((1 - self.alpha) * accel_roll)
        self.pitch = self.alpha * (self.pitch + wy * self.dt) + ((1 - self.alpha) * accel_pitch)
        self.yaw = self.alpha * (self.yaw + wz * self.dt) # i think the second term is removed because accelerometer would give 0?
        self.theta = self.alpha * (self.theta + wy * self.dt) + ((1 - self.alpha) * theta_mag)
        
        return self.roll, self.pitch, self.yaw, self.theta

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
    # 2. calculate kalman gain
    # 3. estimate x n+1
    # 4. update est
    def update(self, nvalue):
        # self.covar_est = self.covar_est + self.noise
        self.kg = self.covar_est / (self.covar_est + self.covar_mea)
        self.n = self.n + (self.kg * (nvalue - self.n))
        self.covar_est = (1 - self.kg) * self.covar_est
        return self.n