import yaml
import math
import numpy as np

#units of this file: x, y in meters, theta in rads, v in m/s, sigma in rads

with open("model_config.yaml", "r") as file:
    config = yaml.safe_load(file)

#all jacobians are specific to this particular racecar/design
def state_transistion_jacobian(state_estimate, control_input, delta_t, **kwargs): #x, y, heading, steering angle | speed, angle_speed | delta_t | wheelbase
    x, y, theta = state_estimate
    v, sigma = control_input
    L = kwargs.get('wheelbase', config['wheelbase'])

    return np.array([ #partial derivaitves of the control model from lecture 3 #how does a small change in current state effect next state
        [1, 0, -v * math.sin(theta) * delta_t],
        [0, 1, v * math.cos(theta) * delta_t],
        [0, 0, 1],
    ])

def measurement_jacobian(state_estimate, delta_t, **kwargs): #kwargs for system parameters, measurement doesnt need control input, since unrelated
    return np.array([ #since measurement = state estimation from lidar icp directly gives pose transform, 1:1 mapping for all x, y, omega, 0 for sigma since steering angle unfindable form icp alone
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1] #how measurement changes when state changes
    ])

def process_noise_jacobian(state_estimate, control_input, delta_t, **kwargs): #x, y, heading, steering angle | speed, angle_speed | delta_t | wheelbase
    x, y, theta = state_estimate #gives effect of process noise. since noise enters through controls exclusively in this model, control jacobian is not needed
    v, sigma = control_input
    L = kwargs.get('wheelbase', config['wheelbase'])

    return np.array([
        [math.cos(theta) * delta_t, 0], #derivative of control model + noise (sub v -> v + v_noise, theta -> theta + theta_noise) w/respect to cotnrol
        [math.sin(theta) * delta_t, 0],
        [math.tan(sigma)/L * delta_t, 0]
    ])

def state_model(state_estimate, control_input, delta_t, **kwargs):
    x, y, theta = state_estimate #gives effect of process noise. since noise enters through controls exclusively in this model, control jacobian is not needed
    v, sigma = control_input
    L = kwargs.get('wheelbase', config['wheelbase']) #0.20

    return np.array([
        x + v * math.sin(theta) * delta_t,
        y + v * math.cos(theta) * delta_t,
        theta + v/L * math.tan(sigma) * delta_t
    ])

def measurement_model(state_estimate, delta_time, **kwargs): #maps state to predicted measurement
    return state_estimate[:3]

def steering_model(control_angle):
    return 0.53 * control_angle #effect of steering mechanism on control

def velocity_model(ang_vel, w_rad):
    return ang_vel * config['wheel_radius'] #0.04 