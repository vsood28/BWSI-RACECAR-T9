import yaml
import math
import numpy as np

with open("model_config.yaml", "r") as file:
    config = yaml.safe_load(file)

#all jacobians are specific to this particular racecar/design
def state_transistion_jacobian(state_estimate, control_input, delta_t, **kwargs): #x, y, heading, steering angle | speed, angle_speed | delta_t | wheelbase
    x, y, theta, sigma = state_estimate
    v, omega = control_input
    L = kwargs.get('wheelbase', config['wheelbase'])

    return np.array([ #partial derivaitves of the control model from lecture 3
        [1, 0, -v * math.sin(theta) * delta_t, 0],
        [0, 1, v * math.cos(theta) * delta_t, 0],
        [0, 0, 1, v/L * (1/math.cos(sigma)**2) * delta_t],
        [0, 0, 0, 1]
    ])

def measurement_jacobian(state_estimate, delta_t, **kwargs): #kwargs for system parameters, measurement doesnt need control input, since unrelated
    return np.array([ #since measurement = state estimation from lidar icp directly gives pose transform, 1:1 mapping for all x, y, omega, 0 for sigma since steering angle unfindable form icp alone
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0] #how measurement changes when state changes
    ])

def process_noise_jacobian(state_estimate, control_input, delta_t, **kwargs): #x, y, heading, steering angle | speed, angle_speed | delta_t | wheelbase
    x, y, theta, sigma = state_estimate #gives effect of process noise. since noise enters through controls exclusively in this model, control jacobian is not needed
    L = kwargs.get('wheelbase', config['wheelbase'])

    return np.array([
        [math.cos(theta) * delta_t, 0], #derivative of control model + noise (sub v -> v + v_noise, theta -> theta + theta_noise)
        [math.sin(theta) * delta_t, 0],
        [math.tan(sigma)/L * delta_t, 0],
        [0, delta_t]
    ])

def state_model(state_estimate, control_input, delta_t, **kwargs):
    x, y, theta, sigma = state_estimate #gives effect of process noise. since noise enters through controls exclusively in this model, control jacobian is not needed
    v, omega = control_input
    L = kwargs.get('wheelbase', config['wheelbase'])

    return np.array([
        x + v * math.sin(theta) * delta_t,
        y + v * math.cos(theta) * delta_t,
        theta + v/L * math.tan(sigma) * delta_t,
        sigma + omega * delta_t
    ])

def measurement_model(state_estimate, delta_time, **kwargs): #maps state to predicted measurement
    return state_estimate[:3]

def control_model(raw_controls): #return control output, convert (v, theta) -> (v, omega), sysid 
    omega = raw_controls[1]
    return (raw_controls[0], omega)
