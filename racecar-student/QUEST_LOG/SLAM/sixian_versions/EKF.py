import numpy as np
#unitless since this is generic
class ExtendedKalmanFilter:
    def __init__(self,
                 initial_state_estimate,
                 initial_state_covariance,
                 process_noise_covariance,
                 measurement_noise_covariance,
                 state_transition_jacobian,
                 measurement_jacobian,
                 process_noise_jacobian,
                 state_model,
                 measurement_model): #last 5 are funcs

        self.state_estimate = initial_state_estimate.copy()
        self.state_estimation_covariance = initial_state_covariance.copy()
        self.process_noise_covariance = process_noise_covariance
        self.measurement_noise_covariance = measurement_noise_covariance

        self.state_transition_jacobian = state_transition_jacobian
        self.measurement_jacobian = measurement_jacobian
        self.process_noise_jacobian = process_noise_jacobian

        self.state_model = state_model
        self.measurement_model = measurement_model

        self.kalman_gain = None

    def predict_state(self, control_input, delta_t, **kwargs): #encoder update
        self.state_estimate = self.state_model(self.state_estimate, control_input, delta_t, **kwargs) #update predicted state

        F = self.state_transition_jacobian(self.state_estimate, control_input, delta_t, **kwargs)
        P = self.state_estimation_covariance
        Q = self.process_noise_covariance
        G = self.process_noise_jacobian(self.state_estimate, control_input, delta_t, **kwargs)

        self.state_estimation_covariance = F @ P @ F.T + G @ Q @ G.T
        
    def update_state(self, true_measurement, delta_t, **kwargs): #measurement step, compares predicted measurement based on state to true measurement, finds difference, applies correction
        predicted_measurement = self.measurement_model(self.state_estimate, delta_t, **kwargs)
        innovation = true_measurement - predicted_measurement #difference

        H = self.measurement_jacobian(self.state_estimate, delta_t, **kwargs) #get jacobian
        P = self.state_estimation_covariance
        R = self.measurement_noise_covariance
        I = np.identity(len(self.state_estimate))

        S = H @ P @ H.T + R #innovation covariance: basically, hpht is uncertainty of state translated into uncertainty of measurement, R, is uncertainty due to sensor noise, sum is "measurement error" (not exactly, close enough)
        self.kalman_gain = np.linalg.solve(S.T, H @ P).T # pht translates from "how wrong is my measurement" (innovation, or S) to how states are affeced by how wrong the measruement is
        # equiv to P @ H.T @ np.linalg.inv(S) but faster or smtg idk
        self.state_estimate += self.kalman_gain @ innovation

        L = (I - self.kalman_gain @ H)

        self.state_estimation_covariance = L @ self.state_estimation_covariance @ L.T + self.kalman_gain @ R @ self.kalman_gain.T #joseph form