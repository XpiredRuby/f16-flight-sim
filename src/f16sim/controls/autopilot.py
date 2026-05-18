import numpy as np


def longitudinal_autopilot(theta, q, altitude, cfg):
    altitude_error = cfg["target_altitude"] - altitude
    theta_command = cfg["Kp_altitude"] * altitude_error
    delta_e = cfg["Kp_pitch"] * (theta_command - theta) - cfg["Kd_pitch"] * q
    return np.clip(delta_e, np.radians(-25), np.radians(25))


def industry_autopilot(phi, p, r, theta, q, altitude, cfg):
    altitude_error = cfg["target_altitude"] - altitude
    theta_command = cfg["Kp_altitude"] * altitude_error
    delta_e = cfg["Kp_pitch"] * (theta_command - theta) - cfg["Kd_pitch"] * q
    delta_a = cfg["Kp_roll"] * phi + cfg["Kd_roll"] * p
    delta_r = cfg["K_yaw"] * r
    delta_e = np.clip(delta_e, np.radians(-25), np.radians(25))
    delta_a = np.clip(delta_a, np.radians(-21), np.radians(21))
    delta_r = np.clip(delta_r, np.radians(-30), np.radians(30))
    return delta_e, delta_a, delta_r
