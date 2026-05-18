import numpy as np


def sixdof_config():
    return {
        "g": 9.81,
        "mass": 12000.0,
        "Ix": 12875.0,
        "Iy": 75673.0,
        "Iz": 85552.0,
        "S": 27.87,
        "cbar": 3.45,
        "b": 9.45,
        "rho": 0.652,
        "CL0": 0.2,
        "CL_alpha": 5.5,
        "CL_q": 7.5,
        "CL_de": 0.35,
        "CD0": 0.02,
        "k": 0.07,
        "Cm0": 0.05,
        "Cm_alpha": -1.8,
        "Cm_q": -30.5,
        "Cm_de": -1.1,
        "delta_e": np.radians(-2.0),
        "thrust": 35000.0,
        "state0": [183.0, 0.0, 8.5, 0.0, 0.0, 0.0, 0.0, np.radians(2.5), 0.0, 0.0, 0.0, -6096.0],
        "t_span": (0, 30),
        "t_eval": np.linspace(0, 30, 2000),
    }


def autopilot_config():
    return {
        "g": 9.81,
        "mass": 12000.0,
        "Iy": 75673.0,
        "S": 27.87,
        "cbar": 3.45,
        "rho": 0.652,
        "CL0": 0.2,
        "CL_alpha": 5.5,
        "CL_q": 7.5,
        "CL_de": 0.35,
        "CD0": 0.02,
        "k": 0.07,
        "Cm0": 0.05,
        "Cm_alpha": -1.8,
        "Cm_q": -30.5,
        "Cm_de": -1.1,
        "Kp_altitude": 0.002,
        "Kp_pitch": 2.5,
        "Kd_pitch": 1.2,
        "thrust": 35000.0,
        "target_altitude": 6500.0,
        "state0": [183.0, 8.0, 0.0, np.radians(2.5), -6000.0],
        "t_span": (0, 60),
        "t_eval": np.linspace(0, 60, 4000),
    }

def industry_config():
    return {
        "g": 9.81,
        "mass": 12000.0,
        "Ix": 12875.0,
        "Iy": 75673.0,
        "Iz": 85552.0,
        "S": 27.87,
        "b": 9.45,
        "cbar": 3.45,
        "rho": 0.652,
        "CL0": 0.2,
        "CL_alpha": 5.5,
        "CL_q": 7.5,
        "CL_de": 0.35,
        "CD0": 0.02,
        "k": 0.07,
        "Cm0": 0.05,
        "Cm_alpha": -1.8,
        "Cm_q": -30.5,
        "Cm_de": -1.1,
        "CY_beta": -0.98,
        "Cl_beta": -0.12,
        "Cl_p": -0.5,
        "Cl_da": 0.08,
        "Cn_beta": 0.25,
        "Cn_r": -0.35,
        "Cn_dr": -0.10,
        "Kp_altitude": 0.002,
        "Kp_pitch": 2.5,
        "Kd_pitch": 1.2,
        "Kp_roll": -2.0,
        "Kd_roll": -0.8,
        "K_yaw": -1.5,
        "thrust": 35000.0,
        "target_altitude": 6500.0,
        "state0": [183.0, 0.0, 8.0, 0.0, 0.0, 0.0, np.radians(5.0), np.radians(2.5), 0.0, -6000.0],
        "t_span": (0, 80),
        "t_eval": np.linspace(0, 80, 5000),
    }


def monte_carlo_config():
    c = autopilot_config()
    c.update({
        "mass_nominal": 12000.0,
        "Iy_nominal": 75673.0,
        "num_runs": 50,
        "simulation_time": 60,
        "seed": 42,
    })
    return c
