import numpy as np
from scipy.integrate import solve_ivp

from f16sim.config.parameters import monte_carlo_config
from f16sim.controls.autopilot import longitudinal_autopilot


def run_monte_carlo():
    cfg = monte_carlo_config()
    np.random.seed(cfg["seed"])
    all_altitudes = []
    for _ in range(cfg["num_runs"]):
        mass = cfg["mass_nominal"] * (1 + np.random.normal(0, 0.03))
        Iy = cfg["Iy_nominal"] * (1 + np.random.normal(0, 0.03))
        CL_alpha = cfg["CL_alpha"] * (1 + np.random.normal(0, 0.05))
        Cm_alpha = cfg["Cm_alpha"] * (1 + np.random.normal(0, 0.05))
        CD0 = cfg["CD0"] * (1 + np.random.normal(0, 0.08))
        gust_frequency = np.random.uniform(0.1, 0.4)
        gust_amplitude = np.random.uniform(1.0, 4.0)

        def gust(t): return gust_amplitude * np.sin(gust_frequency * t)
        def aero(u, w, q, de):
            V = np.sqrt(u**2 + w**2); alpha = np.arctan2(w, u); qbar = 0.5 * cfg["rho"] * V**2
            CL = cfg["CL0"] + CL_alpha * alpha + cfg["CL_q"] * (q * cfg["cbar"] / (2 * V)) + cfg["CL_de"] * de
            CD = CD0 + cfg["k"] * CL**2
            Cm = cfg["Cm0"] + Cm_alpha * alpha + cfg["Cm_q"] * (q * cfg["cbar"] / (2 * V)) + cfg["Cm_de"] * de
            return qbar * cfg["S"] * CL, qbar * cfg["S"] * CD, qbar * cfg["S"] * cfg["cbar"] * Cm, alpha
        def eq(t, s):
            u, w, q, theta, z = s
            de = longitudinal_autopilot(theta, q, -z, cfg)
            Lift, Drag, My, alpha = aero(u, w + gust(t), q, de)
            X = cfg["thrust"] - Drag * np.cos(alpha) + Lift * np.sin(alpha)
            Z = -Lift * np.cos(alpha) - Drag * np.sin(alpha)
            return [X / mass - cfg["g"] * np.sin(theta) - q * w, Z / mass + cfg["g"] * np.cos(theta) - q * u, My / Iy, q, -u * np.sin(theta) + w * np.cos(theta)]

        t_eval = np.linspace(0, cfg["simulation_time"], 3000)
        sol = solve_ivp(eq, (0, cfg["simulation_time"]), cfg["state0"], t_eval=t_eval, rtol=1e-8, atol=1e-8)
        all_altitudes.append(-sol.y[4])

    all_altitudes = np.array(all_altitudes)
    return t_eval, all_altitudes, np.mean(all_altitudes, axis=0), np.percentile(all_altitudes, 5, axis=0), np.percentile(all_altitudes, 95, axis=0), cfg
