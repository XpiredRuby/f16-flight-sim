# ============================================================
# File: run_monte_carlo.py
# ============================================================
#
# Monte Carlo Robustness Analysis
#
# Features:
# - uncertainty propagation
# - aerodynamic perturbations
# - mass perturbations
# - gust disturbances
# - percentile envelopes
# - robustness visualization
#
# ============================================================

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# ============================================================
# Random Seed
# ============================================================

np.random.seed(42)

# ============================================================
# Constants
# ============================================================

g = 9.81

# ============================================================
# Nominal Aircraft Parameters
# ============================================================

mass_nominal = 12000.0

Iy_nominal = 75673.0

S = 27.87
cbar = 3.45

rho = 0.652

# ============================================================
# Nominal Aero Parameters
# ============================================================

CL0_nominal = 0.2
CL_alpha_nominal = 5.5
CL_q_nominal = 7.5
CL_de_nominal = 0.35

CD0_nominal = 0.02
k_nominal = 0.07

Cm0_nominal = 0.05
Cm_alpha_nominal = -1.8
Cm_q_nominal = -30.5
Cm_de_nominal = -1.1

# ============================================================
# Autopilot Gains
# ============================================================

Kp_altitude = 0.002

Kp_pitch = 2.5
Kd_pitch = 1.2

# ============================================================
# Flight Condition
# ============================================================

thrust = 35000.0

target_altitude = 6500.0

# ============================================================
# Initial Conditions
# ============================================================

u0 = 183.0
w0 = 8.0

q0 = 0.0

theta0 = np.radians(2.5)

z0 = -6000.0

state0 = [
    u0,
    w0,
    q0,
    theta0,
    z0
]

# ============================================================
# Monte Carlo Settings
# ============================================================

num_runs = 50

simulation_time = 60

# ============================================================
# Storage
# ============================================================

all_altitudes = []

# ============================================================
# Main Monte Carlo Loop
# ============================================================

for run in range(num_runs):

    # ========================================================
    # Random Perturbations
    # ========================================================

    mass = mass_nominal * (
        1 + np.random.normal(0, 0.03)
    )

    Iy = Iy_nominal * (
        1 + np.random.normal(0, 0.03)
    )

    CL_alpha = CL_alpha_nominal * (
        1 + np.random.normal(0, 0.05)
    )

    Cm_alpha = Cm_alpha_nominal * (
        1 + np.random.normal(0, 0.05)
    )

    CD0 = CD0_nominal * (
        1 + np.random.normal(0, 0.08)
    )

    # ========================================================
    # Gust Model
    # ========================================================

    gust_frequency = np.random.uniform(
        0.1,
        0.4
    )

    gust_amplitude = np.random.uniform(
        1.0,
        4.0
    )

    def gust(t):

        return (
            gust_amplitude
            * np.sin(gust_frequency * t)
        )

    # ========================================================
    # Aerodynamics
    # ========================================================

    def aero_model(
        u,
        w,
        q,
        delta_e
    ):

        V = np.sqrt(u**2 + w**2)

        alpha = np.arctan2(w, u)

        qbar = 0.5 * rho * V**2

        CL = (
            CL0_nominal
            + CL_alpha * alpha
            + CL_q_nominal
            * (q * cbar / (2 * V))
            + CL_de_nominal * delta_e
        )

        CD = CD0 + k_nominal * CL**2

        Cm = (
            Cm0_nominal
            + Cm_alpha * alpha
            + Cm_q_nominal
            * (q * cbar / (2 * V))
            + Cm_de_nominal * delta_e
        )

        Lift = qbar * S * CL

        Drag = qbar * S * CD

        Moment = qbar * S * cbar * Cm

        return (
            Lift,
            Drag,
            Moment,
            alpha
        )

    # ========================================================
    # Autopilot
    # ========================================================

    def autopilot(
        theta,
        q,
        altitude
    ):

        altitude_error = (
            target_altitude
            - altitude
        )

        theta_command = (
            Kp_altitude
            * altitude_error
        )

        delta_e = (
            Kp_pitch
            * (theta_command - theta)
            - Kd_pitch * q
        )

        delta_e = np.clip(
            delta_e,
            np.radians(-25),
            np.radians(25)
        )

        return delta_e

    # ========================================================
    # Equations of Motion
    # ========================================================

    def equations(t, state):

        u, w, q, theta, z = state

        altitude = -z

        delta_e = autopilot(
            theta,
            q,
            altitude
        )

        w_total = w + gust(t)

        (
            Lift,
            Drag,
            My,
            alpha
        ) = aero_model(
            u,
            w_total,
            q,
            delta_e
        )

        X = (
            thrust
            - Drag * np.cos(alpha)
            + Lift * np.sin(alpha)
        )

        Z = (
            -Lift * np.cos(alpha)
            - Drag * np.sin(alpha)
        )

        udot = (
            X / mass
            - g * np.sin(theta)
            - q * w
        )

        wdot = (
            Z / mass
            + g * np.cos(theta)
            - q * u
        )

        qdot = My / Iy

        thetadot = q

        zdot = (
            -u * np.sin(theta)
            + w * np.cos(theta)
        )

        return [
            udot,
            wdot,
            qdot,
            thetadot,
            zdot
        ]

    # ========================================================
    # Simulate
    # ========================================================

    t_span = (
        0,
        simulation_time
    )

    t_eval = np.linspace(
        0,
        simulation_time,
        3000
    )

    solution = solve_ivp(
        equations,
        t_span,
        state0,
        t_eval=t_eval,
        rtol=1e-8,
        atol=1e-8
    )

    altitude = -solution.y[4]

    all_altitudes.append(altitude)

# ============================================================
# Convert to Array
# ============================================================

all_altitudes = np.array(
    all_altitudes
)

# ============================================================
# Compute Statistics
# ============================================================

mean_altitude = np.mean(
    all_altitudes,
    axis=0
)

lower_bound = np.percentile(
    all_altitudes,
    5,
    axis=0
)

upper_bound = np.percentile(
    all_altitudes,
    95,
    axis=0
)

# ============================================================
# Plot Monte Carlo Envelope
# ============================================================

plt.figure(figsize=(12, 6))

for i in range(num_runs):

    plt.plot(
        t_eval,
        all_altitudes[i],
        alpha=0.15
    )

plt.plot(
    t_eval,
    mean_altitude,
    linewidth=2,
    label="Mean Response"
)

plt.fill_between(
    t_eval,
    lower_bound,
    upper_bound,
    alpha=0.3,
    label="5-95 Percentile"
)

plt.axhline(
    target_altitude,
    linestyle="--",
    label="Target Altitude"
)

plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")

plt.title(
    "Monte Carlo Robustness Analysis"
)

plt.legend()

plt.grid(True)

plt.tight_layout()

plt.show()

# ============================================================
# Print Statistics
# ============================================================

final_mean_error = (
    target_altitude
    - mean_altitude[-1]
)

print("\n================================================")
print("Monte Carlo Analysis Complete")
print("================================================")
print(f"Number of Runs: {num_runs}")
print(
    f"Final Mean Altitude Error: "
    f"{final_mean_error:.2f} m"
)
print("================================================")

# ============================================================
# End of File
# ============================================================
