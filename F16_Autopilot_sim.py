# ============================================================
# File: f16_autopilot_sim.py
# ============================================================
#
# Industry-Inspired F-16 Autopilot Simulation
#
# Features:
# - 6DOF-inspired longitudinal dynamics
# - altitude-hold autopilot
# - nested pitch-control loop
# - elevator command generation
# - control saturation
#
# ============================================================

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# ============================================================
# Constants
# ============================================================

g = 9.81

# ============================================================
# Aircraft Parameters
# ============================================================

mass = 12000.0
Iy = 75673.0

S = 27.87
cbar = 3.45

rho = 0.652

# ============================================================
# Aerodynamic Parameters
# ============================================================

CL0 = 0.2
CL_alpha = 5.5
CL_q = 7.5
CL_de = 0.35

CD0 = 0.02
k = 0.07

Cm0 = 0.05
Cm_alpha = -1.8
Cm_q = -30.5
Cm_de = -1.1

# ============================================================
# Autopilot Gains
# ============================================================

Kp_altitude = 0.002
Kp_pitch = 2.5
Kd_pitch = 1.2

# ============================================================
# Trim / Flight Condition
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
# Aerodynamic Model
# ============================================================

def aero_model(u, w, q, delta_e):

    V = np.sqrt(u**2 + w**2)

    alpha = np.arctan2(w, u)

    qbar = 0.5 * rho * V**2

    CL = (
        CL0
        + CL_alpha * alpha
        + CL_q * (q * cbar / (2 * V))
        + CL_de * delta_e
    )

    CD = CD0 + k * CL**2

    Cm = (
        Cm0
        + Cm_alpha * alpha
        + Cm_q * (q * cbar / (2 * V))
        + Cm_de * delta_e
    )

    Lift = qbar * S * CL
    Drag = qbar * S * CD
    Moment = qbar * S * cbar * Cm

    return Lift, Drag, Moment, alpha

# ============================================================
# Autopilot Logic
# ============================================================

def autopilot(theta, q, altitude):

    altitude_error = target_altitude - altitude

    theta_command = Kp_altitude * altitude_error

    delta_e = (
        Kp_pitch * (theta_command - theta)
        - Kd_pitch * q
    )

    # Elevator saturation
    delta_e = np.clip(
        delta_e,
        np.radians(-25),
        np.radians(25)
    )

    return delta_e

# ============================================================
# Equations of Motion
# ============================================================

def equations(t, state):

    u, w, q, theta, z = state

    altitude = -z

    # Autopilot-generated elevator
    delta_e = autopilot(theta, q, altitude)

    Lift, Drag, My, alpha = aero_model(
        u,
        w,
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

# ============================================================
# Simulation
# ============================================================

t_span = (0, 60)

t_eval = np.linspace(0, 60, 4000)

solution = solve_ivp(
    equations,
    t_span,
    state0,
    t_eval=t_eval,
    rtol=1e-8,
    atol=1e-8
)

# ============================================================
# Extract Results
# ============================================================

t = solution.t

u = solution.y[0]
w = solution.y[1]

q = np.degrees(solution.y[2])

theta = np.degrees(solution.y[3])

altitude = -solution.y[4]

alpha = np.degrees(np.arctan2(w, u))

# ============================================================
# Recompute Elevator History
# ============================================================

delta_e_history = []

for i in range(len(t)):

    theta_rad = np.radians(theta[i])
    q_rad = np.radians(q[i])

    de = autopilot(
        theta_rad,
        q_rad,
        altitude[i]
    )

    delta_e_history.append(np.degrees(de))

# ============================================================
# Plot Altitude Response
# ============================================================

plt.figure(figsize=(10, 5))

plt.plot(t, altitude, label="Altitude")

plt.axhline(
    target_altitude,
    linestyle="--",
    label="Target"
)

plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.title("Altitude-Hold Autopilot Response")

plt.legend()
plt.grid(True)
plt.tight_layout()

# ============================================================
# Plot Pitch Response
# ============================================================

plt.figure(figsize=(10, 5))

plt.plot(t, theta, label="Pitch Angle")
plt.plot(t, delta_e_history, label="Elevator")

plt.xlabel("Time (s)")
plt.ylabel("Degrees")
plt.title("Pitch & Elevator Response")

plt.legend()
plt.grid(True)
plt.tight_layout()

# ============================================================
# Plot Angle of Attack
# ============================================================

plt.figure(figsize=(10, 5))

plt.plot(t, alpha)

plt.xlabel("Time (s)")
plt.ylabel("Angle of Attack (deg)")
plt.title("Angle of Attack Response")

plt.grid(True)
plt.tight_layout()

plt.show()

# ============================================================
# End of File
# ============================================================
