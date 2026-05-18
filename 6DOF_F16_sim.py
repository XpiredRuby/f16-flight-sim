# ============================================================
# File: 6dof_f16_sim.py
# ============================================================
#
# Industry-Inspired F-16 6DOF Flight Dynamics Simulation
#
# This script implements:
# - 6DOF rigid-body equations of motion
# - nonlinear longitudinal aerodynamics
# - induced drag
# - pitching moment dynamics
# - simplified F-16 geometry/inertia
# - trim-like cruise condition
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
# F-16 Approximate Parameters
# ============================================================

mass = 12000.0               # kg
Ix = 12875.0                 # kg*m^2
Iy = 75673.0                 # kg*m^2
Iz = 85552.0                 # kg*m^2

S = 27.87                    # wing area (m^2)
cbar = 3.45                  # mean aerodynamic chord (m)
b = 9.45                     # wingspan (m)

rho = 0.652                  # kg/m^3 (approx @ 20k ft)

# ============================================================
# Aerodynamic Coefficients
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
# Control Inputs
# ============================================================

delta_e = np.radians(-2.0)      # elevator
thrust = 35000.0                # N

# ============================================================
# Initial Conditions
# ============================================================

u0 = 183.0
v0 = 0.0
w0 = 8.5

p0 = 0.0
q0 = 0.0
r0 = 0.0

phi0 = 0.0
theta0 = np.radians(2.5)
psi0 = 0.0

x0 = 0.0
y0 = 0.0
z0 = -6096.0

state0 = [
    u0, v0, w0,
    p0, q0, r0,
    phi0, theta0, psi0,
    x0, y0, z0
]

# ============================================================
# Aerodynamics
# ============================================================

def aero_model(u, w, q):

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
    PitchMoment = qbar * S * cbar * Cm

    return Lift, Drag, PitchMoment, alpha

# ============================================================
# Equations of Motion
# ============================================================

def equations(t, state):

    u, v, w, p, q, r, phi, theta, psi, x, y, z = state

    V = np.sqrt(u**2 + v**2 + w**2)

    Lift, Drag, My, alpha = aero_model(u, w, q)

    # Body-axis forces
    X = thrust - Drag * np.cos(alpha) + Lift * np.sin(alpha)
    Z = -Lift * np.cos(alpha) - Drag * np.sin(alpha)

    # Translational dynamics
    udot = X / mass - g * np.sin(theta) - q * w
    vdot = 0.0
    wdot = Z / mass + g * np.cos(theta) - q * u

    # Rotational dynamics
    pdot = 0.0
    qdot = My / Iy
    rdot = 0.0

    # Euler angle rates
    phidot = p
    thetadot = q
    psidot = r

    # Position kinematics
    xdot = (
        u * np.cos(theta) * np.cos(psi)
        + w * np.sin(theta)
    )

    ydot = 0.0

    zdot = (
        -u * np.sin(theta)
        + w * np.cos(theta)
    )

    return [
        udot,
        vdot,
        wdot,
        pdot,
        qdot,
        rdot,
        phidot,
        thetadot,
        psidot,
        xdot,
        ydot,
        zdot
    ]

# ============================================================
# Simulation
# ============================================================

t_span = (0, 30)

t_eval = np.linspace(0, 30, 2000)

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
w = solution.y[2]

theta = np.degrees(solution.y[7])

altitude = -solution.y[11]

alpha = np.degrees(np.arctan2(w, u))

# ============================================================
# Plot Results
# ============================================================

plt.figure(figsize=(10, 5))
plt.plot(t, altitude)
plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.title("F-16 Altitude Response")
plt.grid(True)
plt.tight_layout()

plt.figure(figsize=(10, 5))
plt.plot(t, theta)
plt.xlabel("Time (s)")
plt.ylabel("Pitch Angle (deg)")
plt.title("F-16 Pitch Response")
plt.grid(True)
plt.tight_layout()

plt.figure(figsize=(10, 5))
plt.plot(t, alpha)
plt.xlabel("Time (s)")
plt.ylabel("Angle of Attack (deg)")
plt.title("F-16 Angle of Attack")
plt.grid(True)
plt.tight_layout()

plt.show()

# ============================================================
# End of File
# ============================================================
