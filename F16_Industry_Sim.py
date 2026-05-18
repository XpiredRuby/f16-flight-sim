# ============================================================
# File: f16_industry_sim.py
# ============================================================
#
# High-Fidelity Industry-Inspired F-16 Simulation
#
# Features:
# - nonlinear longitudinal aerodynamics
# - lateral-directional dynamics
# - roll stabilization
# - yaw damper
# - altitude-hold autopilot
# - gust disturbances
# - actuator saturation
# - Monte Carlo-ready architecture
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

Ix = 12875.0
Iy = 75673.0
Iz = 85552.0

S = 27.87
b = 9.45
cbar = 3.45

rho = 0.652

# ============================================================
# Aerodynamic Parameters
# ============================================================

# Longitudinal
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

# Lateral-Directional
CY_beta = -0.98

Cl_beta = -0.12
Cl_p = -0.5
Cl_da = 0.08

Cn_beta = 0.25
Cn_r = -0.35
Cn_dr = -0.10

# ============================================================
# Autopilot Gains
# ============================================================

# Altitude/Pitch
Kp_altitude = 0.002
Kp_pitch = 2.5
Kd_pitch = 1.2

# Roll
Kp_roll = -2.0
Kd_roll = -0.8

# Yaw Damper
K_yaw = -1.5

# ============================================================
# Flight Condition
# ============================================================

thrust = 35000.0

target_altitude = 6500.0

# ============================================================
# Initial Conditions
# ============================================================

u0 = 183.0
v0 = 0.0
w0 = 8.0

p0 = 0.0
q0 = 0.0
r0 = 0.0

phi0 = np.radians(5.0)
theta0 = np.radians(2.5)
psi0 = 0.0

z0 = -6000.0

state0 = [
    u0,
    v0,
    w0,
    p0,
    q0,
    r0,
    phi0,
    theta0,
    psi0,
    z0
]

# ============================================================
# Gust Model
# ============================================================

def vertical_gust(t):

    return 2.0 * np.sin(0.2 * t)

# ============================================================
# Aerodynamics
# ============================================================

def aero_model(
    u,
    v,
    w,
    p,
    q,
    r,
    delta_e,
    delta_a,
    delta_r
):

    V = np.sqrt(u**2 + v**2 + w**2)

    alpha = np.arctan2(w, u)

    beta = np.arcsin(v / V)

    qbar = 0.5 * rho * V**2

    # ========================================================
    # Longitudinal
    # ========================================================

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

    My = qbar * S * cbar * Cm

    # ========================================================
    # Lateral-Directional
    # ========================================================

    CY = CY_beta * beta

    Cl = (
        Cl_beta * beta
        + Cl_p * (p * b / (2 * V))
        + Cl_da * delta_a
    )

    Cn = (
        Cn_beta * beta
        + Cn_r * (r * b / (2 * V))
        + Cn_dr * delta_r
    )

    SideForce = qbar * S * CY

    L = qbar * S * b * Cl
    N = qbar * S * b * Cn

    return (
        Lift,
        Drag,
        SideForce,
        L,
        My,
        N,
        alpha,
        beta
    )

# ============================================================
# Autopilot
# ============================================================

def autopilot(
    phi,
    p,
    r,
    theta,
    q,
    altitude
):

    # ========================================================
    # Altitude Hold
    # ========================================================

    altitude_error = target_altitude - altitude

    theta_command = (
        Kp_altitude * altitude_error
    )

    delta_e = (
        Kp_pitch * (theta_command - theta)
        - Kd_pitch * q
    )

    # ========================================================
    # Roll Stabilization
    # ========================================================

    delta_a = (
        Kp_roll * phi
        + Kd_roll * p
    )

    # ========================================================
    # Yaw Damper
    # ========================================================

    delta_r = K_yaw * r

    # ========================================================
    # Saturation
    # ========================================================

    delta_e = np.clip(
        delta_e,
        np.radians(-25),
        np.radians(25)
    )

    delta_a = np.clip(
        delta_a,
        np.radians(-21),
        np.radians(21)
    )

    delta_r = np.clip(
        delta_r,
        np.radians(-30),
        np.radians(30)
    )

    return (
        delta_e,
        delta_a,
        delta_r
    )

# ============================================================
# Equations of Motion
# ============================================================

def equations(t, state):

    (
        u,
        v,
        w,
        p,
        q,
        r,
        phi,
        theta,
        psi,
        z
    ) = state

    altitude = -z

    # Gust disturbance
    w_gust = vertical_gust(t)

    w_total = w + w_gust

    # Controller
    (
        delta_e,
        delta_a,
        delta_r
    ) = autopilot(
        phi,
        p,
        r,
        theta,
        q,
        altitude
    )

    (
        Lift,
        Drag,
        SideForce,
        L,
        My,
        N,
        alpha,
        beta
    ) = aero_model(
        u,
        v,
        w_total,
        p,
        q,
        r,
        delta_e,
        delta_a,
        delta_r
    )

    # ========================================================
    # Forces
    # ========================================================

    X = (
        thrust
        - Drag * np.cos(alpha)
        + Lift * np.sin(alpha)
    )

    Y = SideForce

    Z = (
        -Lift * np.cos(alpha)
        - Drag * np.sin(alpha)
    )

    # ========================================================
    # Translational Dynamics
    # ========================================================

    udot = (
        X / mass
        - g * np.sin(theta)
        - q * w
        + r * v
    )

    vdot = (
        Y / mass
        + p * w
        - r * u
    )

    wdot = (
        Z / mass
        + g * np.cos(theta)
        - p * v
        + q * u
    )

    # ========================================================
    # Rotational Dynamics
    # ========================================================

    pdot = L / Ix

    qdot = My / Iy

    rdot = N / Iz

    # ========================================================
    # Euler Kinematics
    # ========================================================

    phidot = p

    thetadot = q

    psidot = r

    # ========================================================
    # Altitude Dynamics
    # ========================================================

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
        zdot
    ]

# ============================================================
# Run Simulation
# ============================================================

t_span = (0, 80)

t_eval = np.linspace(
    0,
    80,
    5000
)

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
v = solution.y[1]
w = solution.y[2]

p = np.degrees(solution.y[3])
q = np.degrees(solution.y[4])
r = np.degrees(solution.y[5])

phi = np.degrees(solution.y[6])
theta = np.degrees(solution.y[7])

altitude = -solution.y[9]

alpha = np.degrees(
    np.arctan2(w, u)
)

# ============================================================
# Control Histories
# ============================================================

delta_e_hist = []
delta_a_hist = []
delta_r_hist = []

for i in range(len(t)):

    (
        de,
        da,
        dr
    ) = autopilot(
        np.radians(phi[i]),
        np.radians(p[i]),
        np.radians(r[i]),
        np.radians(theta[i]),
        np.radians(q[i]),
        altitude[i]
    )

    delta_e_hist.append(
        np.degrees(de)
    )

    delta_a_hist.append(
        np.degrees(da)
    )

    delta_r_hist.append(
        np.degrees(dr)
    )

# ============================================================
# Plot Altitude
# ============================================================

plt.figure(figsize=(10, 5))

plt.plot(t, altitude)

plt.axhline(
    target_altitude,
    linestyle="--"
)

plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.title("F-16 Altitude Response")

plt.grid(True)
plt.tight_layout()

# ============================================================
# Plot Pitch/Elevator
# ============================================================

plt.figure(figsize=(10, 5))

plt.plot(
    t,
    theta,
    label="Pitch"
)

plt.plot(
    t,
    delta_e_hist,
    label="Elevator"
)

plt.xlabel("Time (s)")
plt.ylabel("Degrees")
plt.title("Pitch & Elevator")

plt.legend()
plt.grid(True)
plt.tight_layout()

# ============================================================
# Plot Roll/Aileron
# ============================================================

plt.figure(figsize=(10, 5))

plt.plot(
    t,
    phi,
    label="Roll Angle"
)

plt.plot(
    t,
    delta_a_hist,
    label="Aileron"
)

plt.xlabel("Time (s)")
plt.ylabel("Degrees")
plt.title("Roll & Aileron Response")

plt.legend()
plt.grid(True)
plt.tight_layout()

# ============================================================
# Plot Yaw/Rudder
# ============================================================

plt.figure(figsize=(10, 5))

plt.plot(
    t,
    r,
    label="Yaw Rate"
)

plt.plot(
    t,
    delta_r_hist,
    label="Rudder"
)

plt.xlabel("Time (s)")
plt.ylabel("Degrees")
plt.title("Yaw Damper Response")

plt.legend()
plt.grid(True)
plt.tight_layout()

# ============================================================
# Plot Angle of Attack
# ============================================================

plt.figure(figsize=(10, 5))

plt.plot(t, alpha)

plt.xlabel("Time (s)")
plt.ylabel("Alpha (deg)")
plt.title("Angle of Attack")

plt.grid(True)
plt.tight_layout()

plt.show()

# ============================================================
# End of File
# ============================================================
