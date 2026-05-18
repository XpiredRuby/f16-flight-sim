# F-16 Flight Controls GNC Simulation Framework

## Overview
Industry-inspired F-16 6DOF flight dynamics, autopilot, and Monte Carlo robustness simulation framework designed to emulate real aerospace flight-controls and GNC engineering workflows.

This project focuses on:
- aircraft flight dynamics
- autopilot design
- aerospace controls engineering
- robustness analysis
- uncertainty quantification
- aerospace simulation workflows

---

# Features

## Flight Dynamics
- 6DOF rigid-body aircraft simulation
- body-axis translational dynamics
- rotational dynamics
- Euler attitude propagation
- nonlinear aerodynamic force/moment modeling

## Aerodynamics
- nonlinear lift and drag
- induced drag modeling
- pitching moment dynamics
- elevator effectiveness
- lateral-directional derivatives
- dynamic stability derivatives:
  - Cmq
  - Czq
  - CL_q

## Control System
- altitude-hold autopilot
- nested pitch-control loop
- roll stabilization
- yaw damper
- control saturation logic

## Robustness & Uncertainty
- Monte Carlo simulation campaigns
- gust disturbances
- parameter perturbation
- uncertainty envelope visualization

---

# Architecture

## Main Components

| File | Purpose |
|---|---|
| 6dof_f16_sim.py | Core aircraft dynamics |
| f16_autopilot_sim.py | Autopilot logic |
| f16_industry_sim.py | High-fidelity industry-inspired framework |
| run_monte_carlo.py | Uncertainty analysis |

---

# Simulation Outputs

The framework generates:
- altitude response plots
- pitch/elevator response
- angle-of-attack response
- roll/aileron response
- yaw/rudder response
- Monte Carlo envelopes

---

# References

Primary references include:
- published F-16 trim literature
- aerodynamic stability derivative references
- aerospace flight dynamics textbooks
- publicly available F-16 parameter datasets

---

# Future Work

Planned upgrades:
- MATLAB/Simulink integration
- gain scheduling
- actuator dynamics
- sensor models
- EKF/UKF state estimation
- lookup-table aerodynamics
- LQR/MPC controllers
- SIL/HIL workflows

---

# Disclaimer

This project is an educational and research-oriented aerospace simulation framework inspired by real flight-controls engineering workflows. It is NOT production flight software.
