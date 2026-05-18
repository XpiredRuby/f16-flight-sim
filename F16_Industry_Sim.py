import numpy as np
import matplotlib.pyplot as plt

from f16sim.config.parameters import industry_config
from f16sim.controls.autopilot import industry_autopilot
from f16sim.sim.runners import run_industry_simulation
from f16sim.plotting.plots import simple_plot

cfg = industry_config()
solution = run_industry_simulation()

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
alpha = np.degrees(np.arctan2(w, u))

delta_e_hist = []
delta_a_hist = []
delta_r_hist = []
for i in range(len(t)):
    de, da, dr = industry_autopilot(np.radians(phi[i]), np.radians(p[i]), np.radians(r[i]), np.radians(theta[i]), np.radians(q[i]), altitude[i], cfg)
    delta_e_hist.append(np.degrees(de)); delta_a_hist.append(np.degrees(da)); delta_r_hist.append(np.degrees(dr))

simple_plot(t, altitude, "Time (s)", "Altitude (m)", "Industry F-16 Altitude Response")
simple_plot(t, alpha, "Time (s)", "Angle of Attack (deg)", "Industry F-16 AoA Response")

plt.figure(figsize=(10, 5)); plt.plot(t, theta, label="Pitch"); plt.plot(t, delta_e_hist, label="Elevator"); plt.xlabel("Time (s)"); plt.ylabel("Degrees"); plt.title("Pitch/Elevator"); plt.legend(); plt.grid(True); plt.tight_layout()
plt.figure(figsize=(10, 5)); plt.plot(t, phi, label="Roll"); plt.plot(t, delta_a_hist, label="Aileron"); plt.xlabel("Time (s)"); plt.ylabel("Degrees"); plt.title("Roll/Aileron"); plt.legend(); plt.grid(True); plt.tight_layout()
plt.figure(figsize=(10, 5)); plt.plot(t, r, label="Yaw Rate"); plt.plot(t, delta_r_hist, label="Rudder"); plt.xlabel("Time (s)"); plt.ylabel("Degrees"); plt.title("Yaw/Rudder"); plt.legend(); plt.grid(True); plt.tight_layout()

plt.show()
