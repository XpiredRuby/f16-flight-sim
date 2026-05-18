import numpy as np
import matplotlib.pyplot as plt

from f16sim.config.parameters import autopilot_config
from f16sim.controls.autopilot import longitudinal_autopilot
from f16sim.sim.runners import run_autopilot_simulation
from f16sim.plotting.plots import simple_plot

cfg = autopilot_config()
solution = run_autopilot_simulation()

t = solution.t
u = solution.y[0]
w = solution.y[1]
q = np.degrees(solution.y[2])
theta = np.degrees(solution.y[3])
altitude = -solution.y[4]
alpha = np.degrees(np.arctan2(w, u))

delta_e_history = []
for i in range(len(t)):
    de = longitudinal_autopilot(np.radians(theta[i]), np.radians(q[i]), altitude[i], cfg)
    delta_e_history.append(np.degrees(de))

simple_plot(t, altitude, "Time (s)", "Altitude (m)", "Altitude-Hold Autopilot Response", label="Altitude", target=cfg["target_altitude"])

plt.figure(figsize=(10, 5))
plt.plot(t, theta, label="Pitch Angle")
plt.plot(t, delta_e_history, label="Elevator")
plt.xlabel("Time (s)")
plt.ylabel("Degrees")
plt.title("Pitch & Elevator Response")
plt.legend()
plt.grid(True)
plt.tight_layout()

simple_plot(t, alpha, "Time (s)", "Angle of Attack (deg)", "Angle of Attack Response")
plt.show()
