import numpy as np
import matplotlib.pyplot as plt

from f16sim.sim.runners import run_6dof_simulation
from f16sim.plotting.plots import simple_plot

solution = run_6dof_simulation()

t = solution.t
u = solution.y[0]
w = solution.y[2]
theta = np.degrees(solution.y[7])
altitude = -solution.y[11]
alpha = np.degrees(np.arctan2(w, u))

simple_plot(t, altitude, "Time (s)", "Altitude (m)", "F-16 Altitude Response")
simple_plot(t, theta, "Time (s)", "Pitch Angle (deg)", "F-16 Pitch Response")
simple_plot(t, alpha, "Time (s)", "Angle of Attack (deg)", "F-16 Angle of Attack")
plt.show()
