import matplotlib.pyplot as plt
from f16sim.analysis.monte_carlo import run_monte_carlo
from f16sim.plotting.plots import monte_carlo_plot

t_eval, all_altitudes, mean_altitude, lower_bound, upper_bound, cfg = run_monte_carlo()
monte_carlo_plot(t_eval, all_altitudes, mean_altitude, lower_bound, upper_bound, cfg["target_altitude"])

final_mean_error = cfg["target_altitude"] - mean_altitude[-1]
print("\n================================================")
print("Monte Carlo Analysis Complete")
print("================================================")
print(f"Number of Runs: {cfg['num_runs']}")
print(f"Final Mean Altitude Error: {final_mean_error:.2f} m")
print("================================================")
plt.show()
