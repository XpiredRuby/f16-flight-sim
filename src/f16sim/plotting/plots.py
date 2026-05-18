import matplotlib.pyplot as plt


def simple_plot(x, y, xlabel, ylabel, title, label=None, target=None):
    plt.figure(figsize=(10, 5))
    plt.plot(x, y, label=label)
    if target is not None:
        plt.axhline(target, linestyle="--", label="Target")
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    if label is not None or target is not None:
        plt.legend()
    plt.grid(True)
    plt.tight_layout()


def monte_carlo_plot(t_eval, all_altitudes, mean_altitude, lower_bound, upper_bound, target_altitude):
    plt.figure(figsize=(12, 6))
    for i in range(len(all_altitudes)):
        plt.plot(t_eval, all_altitudes[i], alpha=0.15)
    plt.plot(t_eval, mean_altitude, linewidth=2, label="Mean Response")
    plt.fill_between(t_eval, lower_bound, upper_bound, alpha=0.3, label="5-95 Percentile")
    plt.axhline(target_altitude, linestyle="--", label="Target Altitude")
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.title("Monte Carlo Robustness Analysis")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
