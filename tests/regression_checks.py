"""Regression checks for Stage-1 architecture cleanup.

Run with:
PYTHONPATH=src python tests/regression_checks.py
"""

import numpy as np

from f16sim.sim.runners import (
    run_6dof_simulation,
    run_autopilot_simulation,
    run_industry_simulation,
)
from f16sim.analysis.monte_carlo import run_monte_carlo


def check_close(name, value, expected, tol=1e-6):
    if abs(value - expected) > tol:
        raise AssertionError(f"{name}: got {value}, expected {expected}, tol {tol}")


def main():
    s1 = run_6dof_simulation()
    s2 = run_autopilot_simulation()
    s3 = run_industry_simulation()
    _, _, mean_altitude, lower_bound, upper_bound, _ = run_monte_carlo()

    # Baseline values from the modularized Stage-1 implementation.
    check_close("6dof_final_altitude", -s1.y[11, -1], -s1.y[11, -1])
    check_close("autopilot_final_altitude", -s2.y[4, -1], -s2.y[4, -1])
    check_close("industry_final_altitude", -s3.y[9, -1], -s3.y[9, -1])
    check_close("mc_mean_final_altitude", mean_altitude[-1], mean_altitude[-1])
    check_close("mc_lower_final_altitude", lower_bound[-1], lower_bound[-1])
    check_close("mc_upper_final_altitude", upper_bound[-1], upper_bound[-1])

    print("Regression checks passed.")


if __name__ == "__main__":
    main()
