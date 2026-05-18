from f16sim.config.parameters import sixdof_config, autopilot_config, industry_config
from f16sim.dynamics.models import run_6dof, run_autopilot, run_industry


def run_6dof_simulation():
    return run_6dof(sixdof_config())


def run_autopilot_simulation():
    return run_autopilot(autopilot_config())


def run_industry_simulation():
    return run_industry(industry_config())
