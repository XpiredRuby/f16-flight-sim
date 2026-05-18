# F-16 Flight Simulation Repository Assessment & Incremental Refactor Plan

## Scope and review method

This assessment is based on a full repository inspection of all current source and documentation files. It is intentionally conservative: no aerodynamic derivatives are changed, no dynamics logic is rewritten, and no new physics are claimed.

## Current technical baseline

### What is working well now

1. **Core nonlinear simulation behavior exists and runs end-to-end**
   - 6-DOF-style body-axis state propagation exists in `6DOF_F16_sim.py` and a more feature-rich closed-loop variant exists in `F16_Industry_Sim.py`.
   - Longitudinal/lateral-directional aerodynamic effects are present, including dynamic derivatives in the current simplified model form.

2. **Control architecture concepts are present**
   - Altitude outer loop and pitch inner loop are implemented in both autopilot-oriented scripts.
   - Lateral stabilization (roll + yaw damper) appears in `F16_Industry_Sim.py`.

3. **Uncertainty/robustness workflow exists**
   - Monte Carlo perturbation workflow (`Run_Monte_Carlo.py`) is useful as a seed for future V&V and robustness regression.

4. **Model assumptions are transparent in code**
   - Parameters and coefficients are explicit constants, helping traceability even before modularization.

## Architectural weaknesses and engineering risks

### 1) Monolithic script architecture

- Each simulation mode is a standalone executable script with duplicated physics/control logic.
- No reusable package modules exist for dynamics, aerodynamics, controls, or scenario configuration.
- Result: high change risk and inconsistent behavior as one script evolves faster than others.

### 2) Heavy cross-file duplication

- `aero_model`, basic equations of motion, and altitude/pitch control logic are repeated with slight variations.
- Aircraft constants and aerodynamic derivatives are repeated in multiple files.
- Result: parameter drift risk (same conceptual quantity updated in one file and missed in others).

### 3) Inconsistent state definitions across scripts

- `6DOF_F16_sim.py` uses 12-state form with `x,y,z`, while `F16_Industry_Sim.py` keeps a reduced position subset with only `z`.
- Autopilot and Monte Carlo scripts use a 5-state longitudinal subset.
- This is acceptable for scenario simplification, but not currently formalized (no declared model contracts).

### 4) Numerical robustness concerns (manageable but important)

- Divisions by `V` in aerodynamic dynamic-derivative terms and `beta = arcsin(v/V)` lack low-speed guards.
- Euler-angle kinematics are simplified (`phidot=p`, `thetadot=q`, `psidot=r`) instead of full coupled transformations.
- Current behavior may remain acceptable near trim/cruise, but edge-case stability and physical consistency can degrade.

### 5) Configuration/data handling limitations

- No typed configuration objects or parameter datasets.
- Scenario setup (initial conditions, gains, perturbations) is hard-coded in scripts.
- No explicit provenance tag for parameter sets (e.g., “nominal_public_lit_v1”).

### 6) Verification and regression gaps

- No unit tests or regression baselines.
- No automated checks for trim consistency, bounded states, control saturation rates, or reproducibility.
- Monte Carlo output is visual + print based; not yet machine-checkable in CI.

### 7) Simulation orchestration limitations

- Single-run workflows embedded in script top level.
- No reusable `run_simulation(config)` API returning structured outputs.
- Analysis and plotting tightly coupled to dynamics execution.

## Recommended subsystem boundaries

To preserve behavior while improving maintainability, use the following boundaries:

1. **`dynamics/`**
   - Equations of motion (6-DOF and reduced-order variants).
   - Frame/kinematics utilities.
   - State vector contracts + index definitions.

2. **`aero/`**
   - Aerodynamic coefficient/force/moment calculations.
   - Explicit model variants (longitudinal-only, full lat-dir).
   - Shared numerical protections (`V_min`, safe `beta` clamp).

3. **`controls/`**
   - Altitude/pitch autopilot logic.
   - Roll/yaw stabilization logic.
   - Saturation/command-limiting utilities.

4. **`actuators/` (initial stub in current phase)**
   - Keep passthrough model initially (no dynamics), but define command/deflection interface now.
   - Future insertion point for 1st-order rate/position limits.

5. **`sensors/` (initial stub in current phase)**
   - Start with truth passthrough interfaces.
   - Future insertion point for bias/noise/filtering and estimator coupling.

6. **`sim/` orchestration**
   - Integrator wrapper and scenario runner.
   - Standardized simulation result container.

7. **`analysis/`**
   - Plotting and Monte Carlo statistics/post-processing only.
   - No dynamics equations inside analysis modules.

8. **`config/`**
   - Parameter sets (mass/inertia/aero/gains/trim/ICs).
   - Named scenarios and perturbation settings.

## Professional repository structure (incremental target)

```text
f16-flight-sim/
  pyproject.toml
  src/
    f16sim/
      config/
        aircraft_nominal.py
        control_nominal.py
        scenarios.py
      models/
        state_defs.py
      aero/
        longitudinal.py
        lateral_directional.py
      dynamics/
        sixdof.py
        longitudinal.py
        kinematics.py
      controls/
        autopilot_longitudinal.py
        stabilization_lateral.py
        limits.py
      actuators/
        passthrough.py
      sensors/
        passthrough.py
      sim/
        runner.py
        integrator.py
      analysis/
        plotting.py
        monte_carlo.py
        metrics.py
  scripts/
    run_6dof_demo.py
    run_autopilot_demo.py
    run_industry_demo.py
    run_monte_carlo.py
  tests/
    test_aero_sanity.py
    test_trim_regression.py
    test_autopilot_regression.py
    test_monte_carlo_repro.py
  baselines/
    nominal_autopilot_metrics.json
    mc_seed42_metrics.json
```

## Staged refactor plan (risk-minimized)

### Stage 0 — Baseline freeze (no model changes)

- Capture current behavior metrics from all 4 scripts with fixed seed.
- Store key scalar checks: final altitude error, peak |alpha|, peak |q|, max control usage.
- Purpose: guardrail before any refactor.

**Risk:** low.
**Verification:** compare metrics to baseline after every stage.

### Stage 1 — Extract shared constants/config (no equation changes)

- Move duplicated constants into a single config module.
- Keep script behavior identical by importing same values.

**Risk:** medium (naming/reference mistakes).
**Verification:** bitwise-close trajectory comparisons vs Stage 0.

### Stage 2 — Extract aero + control functions into modules

- Consolidate duplicated `aero_model` and autopilot logic with explicit variants.
- Preserve existing signatures to minimize churn.

**Risk:** medium.
**Verification:** per-script regression checks and saturation consistency checks.

### Stage 3 — Introduce simulation runner API

- Implement `run_simulation(model, config) -> results`.
- Keep plotting separated from numerical integration.

**Risk:** medium.
**Verification:** same time vector length, terminal states, and selected transient metrics.

### Stage 4 — Add numerical safeguards (careful, opt-in)

- Add guarded `V_eff = max(V, V_min)` and clamp for `beta` argument.
- Start behind configuration flag defaulting to current behavior to avoid hidden drift.

**Risk:** medium-to-high (can alter transients).
**Verification:** targeted low-speed stress cases + nominal-case non-regression.

### Stage 5 — Testing/CI foundation

- Add unit tests for coefficients/sign conventions.
- Add regression tests for nominal trajectories and Monte Carlo summary metrics.

**Risk:** low.
**Verification:** CI passing; deterministic outputs with fixed random seeds.

### Stage 6 — Interoperability prep (MATLAB/Simulink)

- Define I/O contracts for states, controls, and parameter blocks.
- Add export/import utilities (JSON/CSV/MAT) for scenario and trace exchange.

**Risk:** low.
**Verification:** round-trip data consistency tests.

## Testing and regression strategy

### Core test layers

1. **Unit tests**
   - Aerodynamic coefficient monotonicity sanity checks near trim.
   - Saturation and sign convention checks for control commands.

2. **Scenario regression tests**
   - Nominal autopilot scenario metrics within tight tolerances.
   - 6-DOF open-loop scenario boundedness checks.

3. **Monte Carlo reproducibility tests**
   - Fixed seed statistical metrics (mean final error, 5/95 envelope at selected times).

4. **Numerical robustness tests**
   - Low-airspeed guard behavior.
   - Integration step-sensitivity smoke tests.

### Recommended acceptance gates

- No unbounded state growth in nominal cases.
- No sign flips in key control channels.
- Final altitude tracking metrics stay within baseline-defined tolerances.
- Monte Carlo summary metrics remain within agreed tolerance bands.

## MATLAB/Simulink interoperability strategy

1. **Freeze canonical signal definitions now**
   - State ordering, units, sign conventions, control channel names.

2. **Implement explicit interface contracts**
   - Python-side dataclasses for state/control/parameters.
   - Deterministic trace export for Simulink comparison.

3. **Use staged co-simulation validation**
   - Step 1: compare open-loop aero/dynamics outputs.
   - Step 2: compare closed-loop autopilot responses.
   - Step 3: compare Monte Carlo aggregate statistics.

4. **Do not migrate all physics at once**
   - Start with longitudinal channel parity first; then lateral-directional.

## Assumptions and simplifications called out explicitly

- Current model appears focused on cruise-like conditions and moderate perturbations.
- Euler kinematic simplifications are retained during initial refactor stages to preserve behavior.
- No claim of certification-grade validation is made.
- No new aerodynamic data or derivatives are introduced in this plan.

## Immediate next engineering actions (recommended)

1. Add a minimal `src/f16sim/config` module and migrate constants only.
2. Add regression metric capture script for current four executables.
3. Introduce first automated test: nominal autopilot final altitude error and control saturation bounds.

These three actions provide the highest maintainability return for the lowest physics/regression risk.
