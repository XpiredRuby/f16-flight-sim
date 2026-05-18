# Repository Technical Assessment & Incremental Refactor Plan

## Scope and approach
This assessment is based on direct inspection of the current Python scripts in the repository. It intentionally avoids changing flight physics, gains, or aerodynamic coefficients at this stage. The goal is to stabilize architecture and verification workflow while preserving current behavior.

## Current architecture assessment

### What is working well
- The codebase already contains the core ingredients for a useful early-stage GNC simulation workflow: nonlinear equations of motion, autopilot logic, and uncertainty analysis. 
- The simulation scripts use strict ODE tolerances (`rtol=1e-8`, `atol=1e-8`), which is a good baseline for repeatable continuous-time integration studies.
- Control saturation is present in autopilot implementations, reducing immediate risk of unbounded control commands.

### Principal architectural weaknesses
1. **Script-centric and monolithic structure**  
   Flight model equations, control laws, plotting, constants, and scenario setup are tightly mixed inside executable scripts rather than separated into reusable modules.

2. **Logic duplication across scripts**  
   Very similar aerodynamic and longitudinal dynamics logic appears in multiple files, increasing drift risk and making consistent updates difficult.

3. **Implicit data contracts**  
   State vectors are positional lists/arrays with no typed schema or named container, which raises maintenance risk when adding states or refactoring model variants.

4. **No explicit simulation configuration object**  
   Parameters, gains, trim conditions, and run options are globals. This reduces traceability and reproducibility when running multiple scenarios.

5. **No automated regression harness**  
   There are no unit tests or baseline trajectory checks to guard against unintended behavior changes.

6. **Coupled compute and visualization**  
   Simulation and plotting are fused, preventing batch/non-plot workflows and hindering CI execution.

### Numerical and modeling risks to track
- **Potential divide-by-zero / conditioning risk** when terms use `1/V` and `V` is computed from states without floor protection in aerodynamic-rate terms.
- **Euler-angle kinematics limitations** are acceptable for current envelope but should be explicitly bounded and tested for larger attitudes.
- **Gust insertion consistency** should be reviewed to ensure disturbance enters translational dynamics consistently with aerodynamic angle calculations.
- **Randomized Monte Carlo parameters** are reproducible due to seeded RNG, but parameter sampling bounds and physical plausibility checks are not yet formalized.

### Verification gaps
- No smoke tests verifying solver success status and finite outputs.
- No deterministic regression metrics (e.g., final altitude error bands, peak alpha bounds, saturation dwell time).
- No separate tests for controller logic (e.g., sign conventions, saturation limits, command monotonicity).

## Recommended subsystem boundaries
Use these boundaries without changing physics content initially:

1. `dynamics/`
   - Equations of motion (longitudinal-only and 6DOF variants)
   - State derivative assembly only

2. `aero/`
   - Aerodynamic coefficient/force/moment calculations
   - Shared helper functions (alpha/beta, dynamic pressure, rate normalization)

3. `controls/`
   - Altitude/pitch loop
   - Roll stabilizer
   - Yaw damper
   - Command limits/saturation utilities

4. `disturbances/`
   - Gust models and disturbance injection interfaces

5. `analysis/`
   - Monte Carlo orchestration
   - Percentile/statistics computation
   - Batch metrics utilities

6. `sim/`
   - Integrator wrappers
   - Scenario runners
   - Time-grid and termination settings

7. `config/`
   - Aircraft parameters
   - Controller gains
   - Scenario/trim settings
   - Monte Carlo uncertainty settings

8. `io_plot/`
   - Plot functions only (no dynamics)
   - Optional report export helpers

## Staged refactor plan (incremental, low-risk)

### Stage 0 — Baseline capture (no behavior change)
- Add a baseline script that runs existing scenarios and records reference metrics:
  - final altitude
  - final altitude error
  - max/95th percentile alpha
  - max |delta_e|, |delta_a|, |delta_r| where relevant
- Store baseline JSON artifacts for future comparisons.

**Risk:** Minimal.  
**Verification:** Compare reruns against saved baseline with tight tolerances.

### Stage 1 — Parameter/config isolation
- Move constants and gains into structured config dataclasses (or plain frozen dictionaries if dataclasses are undesired initially).
- Keep equations and gains numerically identical.

**Risk:** Mis-wiring parameters after extraction.  
**Verification:** Bitwise-close trajectory comparison (or very small numeric tolerance) against Stage 0 baseline.

### Stage 2 — Pure function extraction
- Extract reusable pure functions for:
  - aero calculations
  - autopilot laws
  - EOM RHS functions
- Ensure plotting and top-level execution call these functions.

**Risk:** Hidden dependency on globals.  
**Verification:** Unit tests for each extracted function + trajectory regression.

### Stage 3 — Scenario runners and orchestration
- Add a `run_scenario(config)` API returning structured results.
- Separate plotting from solver execution.

**Risk:** API transition errors in existing scripts.  
**Verification:** Keep compatibility wrapper scripts; compare outputs and key metrics.

### Stage 4 — Test harness introduction
- Add `pytest` tests for:
  - smoke runs
  - autopilot saturation limits
  - aero finite outputs for representative states
  - regression metrics within tolerance bands

**Risk:** None to runtime; only CI/runtime burden.  
**Verification:** Ensure tests pass locally and in CI environment.

### Stage 5 — Monte Carlo modularization
- Convert Monte Carlo into reusable analysis module using shared model APIs.
- Add summary-stat regression checks (mean final error range, percentile monotonicity, non-NaN envelope outputs).

**Risk:** Performance regressions and accidental behavioral drift.  
**Verification:** Seeded-run comparisons with archived statistics.

## Recommended repository structure

```text
f16-flight-sim/
  src/
    f16sim/
      aero/
        longitudinal.py
        lateral_directional.py
      dynamics/
        eom_longitudinal.py
        eom_6dof.py
      controls/
        altitude_pitch.py
        lateral_autopilot.py
        saturation.py
      disturbances/
        gust.py
      sim/
        integrator.py
        scenarios.py
      analysis/
        monte_carlo.py
        metrics.py
      config/
        aircraft.py
        controllers.py
        scenarios.py
        uncertainty.py
      io_plot/
        timeseries.py
        envelopes.py
  scripts/
    run_6dof_demo.py
    run_autopilot_demo.py
    run_industry_demo.py
    run_monte_carlo.py
  tests/
    test_smoke.py
    test_controls.py
    test_aero.py
    test_regression.py
  baselines/
    nominal_autopilot_metrics.json
    industry_metrics.json
    monte_carlo_seed42_metrics.json
  README.md
```

## Regression and testing strategy

### Minimum immediate checks (first pass)
1. **Smoke checks**: each scenario executes, solver succeeds, outputs finite.
2. **Control checks**: commanded surfaces always stay inside saturation limits.
3. **State sanity checks**: no NaN/Inf; altitude/alpha remain bounded for nominal run window.
4. **Determinism checks**: Monte Carlo seed reproducibility for summary metrics.

### Medium-term checks
- Golden-metric regression with tolerances rather than full-trajectory exact matching.
- Separate tests for aerodynamic helper functions over representative state grids.
- Runtime budget checks to keep Monte Carlo manageable in CI.

## MATLAB/Simulink interoperability strategy

1. **Freeze data contracts first**  
   Define explicit state ordering, units, sign conventions, and control interface in one schema document/module.

2. **Keep model kernels pure and side-effect free**  
   Pure RHS/aero/controller functions translate cleanly to MATLAB functions or Simulink MATLAB Function blocks.

3. **Use config files with simple serialization**  
   JSON/YAML parameter bundles can be imported by Python and MATLAB to maintain parameter consistency.

4. **Build cross-tool benchmark cases**  
   Define 2–3 canonical scenarios (nominal hold, gust response, perturbed run) with expected summary metrics and compare Python vs MATLAB outputs.

5. **Delay advanced MBD additions until regression harness exists**  
   Add actuator/sensor/discrete timing only after automated baseline protection is in place.

## Assumptions and simplifications stated explicitly
- This plan assumes current scripts are the accepted behavioral baseline and should be preserved unless a bug is explicitly identified.
- No claim is made that current coefficients are high-fidelity across full envelope; recommendations focus on architectural rigor and repeatability.
- No claim is made of certification-grade verification; proposed strategy is engineering-grade regression suitable for early-stage development.
