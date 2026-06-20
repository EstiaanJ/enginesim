# Description of this project
This will be a sim for IC engines which uses 1D simulation for the throttle body, air intake plennum, intake runners, exhaust runners and exhaust, and a 0D sim for the combustion chamber.

# Simulation Architecture

The simulator should have one deterministic simulation core that can run in multiple execution profiles. Do not build separate real-time and render-mode simulators. The same model graph should run with different timestep sizes, solver accuracy, mesh resolution, output rates and optional physics models.

## Execution Profiles

### Real-Time Mode
- Must prioritize bounded CPU cost, deterministic behavior and numerical stability.
- May use coarser 1D pipe discretization, simpler heat transfer and simpler combustion settings.
- Should produce stable instantaneous outputs for engine testing, control-loop experiments and interactive use.
- Should emit audio-relevant pressure signals at a rate that can be resampled or interpolated for sound output.

### Render Mode
- Must prioritize accuracy and output quality over wall-clock speed.
- May use smaller timesteps, finer 1D meshes, chamber substepping, richer heat transfer and more detailed combustion settings.
- Should produce high-quality torque curves, power curves, pressure traces, mass-flow traces and engine sound.
- Should support repeatable sweeps across RPM, throttle, load and environmental conditions.

`SimulationProfile` is an execution input, not just descriptive metadata. The single-cylinder runner owns a profile and uses it for direct step timesteps and fixed-speed sweep resolution. Engine JSON may provide a default timestep, but profile settings are the execution path that should diverge between real-time and render modes.

## Deterministic Scheduler

Start with a single-threaded fixed-step scheduler. Correctness depends on stable ordering, conservation and repeatable coupling between the 0D chamber, 1D flow boundaries and crank mechanics.

A first complete engine loop should follow this shape:

1. Advance or read crank angle and crank speed.
2. Compute cylinder volume and volume rate from slider-crank geometry.
3. Compute valve lift and effective valve areas.
4. Compute boundary flows and species fluxes.
5. Compute combustion heat/species source terms for the current crank-angle interval.
6. Update chamber mass, species and internal energy from boundary flow, piston work and combustion source terms.
7. Compute chamber pressure and gas force.
8. Convert gas force to crank torque.
9. Integrate crank/flywheel speed under gas, friction and load torque.
10. Record outputs.

Threading and parallelism should be added after this deterministic model is correct. Good later parallelization targets are independent cylinders, 1D pipe sections, audio rendering and post-processing. Threading must not change simulation results for the same inputs.

# Model Stack

## First Vertical Slice

The first target is a complete single-cylinder 0D engine loop:

- 0D chamber with moving volume.
- Slider-crank cylinder volume from crank angle.
- Prescribed intake and exhaust boundary pressure/temperature.
- Intake and exhaust valve lift curves mapped to effective area.
- Bidirectional valve/port mass flow.
- Wiebe heat release.
- Crank/flywheel inertia integration.
- Instantaneous torque, mean torque and RPM outputs.

This vertical slice should be physically coherent before adding multi-cylinder behavior or full 1D intake/exhaust pipes.

### Current Phase 2 Implementation

The current Phase 2 runner is `SingleCylinderEngine` in `src/single_cylinder.rs`. It combines:

- a 0D chamber state with total gas mass and temperature,
- slider-crank volume, `dV/dtheta`, `dV/dt` and `dx/dtheta`,
- prescribed intake and exhaust pressure/temperature boundaries,
- intake and exhaust valve events with smooth lift-to-effective-area curves,
- bidirectional mass flow through each valve,
- a fixed-delay Wiebe heat-release event,
- profile-driven timesteps for direct stepping and fixed-speed sweeps,
- indicated gas force, indicated torque, accumulated work, mean indicated torque and RPM outputs,
- fixed-RPM sweep helpers that output plot-ready torque and indicated power values.

The current runner is intentionally indicated-only. It does not yet compute brake torque, friction, pumping-loss models beyond the actual chamber pumping work, throttle/plenum/runner dynamics, wall heat transfer, or species-specific chamber composition. Those are later roadmap phases.

The chamber update currently uses the existing explicit chamber step. This keeps the integration path simple and deterministic while Phase 2 proves coupling signs, event timing and data flow. Render-mode work should later add smaller timesteps, substepping or higher-order integration where convergence tests show a need.

## Engine Definition Data

Engine definitions are JSON files loaded through serde-backed structs in `src/engine_config.rs`. JSON values should use SI units. Crank-angle fields are accepted in degrees for readability at the data-file boundary and converted to radians internally.

The first fixture is `data/engines/gn250.json`, based on the provided GN250 spec notes. Bore, stroke, compression ratio, connecting rod length, valve diameters/counts, advertised valve timing and the simple spark-advance schedule come from supplied GN250 notes. Effective flow areas, crank inertia and the simple combustion energy settings remain approximations until measured or sourced values are added.

The current crank-angle convention is:

- `0 deg` is TDC at the start of the power stroke.
- `180 deg` is BDC after expansion.
- `360 deg` is TDC between exhaust and intake.
- `540 deg` is BDC after intake.
- `720 deg` wraps back to power-stroke TDC.

Spark timing in the JSON fixture is therefore near the end of the compression stroke, before the next `720 deg` TDC.

The GN250 fixture uses these current conversions:

- Spark advance: `10 deg BTDC` below `1700 rpm` maps to `710 deg`; `35 deg BTDC` at and above `3000 rpm` maps to `685 deg`. The Phase 2 runner linearly interpolates between those RPM points.
- Intake opens `12 deg BTDC` before overlap TDC, so it maps to `348 deg`.
- Intake closes `42 deg ABDC` after intake BDC, so it maps to `582 deg`.
- Exhaust opens `45 deg BBDC` before expansion BDC, so it maps to `135 deg`.
- Exhaust closes `10 deg ATDC` after overlap TDC, so it maps to `370 deg`.
- Lambda target is fixed at `1.0` for now. Phase 2 records the value but does not yet use species-aware fueling.
- The real GN250 four-valve head is collapsed to one aggregate intake boundary and one aggregate exhaust boundary. The fixture records `2 x 26 mm` intake valves and `2 x 22 mm` exhaust valves, but the current flow model still uses an aggregate effective area.
- The current crank inertia value uses the crank plus rotor/alternator estimate range. Clutch-reflected inertia should be a load/driveline concern, not baked into the base crank value by default.

## 0D Chamber State

The chamber state should be based on conserved quantities:

- Oxygen mass.
- Fuel mass.
- Inert gas mass.
- Combustion-product gas mass.
- Total internal energy.
- Current chamber volume.

Pressure and temperature should be derived from conserved state and mixture properties. Early models may use approximate mixture properties, but the approximation must be explicit.

## Combustion And Wiebe Burn Model

The Wiebe model is the crank-angle burn-shape model, not the complete combustion model. The surrounding chamber/combustion model must decide combustibility, burnable fuel mass, oxygen availability, combustion efficiency, heat loss, remaining unburned fuel and generated products.

The recommended combustion split is:

- `x_b(theta)` describes normalized cumulative burn shape.
- `eta_comb` describes combustion efficiency and incomplete burn.
- species conversion consumes fuel and oxygen and generates products.
- chamber energy receives released heat after wall/structure losses.
- ignition delay determines start of combustion after spark.
- burn duration determines how quickly heat release progresses after start of combustion.

The standard cumulative burn fraction is:

```text
x_b(theta) = 1 - exp[-a * ((theta - theta_0) / delta_theta)^(m + 1)]
```

Where:

- `theta_0` is start of combustion, not necessarily spark timing.
- `delta_theta` is combustion duration.
- `a` is the Wiebe efficiency coefficient.
- `m` is the shape factor.

Do not force the burned fraction to exactly `1.0` at `delta_theta` unless the model explicitly chooses to normalize the curve. With `a = 5`, the standard expression reaches about 99.3 percent at the duration boundary.

Spark timing and start of combustion should be separated:

```text
theta_soc = theta_spark + ignition_delay(p_spark, T_spark, phi, residual_fraction)
```

Early implementations may use fixed ignition delay and fixed duration, but the API should leave room for state-dependent delay and duration based on pressure, temperature, equivalence ratio, residual fraction, engine speed and turbulence.

Ignition delay should eventually account for unburned-gas pressure, unburned-gas temperature, equivalence ratio, residual/EGR fraction, spark energy and turbulence/flame-kernel conditions.

Combustion duration should eventually account for engine speed, pressure and temperature at ignition or early burn, equivalence ratio, residual/EGR fraction, turbulence intensity, chamber geometry or characteristic flame travel distance and spark timing through its effect on in-cylinder conditions.

Incomplete combustion should be separate from burn shape:

```text
Q_released = eta_comb * m_fuel_burnable * LHV * delta_x_b
```

`eta_comb` may depend on equivalence ratio, oxygen availability, residual/EGR fraction, pressure and temperature, wall quench, misfire, cyclic variability and turbulence. Partial burn or misfire must leave physically meaningful remaining species: unburned fuel, unused oxygen and generated products should all match the amount of fuel actually burned.

Avoid double-counting combustion effects. If pressure and temperature affect ignition delay, do not also apply the same correction again to start-of-combustion timing. If equivalence ratio affects both duration and combustion efficiency, duration should affect timing while efficiency affects released energy.

Initial spark-ignition starting values:

- `a = 5`
- `m = 2`
- fixed ignition delay around 5 to 15 degCA
- duration around 30 to 50 degCA for normal stoichiometric SI operation

Combustion heat release must be integrated using the delta in cumulative burned fraction over each crank-angle step. AFR, pressure, temperature and residual fraction used for combustion decisions must come from the correct chamber state at the event time.

## 1D Intake And Exhaust

The eventual intake and exhaust model should use finite-volume compressible flow. Pipe cells should track the state needed for pressure waves, flow reversal and acoustic output. The 0D chamber should couple to the 1D model through valve boundary fluxes that conserve mass, species and energy.

Until full 1D pipes exist, prescribed pressure/temperature boundaries are acceptable for the first vertical slice.

## Crank And Mechanical Model

The initial engine loop should model one independent crank/flywheel inertia. Cylinder gas force should be projected to crank torque through slider-crank geometry. General spring/damper mechanical links are useful tools, but rigid piston/crank coupling should eventually be solved as a coupled kinematic/inertia problem rather than as a springy approximation.

## Outputs

Core outputs should include:

- Crank angle and RPM.
- Instantaneous cylinder pressure and temperature.
- Chamber species masses.
- Intake and exhaust mass-flow rates.
- Instantaneous indicated torque.
- Cycle-averaged indicated torque.
- Brake torque and power once friction/load models exist.
- Exhaust and intake pressure traces for sound generation.
- Sweep outputs for torque and power curves.

## GUI And Visual Feedback

The GUI is a human-verification tool for watching whether the current model behaves plausibly while development continues. It should use the deterministic simulation core as its data source and should keep GUI refresh, telemetry publication and simulation timesteps as separate concerns.

The GUI should be implemented as a thin application layer over the library crate. It should not duplicate simulator state transition logic. A telemetry layer should sit between the engine model and the GUI widgets so averaging, cycle accumulation, peak tracking and plot buffering are deterministic and testable without a windowing backend.

### Current GUI Vertical Slice

The current GUI implementation is `src/bin/engine_gui.rs`, backed by `src/telemetry.rs`. It loads `data/engines/gn250.json` by default, steps the existing `SingleCylinderEngine`, and only renders telemetry snapshots rather than reading raw engine state directly from egui.

The current GUI wiring intentionally stays within Phase 2 physics:

- Starter torque, starter speed limit, added inertia, added torque load, spark enable, fuel enable and dyno fixed-RPM mode are mapped into single-cylinder step inputs.
- Dyno mode currently uses the existing fixed-crank-speed path and is therefore an explicitly unrealistic forced-rotation diagnostic mode.
- Throttle position, idle leak and lambda target are exposed in the controls panel, but they remain placeholders until intake restriction, plenum state and species-aware fueling exist.
- Chamber lambda is currently displayed as the lambda target placeholder.
- Exhaust lambda is currently unavailable.
- MAP currently displays the prescribed intake boundary pressure until a real manifold or plenum state exists.
- Fuel flow is currently estimated from configured fuel mass per cycle and engine speed.
- Air flow is currently estimated from integrated intake mass flow.

This is a development dashboard, not a calibrated operator interface. Its purpose is to make timing, coupling, pressure, temperature, torque and control interactions visible while the physics stack is still incomplete.

### GUI Data Rates And Averaging

GUI refresh rate and simulation timestep are different rates. The simulation should advance at the fixed timestep from the active `SimulationProfile`. Telemetry should be published into GUI-facing buffers at explicit display rates:

- Engine data panel: `15 Hz`.
- Engine-angle plots: `15 Hz`.
- Time plots: `30 Hz`.
- RPM-based torque/power plots: `30 Hz`.

Values shown at `15 Hz` must not be only the latest simulation sample at the display tick. They should use sensible averaging over the samples accumulated since the previous display update, followed by a rolling average over the latest 15 published display frames where that produces a more readable signal.

Different values need different aggregation:

- Mean-like values such as RPM, torque, power, fuel flow, air flow and MAP should use rolling averages.
- Cycle-varying values that are meaningful over an engine cycle should also have cycle accumulators where appropriate.
- RPM delta should be the difference between maximum RPM and minimum RPM over the current or most recently completed 720 degree engine cycle.
- Peak cylinder pressure and peak cylinder temperature should be maxima over the current or most recently completed cycle, not arithmetic averages.
- Combustion event ratio should be counted as combustion events divided by possible combustion strokes over the tracked cycle window.

### Engine Data Panel

The engine data panel should update at `15 Hz` and should use rolling display-frame averaging where appropriate:

- RPM.
- RPM delta (difference between peak rpm and lowest rpm in a single cycle) averaged over the sampling time.
- Torque.
- Power.
- Fuel flow in `mg/s`.
- Air flow in `g/s`.
- Measured lambda in the combustion chamber.
- Measured lambda from oxygen and fuel in the exhaust.
- Peak cylinder temperature.
- Peak cylinder pressure.
- MAP.
- Combustion event ratio.

Some requested values are not physically available in the current Phase 2 model. Until Phase 3 species-aware chamber state and later exhaust composition exist, GUI values for chamber lambda, exhaust lambda, fuel mass in the chamber and exhaust oxygen/fuel should be marked as placeholders or unavailable rather than presented as validated physics. Fuel flow may be estimated from configured fuel mass per combustion event, air flow may be accumulated from intake mass flow, and MAP may use the prescribed intake boundary pressure until a manifold/plenum model exists.

### Engine-Angle Plots

Engine-angle plots should use total engine angle from `0` to `720 deg` and should update at `15 Hz`.

If a complete engine cycle takes less than the display interval, plot the completed cycle trace. If a single cycle takes longer than `1/15 s`, plot a live trace showing progress through the current cycle and replace or complete the trace as new crank-angle bins arrive.

Required engine-angle plots:

- Cylinder pressure.
- Cylinder temperature.
- Intake and exhaust effective area.
- Air mass and fuel mass.

Air mass and fuel mass should be true chamber species values once species-aware chamber state exists. Until then, total chamber gas mass and configured/inferred fuel-event quantities may be displayed only as explicitly labelled placeholders.

### Time Plots

Time plots should update at `30 Hz` and use time as the x-axis:

- RPM.
- MAP.
- Torque.

These plots should use bounded rolling buffers so long GUI sessions do not grow memory without limit.

### RPM Plots

RPM-based plots should update at `30 Hz` and should use a fading trace:

- Torque versus RPM.
- Power versus RPM.

Torque and power plotted against RPM should prefer cycle-averaged samples or dyno-sweep samples over instantaneous crank-angle samples. Instantaneous torque can vary strongly within one engine cycle and should not be used as the main torque curve without cycle averaging.

### Controls

GUI controls should write to a single control-state struct consumed by the simulation runner. Controls may exist before every control path is physically implemented, but unimplemented controls should be visibly disabled or routed to documented placeholder behavior.

Required controls:

- Throttle position.
- Idle leak amount.
- Combined throttle command derived from throttle position and idle leak amount.
- Lambda target.
- Starter motor torque.
- Starter motor RPM limit, with behavior that allows the engine to overrun the starter.
- Starter motor toggle.
- Added inertia.
- Added torque load.
- Spark toggle.
- Fuel toggle.
- Redline spark-cut RPM input.
- Dyno mode toggle.
- Dyno target.

Dyno mode is intentionally allowed to force engine rotation in an unrealistic absolute way. It is a diagnostic and calibration mode, not a physical load model.

# Timing And Resolution

The core should use fixed timesteps for determinism. Timestep size is a profile setting, not a hard-coded global. Some submodels may substep internally when needed for stability or accuracy.

Initial target settings:

- Real-time mode: choose the largest timestep that remains stable and physically plausible for the selected model detail.
- Render mode: use smaller timesteps and optional substeps until outputs converge within chosen tolerances.
- 0D chamber and crank integration should remain synchronized.
- 1D flow timesteps must respect the relevant wave-speed/CFL stability limits once finite-volume pipes are implemented.

# Testing And Validation Strategy

Testing is part of the simulator design. The project should use a layered test suite that protects basic math, model contracts, coupled behavior, numerical convergence and saved reference outputs.

## Test Categories

### Unit Tests

Unit tests should be fast and deterministic. They should live next to the modules they validate and should run on every code change.

Required unit-test coverage:

- Geometry helpers: bore area, swept volume, clearance volume, slider-crank volume and `dx/dtheta` once exposed.
- Gas helpers: ideal-gas pressure, density, heat capacities and temperature-from-energy.
- Flow helpers: choked/unchoked limits, zero-flow cases, bidirectional signs and monotonic mass-flow behavior with pressure ratio and area.
- Chamber helpers: closed-volume pressure/temperature response, enthalpy accounting for inlet/outlet flow and lower-bound behavior.
- Combustion helpers: AFR, stoichiometric limiting, mixture-efficiency limits, Wiebe burn shape and incomplete-burn behavior.
- Mechanical helpers: force/torque sign conventions, wrapped angle behavior, inertia and simple integration cases.

Unit tests should prefer physical invariants and limiting cases over snapshots of arbitrary intermediate values.

### Integration Tests

Integration tests should validate coupled behavior across modules. They should be deterministic and should use small fixtures that are easy to inspect.

Required integration-test coverage:

- Closed adiabatic compression/expansion should approximately conserve energy and follow expected pressure/temperature trends.
- Open chamber filling and blowdown should conserve mass and energy across boundaries within numerical tolerance.
- A single-cylinder motored case with no combustion should produce plausible compression/expansion pressure traces and near-zero net indicated work over a closed cycle, excluding modeled losses.
- A single-cylinder fired case should produce positive indicated work when heat release occurs during the correct crank-angle window.
- Combustion integration should validate heat-release timing, pressure response, oxygen/fuel consumption and product generation.
- Valve overlap cases should allow flow reversal when pressure conditions require it.
- Fixed-RPM torque sweep fixtures should produce stable, repeatable cycle-averaged torque for identical inputs.

### Regression Tests

Regression tests should protect known-good simulator outputs from accidental drift. They are not a substitute for physical validation, but they are useful once an output is intentionally accepted.

Regression baselines should record:

- Simulator version or commit identifier.
- Model/profile settings.
- Engine configuration.
- Environmental inputs.
- Timestep and substep settings.
- Key scalar outputs such as mean torque, power, peak pressure, IMEP and fuel consumed.
- Optional sampled traces such as pressure vs crank angle or exhaust pressure over time.

Regression tolerances must be explicit. Tight tolerances are appropriate for deterministic math helpers. Wider tolerances are acceptable for high-level outputs when solver changes intentionally alter numerical behavior.

### Convergence Tests

Render-mode credibility depends on convergence. Important outputs should be compared across timestep and mesh refinements.

Convergence checks should include:

- 0D timestep refinement for cylinder pressure, work and torque.
- Chamber substep refinement around combustion and valve events.
- Future 1D mesh refinement for intake/exhaust pressure waves.
- Audio-signal convergence checks for pressure trace timing and amplitude.

The render profile should eventually have documented convergence thresholds for torque, power, peak pressure and pressure-trace timing.

### Accuracy Validation

Accuracy validation should compare model outputs against analytical cases first, then controlled reference cases, then real engine data when available.

Validation sources, in preferred order:

- Analytical thermodynamic cases: ideal-gas relations, adiabatic compression/expansion and nozzle-flow limits.
- Published or hand-calculated reference cases for chamber filling, blowdown and simple heat release.
- Known engine geometry sanity checks.
- Dynamometer torque/power curves when real data is available.
- Recorded exhaust/intake pressure or sound only after the pressure-wave model is mature.

When real data is used, the exact assumptions and calibration parameters must be documented with the fixture.

## Test Execution Profiles

The test suite should have at least three practical tiers:

- Fast tests: unit tests and small integration tests suitable for every edit.
- Validation tests: slower coupled tests and convergence checks run before major model changes are accepted.
- Render baselines: high-accuracy reference runs used intentionally, not on every edit unless they are cheap enough.

Fast tests should remain deterministic and should not depend on wall-clock timing, threads or random seeds. Randomized tests may be added only if the seed is fixed and printed on failure.

## Accuracy And Regression Rules

- Every physics/model change should add or update at least one test that would fail for the bug or missing behavior being addressed.
- Conservation tests should track mass, species and energy budgets explicitly.
- Engine-loop tests should record both instantaneous traces and cycle-averaged quantities where practical.
- If a regression baseline changes intentionally, the commit or change description should explain why the new output is more correct.
- Tests should use SI units internally and name tolerances clearly.


# Units & Standards
Always use SI units internally
External specifications, GUI and other things will be a mixture of units, mostly metric

# Physics Requirements

## Charge Composition
- The chamber model must track oxygen mass, fuel mass, inert gas mass and total combined charge mass.
- Combustion must consume fuel and oxygen and generate combustion-product gas mass instead of only adding heat to an unchanged air mass.
- Pressure and temperature calculations must use gas properties that are consistent with the current chamber composition, even if an early implementation uses approximate mixture properties.

## 0D / 1D Coupling
- Boundary flow between 1D elements and 0D chambers must be mass conservative on both sides of the interface.
- Valve and port flow must support both directions of flow so intake backflow and exhaust reversion are represented.
- Prescribed source terms are only allowed when they also define the corresponding source or sink state required for conservation.

## Intake And Manifold Behavior
- Future manifold and plenum models must not assume fixed temperature unless an explicit isothermal boundary is being modeled.
- Manifold/plenum updates must account for mass flow enthalpy, heat transfer assumptions, throttling losses and pressure-wave effects from the 1D model.
- Naturally aspirated components must not clamp pressure to upstream pressure in a way that prevents ram, resonance or transient wave effects.

## Combustion Timing State
- Combustion and AFR decisions must use the physically correct state at the decision point.
- If intake flow, exhaust flow, injection and spark occur during the same integration interval, the model must define and document the event order or use substeps so combustion does not use stale charge mass.

## Moving-Boundary Integration
- Moving chambers must evaluate volume, volume rate, heat transfer and boundary flow states at the integration substep times.
- Higher-order integrators such as RK4 are only valid for moving chambers when the moving boundary and flow boundary inputs are also evaluated at the intermediate RK stages.

## Mechanical Coupling
- Rigid dependent bodies must not reflect inertia to the parent using stale acceleration from a previous integration step.
- Rigid linear/rotational and rotational-ratio constraints should be solved as coupled inertia or constraint equations in the same timestep.
- The project must document one sign convention for piston force direction, positive crank rotation, positive gas torque and positive load torque.
- Wrapped rotational displacement must be computed as a signed wrapped displacement, not plain `new_angle - old_angle`.
