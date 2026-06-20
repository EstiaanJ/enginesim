# Roadmap

Status note: the implementation has already moved through Phase 5. The earlier phase sections are kept as a historical roadmap and for traceability, while the remaining unchecked items describe later work.

## Phase 1: Stabilize The Simulation Core

- [x] Define common model interfaces.
  - [x] Define a step API shape for stateful models: inputs, constants/config, timestep and outputs.
  - [x] Define naming conventions for state, inputs, constants and update structs.
  - [x] Define deterministic event ordering for crank, valves, flow, combustion, chamber update and mechanics.
  - [x] Add tests for deterministic repeatability with fixed inputs.

- [x] Establish the test harness structure.
  - [x] Keep fast unit tests next to the modules they validate.
  - [x] Create integration tests for coupled chamber/geometry/combustion/mechanics behavior.
  - [x] Add shared tolerance helpers for approximate comparisons.
  - [x] Add test fixtures for common gases, engine geometry and profile settings.
  - [x] Document how to run fast tests, validation tests and render baseline tests.

- [x] Define shared simulation profiles.
  - [x] Add `real_time` and `render` profile concepts.
  - [x] Put timestep, substep count, mesh resolution and output rate behind profile settings.
  - [x] Ensure profile changes do not require different model code paths.
  - [x] Make the single-cylinder loop use profile timesteps for direct stepping and fixed-speed sweeps.

- [x] Document and enforce sign conventions.
  - [x] Define positive crank rotation.
  - [x] Define positive piston displacement and force direction.
  - [x] Define positive gas torque, load torque and friction torque.
  - [x] Add focused tests for force-to-torque projection signs.

- [x] Phase 1 testing and validation.
  - [x] Ensure all model-interface examples have deterministic repeatability tests.
  - [x] Ensure all sign conventions have positive and negative case tests.
  - [x] Ensure profile settings can be serialized or inspected in test output.
  - [x] Ensure `cargo test` remains the fast default test command.

## Phase 1A: Testing, Validation And Regression Plan

- [x] Expand unit-test coverage for math and helper modules.
  - [x] Geometry: bore area, swept volume, clearance volume and slider-crank volume limits.
  - [x] Geometry: add and test `dx/dtheta` when crank torque projection is implemented.
  - [x] Geometry: add and test `dV/dtheta` and `dV/dt` helpers for moving-volume chamber integration.
  - [x] Gas: pressure, density, `cv`, `cp` and temperature-from-internal-energy limiting cases.
  - [x] Flow: choked/unchoked transition, zero-flow cases, bidirectional signs and monotonic behavior.
  - [x] Combustion: AFR, stoich limiting, mixture-efficiency bounds and Wiebe burn curve shape.
  - [x] Mechanics: sign conventions, wrapped angles and simple inertia integration.

- [x] Add 0D chamber conservation integration tests.
  - [x] Closed adiabatic compression/expansion energy behavior.
  - [x] Open chamber filling mass and energy budget.
  - [x] Open chamber blowdown mass and energy budget.
  - [x] Heat-addition pressure/temperature response at fixed volume.
  - [x] Moving-volume pressure/work consistency.

- [x] Add first single-cylinder vertical-slice integration tests.
  - [x] Motored no-combustion cycle produces finite, repeatable outputs.
  - [ ] Motored no-combustion cycle net indicated work near zero, excluding modeled losses, after valve/heat-transfer assumptions are calibrated.
  - [x] Fired single-cylinder cycle produces more indicated work than the motored case with sane heat-release timing.
  - [x] Valve overlap allows flow reversal when pressure conditions require it.
  - [x] Repeating the same fixed-RPM case gives identical outputs.

- [x] Add regression baseline infrastructure.
  - [x] Define a small serialized output format for scalar baselines.
  - [x] Record model/profile settings with every baseline.
  - [x] Record engine configuration and environmental conditions with every baseline.
  - [x] Define explicit tolerances per output.
  - [x] Add a workflow for intentionally accepting changed baselines.

- [x] Add render-mode convergence checks.
  - [x] Add common convergence comparison helper.
  - [x] Compare torque/work/peak-pressure outputs across timestep reductions.
  - [x] Compare chamber substep settings around valve and combustion events.
  - [x] Add future 1D mesh refinement checks once pipe models exist.
  - [x] Add pressure-trace timing/amplitude checks for future sound generation.

- [ ] Add accuracy validation fixtures.
  - [x] Analytical ideal-gas cases.
  - [x] Analytical adiabatic compression/expansion cases.
  - [x] Nozzle/isentropic mass-flow reference cases.
  - [x] Hand-calculated or published simple chamber filling/blowdown cases.
  - [ ] Real dyno or pressure-trace data fixtures when available.

## Phase 2: Single-Cylinder 0D Vertical Slice

- [x] Build a single-cylinder engine loop.
  - [x] Add a cylinder/chamber wrapper that combines chamber state, geometry, valve states and combustion state.
  - [x] Use slider-crank geometry to compute cylinder volume from crank angle.
  - [x] Use slider-crank `dx/dtheta` to project gas force to crank torque with the documented sign convention.
  - [x] Use slider-crank `dV/dtheta` and crank speed to compute chamber volume rate.
  - [x] Step the chamber from boundary flow, combustion heat release and piston work.
  - [x] Output pressure, temperature, total chamber mass, gas force and indicated torque.
  - [x] Output species masses after Phase 3 adds species-aware chamber state.

- [x] Add valve events and effective area.
  - [x] Represent valve timing, duration and lift curve.
  - [x] Convert lift to effective flow area.
  - [x] Use bidirectional flow at intake and exhaust boundaries.
  - [x] Test intake filling, exhaust blowdown and flow reversal cases.

- [x] Add crank/flywheel integration.
  - [x] Model one independent crank inertia.
  - [x] Project gas force to crank torque using `dx/dtheta`.
  - [x] Add simple external load torque.
  - [x] Output instantaneous torque, mean torque and RPM.

- [x] Add basic torque/power sweep tooling.
  - [x] Run fixed-RPM cases over RPM points.
  - [x] Add throttle/load-range sweep dimensions once controls and throttle/plenum coupling are in place.
  - [x] Compute cycle-averaged torque.
  - [x] Compute power from torque and RPM.
  - [x] Trim the final fixed-speed step so cycle runs integrate exactly the requested crank angle.
  - [x] Emit structured output suitable for plotting.

- [x] Add JSON engine data path.
  - [x] Add serde-backed engine definition structs.
  - [x] Add a GN250 JSON fixture using the provided spec values where available.
  - [x] Convert GN250 spark timing, valve timing, rod length, valve diameters/counts, and crank inertia notes into the JSON fixture.
  - [x] Mark crank inertia interpretation, aggregate effective flow areas and combustion energy values as approximations.

- [x] Phase 2 testing and validation.
  - [x] Add integration tests for a full motored cycle.
  - [x] Add integration tests for a full fired cycle.
  - [x] Check cycle work by integrating torque over crank angle and comparing against reported mean indicated torque.
  - [x] Check deterministic repeatability for identical single-cylinder runs.
  - [x] Add tests that profile timestep drives the single-cylinder loop.
  - [x] Add saved regression baselines for pressure trace shape, peak pressure, indicated work and mean torque once the placeholder GN250 values are calibrated enough to preserve.
  - [x] Add timestep-refinement checks for the single-cylinder loop.

## Phase 2A: GUI And Visual Feedback

- [x] Add a GUI application binary.
  - [x] Add `eframe` and `egui_plot` dependencies.
  - [x] Create a thin GUI binary, such as `src/bin/engine_gui.rs`.
  - [x] Keep simulation stepping in the library crate rather than duplicating engine logic in the GUI.
  - [x] Load the GN250 fixture as the initial engine configuration.

- [x] Add a GUI telemetry layer.
  - [x] Add a telemetry module with GUI-facing sample, frame and plot-buffer structs.
  - [x] Add a control-state struct for GUI inputs before they are applied to the simulation runner.
  - [x] Keep telemetry aggregation deterministic and testable without a windowing backend.
  - [x] Separate simulation timestep, telemetry publication rates and GUI repaint rate.

- [x] Implement engine-data aggregation.
  - [x] Publish engine data panel values at `15 Hz`.
  - [x] Use per-display-interval accumulation rather than only sampling the latest sim step.
  - [x] Add rolling averaging over the latest 15 published engine-data frames where appropriate.
  - [x] Compute RPM delta from max minus min RPM over the current or most recently completed 720 degree cycle.
  - [x] Track peak cylinder pressure and peak cylinder temperature as cycle maxima, not averages.
  - [x] Track combustion event ratio as combustion events divided by possible combustion strokes over the selected cycle window.

- [x] Define placeholder telemetry for values not physically available yet.
  - [x] Mark chamber lambda as unavailable or placeholder until species-aware chamber state exists.
  - [x] Mark exhaust lambda as unavailable or placeholder until exhaust oxygen/fuel/product state exists.
  - [x] Estimate fuel flow from configured fuel mass per combustion event until species-aware fueling exists.
  - [x] Estimate air flow from integrated intake mass flow.
  - [x] Use prescribed intake boundary pressure as temporary MAP until a manifold/plenum state exists.
  - [x] Avoid presenting placeholder values as validated physical measurements.

- [ ] Add engine data panel.
  - [x] Display RPM.
  - [x] Display cycle RPM delta.
  - [x] Display torque.
  - [x] Display power.
  - [x] Display fuel flow in `mg/s`.
  - [x] Display air flow in `g/s`.
  - [x] Display measured chamber lambda.
  - [x] Display measured exhaust lambda.
  - [x] Display peak cylinder temperature.
  - [x] Display peak cylinder pressure.
  - [x] Display MAP.
  - [x] Display combustion event ratio.

- [x] Add engine-angle plots.
  - [x] Update engine-angle plots at `15 Hz`.
  - [x] Use total engine angle from `0` to `720 deg`.
  - [x] Plot completed cycle traces when a cycle completes faster than the display interval.
  - [x] Plot a live in-progress cycle trace when one cycle takes longer than `1/15 s`.
  - [x] Plot cylinder pressure.
  - [x] Plot cylinder temperature.
  - [x] Plot intake and exhaust effective area.
  - [x] Plot chamber air mass and fuel mass, using clearly labelled placeholders until species state exists.

- [x] Add time plots.
  - [x] Update time plots at `30 Hz`.
  - [x] Use bounded rolling time buffers.
  - [x] Plot RPM over time.
  - [x] Plot MAP over time.
  - [x] Plot torque over time.

- [x] Add GUI controls.
  - [x] Add throttle position control.
  - [x] Add idle throttle amount control.
  - [x] Model throttle position and idle throttle as parallel throttle valves.
  - [x] Add lambda target control.
  - [x] Add starter motor torque control.
  - [x] Add starter motor RPM limit control that allows the engine to overrun the starter.
  - [x] Add starter motor toggle.
  - [x] Add added inertia control.
  - [x] Add added torque load control.
  - [x] Add spark toggle.
  - [x] Add fuel toggle.
  - [x] Add redline spark-cut RPM input.
  - [x] Add dyno mode toggle.
  - [x] Add dyno target control.
  - [x] Implement dyno mode as an intentionally unrealistic absolute forced-rotation diagnostic mode.

- [x] Phase 2A testing and validation.
  - [x] Unit-test rolling average behavior.
  - [x] Unit-test cycle peak tracking for pressure and temperature.
  - [x] Unit-test cycle RPM delta tracking.
  - [x] Unit-test 720 degree plot binning and live-trace replacement behavior.
  - [x] Unit-test bounded rolling buffers for time plots.
  - [x] Unit-test control-state mapping into simulation inputs or documented placeholders.
  - [x] Add a smoke test that the GUI binary compiles when GUI dependencies are enabled.
  - [x] Add GUI-facing tests for placeholder status rendering and multi-rate publication cadence.

## Phase 3: Species-Aware Combustion

- [x] Replace air-only chamber mass with species masses.
  - [x] Track oxygen mass.
  - [x] Track fuel mass or fuel vapor mass.
  - [x] Track inert gas mass.
  - [x] Track combustion-product gas mass.
  - [x] Track total charge mass as a derived quantity.

- [x] Implement conservative combustion conversion.
  - [x] Consume fuel and oxygen according to stoichiometric limits.
  - [x] Generate combustion products.
  - [x] Release heat from burned fuel mass and LHV.
  - [x] Keep incomplete combustion and misfire behavior explicit.

- [x] Implement Wiebe-based heat-release event model.
  - [x] Add an explicit combustion-event state, such as `CombustionEvent` or `ActiveCombustion`.
  - [x] Track start-of-combustion angle, burn duration, burnable fuel snapshot, previous cumulative Wiebe fraction, consumed fuel, consumed oxygen, generated products and released heat.
  - [x] Separate spark timing from start of combustion.
  - [x] Add fixed ignition delay first, with API space for state-dependent delay.
  - [x] Add API inputs for pressure, temperature, equivalence ratio, residual fraction, spark energy and turbulence effects on ignition delay.
  - [x] Add fixed duration first, with API space for state-dependent duration.
  - [x] Add API inputs for speed, pressure, temperature, equivalence ratio, residual fraction, turbulence and chamber geometry effects on duration.
  - [x] Use cumulative Wiebe burned fraction and per-step burned-fraction delta.
  - [x] Keep incomplete combustion in combustion efficiency/species conversion rather than forcing the burn shape.
  - [x] Keep crank-angle inputs in radians internally.

- [x] Add combustion-efficiency and partial-burn model.
  - [x] Keep `eta_comb` separate from Wiebe burn shape.
  - [x] Make released heat depend on burned fuel mass, LHV, combustion efficiency and burned-fraction delta.
  - [x] Preserve unburned fuel and unused oxygen after partial burn or misfire.
  - [x] Generate products only from actually burned fuel.
  - [x] Avoid double-counting pressure, temperature, equivalence ratio, spark timing and turbulence effects across delay, duration and efficiency.

- [x] Add mixture property approximation.
  - [x] Compute effective gas constant from composition.
  - [x] Compute effective heat capacity from composition.
  - [x] Keep approximate properties documented and replaceable.

- [x] Phase 3 testing and validation.
  - [x] Unit-test Wiebe burn fraction at start, midpoint and duration.
  - [x] Unit-test monotonic burned fraction for normal events.
  - [x] Unit-test that `a = 5` does not force exactly complete burn at duration unless normalization is explicitly enabled.
  - [x] Unit-test ignition-delay factor directions for pressure, temperature, equivalence ratio, residual fraction, spark energy and turbulence once implemented.
  - [x] Unit-test duration factor directions for speed, pressure, temperature, equivalence ratio, residual fraction and turbulence once implemented.
  - [x] Test oxygen-limited, fuel-limited, rich, lean and misfire cases.
  - [x] Test conservation of fuel, oxygen, inert gas, products and total mass through combustion.
  - [x] Test heat release from burned fuel mass and LHV.
  - [x] Test partial burn leaves physically meaningful remaining fuel, oxygen and products.
  - [x] Test delay, duration and efficiency corrections are not double-counted in a single calibration path.
  - [x] Add fixed-volume heat-release integration tests for pressure and temperature response.
  - [x] Add regression baselines for pressure trace sensitivity to ignition delay, duration and equivalence ratio.

## Phase 4: Multi-Cylinder Engine

- [ ] Add engine configuration.
  - [ ] Represent cylinder count, firing order and crank phase offsets.
  - [ ] Share crank/flywheel inertia across cylinders.
  - [ ] Aggregate cylinder torque onto the crank.

- [ ] Add cycle-level outputs.
  - [ ] Per-cylinder pressure traces.
  - [ ] Total instantaneous crank torque.
  - [ ] Cycle-averaged indicated torque.
  - [ ] RPM stability and cyclic variation metrics.

- [ ] Phase 4 testing and validation.
  - [ ] Test firing-order and crank-phase scheduling for common engine layouts.
  - [ ] Test torque aggregation from multiple cylinders onto the shared crank.
  - [ ] Test cylinder isolation: identical cylinders with phase offsets should produce phase-shifted matching traces.
  - [ ] Test repeatability of multi-cycle multi-cylinder runs.
  - [ ] Add regression baselines for total torque ripple, mean torque and per-cylinder peak pressure.
  - [ ] Add validation checks that changing firing order changes torque trace phase but not mean torque for identical cylinders.

## Phase 5: 1D Intake And Exhaust

- [x] Add finite-volume 1D pipe primitives.
  - [x] Represent pipe cells with pressure, temperature, density/species and velocity.
  - [x] Implement stable flux calculations between cells.
  - [x] Respect CFL/wave-speed timestep limits.
  - [x] Add tests for wave propagation and reflection.

- [x] Couple 1D pipes to 0D chambers.
  - [x] Implement valve boundary fluxes that conserve mass, species and energy.
  - [x] Support pressure waves, flow reversal and exhaust reversion.
  - [x] Replace prescribed intake/exhaust boundaries in the engine loop.

- [x] Add throttle, plenum and runner models.
  - [x] Connect throttle flow to plenum state.
  - [x] Connect plenum to intake runners.
  - [ ] Model exhaust runners and collector.
  - [ ] Model exhaust runners collector (multi cylinder).
  - [ ] Model main exhaust pipe (multi cylinder).
  - [ ] Preserve pressure-wave behavior needed for sound and tuning effects.

- [x] Phase 5 testing and validation.
  - [x] Unit-test finite-volume flux functions against analytical limiting cases.
  - [x] Integration-test pressure-wave propagation speed.
  - [x] Integration-test wave reflection at open and closed boundaries.
  - [x] Test mass, species and energy conservation across pipe cells and 0D/1D boundaries.
  - [x] Test intake backflow and exhaust reversion with valve overlap.
  - [x] Add CFL stability tests or assertions for configured timesteps.
  - [x] Add mesh-refinement convergence checks for pressure-wave timing and amplitude.
  - [x] Add regression baselines for intake/exhaust pressure traces once runner, plenum and exhaust geometry are configured data rather than placeholder defaults.
  
  - [ ] engine_gui Exhaust exit pressure pulse with respect to engine angle
  - [ ] engine_gui Exhaust exit pressure with respect to time

## Phase 6: Losses, Controls And Calibration

- [ ] Add friction and pumping losses.
  - [ ] Start with simple speed/load-dependent friction.
  - [ ] Separate indicated torque from brake torque.
  - [ ] Validate brake power calculations.

- [ ] Add environmental and operating controls.
  - [ ] Throttle position.
  - [ ] Ambient pressure and temperature.
  - [ ] Fueling target.
  - [ ] Spark timing.
  - [ ] External load or controlled RPM mode.

- [ ] Add calibration fixtures.
  - [ ] Known geometry sanity cases.
  - [ ] Conservation checks.
  - [ ] Sensitivity checks for AFR, spark timing, compression ratio and RPM.

- [ ] Phase 6 testing and validation.
  - [ ] Test friction/loss models against hand-calculated cases.
  - [ ] Test indicated-to-brake torque and power calculations.
  - [ ] Test controlled-RPM and external-load modes.
  - [ ] Test throttle, fueling, spark and ambient-condition input changes independently.
  - [ ] Add sensitivity tests for AFR, spark timing, compression ratio, RPM and load.
  - [ ] Add regression baselines for torque and power sweeps with fixed calibration.
  - [ ] Compare calibration fixtures against available reference or dyno data when available.

## Phase 7: Sound And Render Outputs

- [ ] Add audio signal extraction.
  - [ ] Emit exhaust pressure traces at sufficient rate.
  - [ ] Emit intake pressure traces when useful.
  - [ ] Define resampling/interpolation into audio sample rate.
  - [ ] Keep sound generation deterministic for a fixed simulation output.

- [ ] Add render-mode sweep outputs.
  - [ ] Torque curve.
  - [ ] Power curve.
  - [ ] Per-cylinder pressure traces.
  - [ ] Intake/exhaust pressure wave traces.
  - [ ] Mass-flow and combustion diagnostics.

- [ ] Add convergence checks for render mode.
  - [ ] Compare output changes across timestep reductions.
  - [ ] Compare output changes across 1D mesh refinements.
  - [ ] Record profile settings with output data.

- [ ] Phase 7 testing and validation.
  - [ ] Test audio resampling/interpolation determinism.
  - [ ] Test pressure trace to audio-signal pipeline with fixed fixtures.
  - [ ] Add regression baselines for torque curve and power curve outputs.
  - [ ] Add regression baselines for representative exhaust pressure/audio traces.
  - [ ] Add render-mode convergence checks for torque, power, peak pressure and pressure-trace timing.
  - [ ] Add output schema tests for all render-mode artifacts.
  - [ ] Ensure render outputs record profile settings, engine configuration, environment and tolerances.

# Fixes & Cleanup

## Physics / Thermodynamics

- [x] Replace cylinder flow energy terms with open-control-volume enthalpy terms.
  Removed the old cylinder thermodynamic model. New 0D chamber work should use `src/physics/chamber.rs`, which uses enthalpy for flow boundaries.

- [x] Add bidirectional valve flow support.
  Added signed bidirectional compressible-flow support in `src/physics/flow.rs`. Future valve/port models should use it or an equivalent 1D boundary formulation.

- [x] Introduce typed bidirectional flow boundary inputs.
  Added `FlowEndpoint`, `FlowOrifice`, and `GasFlowProperties` to avoid long fragile argument lists at bidirectional coupling points.

- [x] Decide how combustion products and oxygen consumption should be represented.
  Chamber state now tracks oxygen, fuel, inert gas and combustion products; combustion consumes oxygen and fuel, generates product mass and uses approximate composition-dependent gas properties.

- [x] Rework or retire the old cylinder-specific thermodynamic update.
  Removed `src/thermodynamic_model.rs`.

- [x] Make prescribed intake mass conservative.
  Removed the old prescribed-intake cylinder path. Added design requirements that future prescribed source terms must define the corresponding source/sink state required for conservation.

- [x] Reconsider the isothermal intake manifold update.
  Removed the old isothermal manifold update and added design requirements for non-isothermal, 1D-aware manifold/plenum behavior.

- [x] Remove the naturally aspirated manifold pressure cap or make it model-dependent.
  Removed the old manifold pressure clamp and documented that future models must not suppress ram, resonance or transient wave effects.

- [x] Use gas constants consistently in throttle flow.
  Replaced the old helper with `src/throttle.rs`; throttle mass flow now accepts gas constant and gamma explicitly.

- [x] Clamp and validate throttle effective area inputs.
  `src/throttle.rs` clamps throttle position and rejects invalid area/range inputs.

- [x] Compute combustion AFR from the correct in-step mass state.
  Removed the old stale-state combustion path and documented the future event-order/substep requirement.

- [x] Make piston work and reported indicated torque use a consistent pressure approximation.
  Removed the old inconsistent cylinder work/torque path.

- [x] Validate cylinder geometry instead of masking impossible values.
  `src/engine_geometry.rs` now rejects invalid compression ratio and impossible slider-crank geometry.

- [x] Update RK4 chamber integration to evaluate moving boundaries at substeps.
  Added `step_rk4_with_stage_inputs` so moving volume, heat and flow boundary inputs can be evaluated at each RK4 stage; `step_rk4` remains a constant-input convenience wrapper.

- [x] Decide whether Wiebe burn should force exactly complete combustion at duration.
  `src/combustion.rs` keeps the normal Wiebe value at duration instead of forcing exact completion.

- [x] Clean up AFR limit naming/comments.
  Removed the stale legacy comments with the old thermodynamic model.

## Mechanics / Integration

- [x] Remove one-step-lag inertia reflection from dependent bodies.
  Dependent linear and rotational inertia is folded into the parent body's effective inertia for the current mechanical-system step instead of reflecting stale acceleration as a load.

- [x] Review torque sign conventions for linear/rotational constraints.
  `torque_from_linear_force(force, dx_dtheta)` is mathematically reasonable, but the project needs one documented convention for piston force direction, positive crank rotation, and gas force sign.

- [x] Handle wrapped angular displacement correctly.
  Dependent rotational bodies now compute shortest signed wrapped displacement across wrap boundaries.

- [x] Revisit explicit integrators for stiff spring/damper links.
  Added an explicit spring stability timestep helper and tests so stiff spring/damper links can be checked instead of silently treated as unconditionally stable.

## Tests / Documentation

- [x] Fix stale integration-step test naming and expectations.
  Removed the old cylinder integration tests with the old thermodynamic model.

- [x] Add conservation tests for 0D chamber behavior.
  Cover closed adiabatic compression/expansion, open chamber filling/blowdown, and consistency between integrated pressure work and reported work.

- [x] Document which model path is authoritative.
  `AGENTS.md` now points new 0D gas work at `src/physics/chamber.rs`, `src/physics/flow.rs`, and `src/physics/gas.rs`.

- [ ] Model the GN250 four-valve DOHC head explicitly instead of collapsing it to one intake boundary and one exhaust boundary.
  The current GN250 fixture records `2 x 26 mm` intake valves and `2 x 22 mm` exhaust valves, but the runner still aggregates them into one intake valve event and one exhaust valve event.
