# Engine Simulator Codebase Inspection Report

**Date:** 2026-06-17
**All 134 tests pass. No compilation errors.**

---

## Status update — 2026-06-20 (branch `inspection-fixes`)

The report was triaged against the current code (it has since advanced past the
config split and `b476674` fueling fix). Outcomes:

**Already fixed since the report (stale — verified, no action needed):**
- **B1** — valve `discharge_coefficient` is now applied to the valve effective
  area (`single_cylinder.rs`) and the plenum throttle uses `1.0`; a test covers it.
- **B2 / I5** — indicated torque and the reported `cylinder_pressure_pa` both use
  `end_pressure_pa` now (same instant).
- **B3** — `air_flow_g_per_s` no longer clamps with `.max(0.0)`; a backflow-sign
  test exists.
- **B4** — lean (lambda 1.3, 3000 rpm) indicated torque is now positive with the
  current fixture calibration; the regression baseline was refreshed.
- **O2** — `SimulationDefinition` was removed; `timestep`, `redline_cut_time` and
  `max_added_inertia` now live in `EngineHandlingDefinition` (correctly categorised).

**Implemented on this branch (safe, output-neutral cleanups):**
- **O3 / S2** — removed the dead `pipe_cell_to_chamber_flux`; its one test now
  uses the orifice variant at full area.
- **O5 / S1** — derived `Default` for `PendingFrameAccumulator`; the four verbose
  initializers collapse to `::default()`.
- **O6 / S3** — single source of truth `IntakeExhaustDefinition::effective_idle_throttle_area_m2()`,
  used by both `single_cylinder` and `telemetry`.
- **O7** — documented `maximum_combustible_air_fuel_ratio` (lean limit phi ≈ 0.563).
- **O8 / S4** — replaced the `#[path = "physics/…"]` aliases with a real
  `pub mod physics { … }`; all references now use `crate::physics::*`.

**Deferred (change simulation outputs and/or need their own calibrated PR):**
- **B5** (throttle `minimum_area_m2` side-channel) — current values are correct;
  only the API is non-obvious. API-clarity refactor, low priority.
- **B6** (combustion-event-ratio resolution) — already correct for a single
  cylinder (one combustion stroke per cycle → windowed average is the ratio);
  finer counting matters at Phase 4 multi-cylinder.
- **O1 / I6** (make `one_d_mesh_cells_per_meter` authoritative) — conflicts with
  the new design where users tune `number_of_cells` per runner in JSON; needs a
  design decision before overriding that.
- **O4 / S5** (fold `step_chamber_with_pipe_fluxes` into `chamber.rs`) — risks
  shifting numeric outputs / regression baselines; do as a separately verified refactor.
- **I1** (RK4 main integration), **I2** (wall heat transfer), **I3** (piston/rod
  inertia), **I4** (friction / FMEP) — physics features that change outputs and
  require re-baselining; each warrants its own PR (I4 is the highest-value next step).

---

## Bugs

### B1 — Valve discharge coefficient is dead data
`ValveDefinition.discharge_coefficient` and `ValveEvent.discharge_coefficient` are stored but never applied to actual valve mass flow. The `pipe_cell_to_chamber_orifice_flux` → `rusanov_flux` path does not accept a Cd parameter; the valve effective area already incorporates `max_effective_area_m2` but is not further multiplied by Cd. The field is therefore always silently ignored. Separately, `single_cylinder.rs:393` (mis)uses the *intake valve's* Cd as the plenum throttle's discharge coefficient, which is semantically incorrect.

### B2 — Reported cylinder pressure and indicated torque are from different instants
`indicated_torque_nm` is computed from `start_pressure_pa` but `cylinder_pressure_pa` in the step output is `end_pressure_pa`. A consumer correlating the two fields gets a physically inconsistent snapshot within the same step.

### B3 — `air_flow_g_per_s` silently discards backflow
`EngineInstantSample` uses `intake_mass_flow_kg_per_s.max(0.0)`, which makes backflow during valve overlap invisible. This corrupts cycle-integrated air flow calculations and volumetric efficiency estimates at low loads.

### B4 — `lean_mean_torque_nm` regression baseline is physically implausible
The regression baseline in `single_cylinder_vertical_slice.rs` records `lean_mean_torque_nm = -1.508 Nm` at 3000 rpm, lambda=1.3. A 250cc engine at WOT and 3000 rpm should not produce negative *indicated* torque at lambda=1.3 — this points to incorrect fixture calibration in the exhaust geometry, combustion efficiency values, or gas constant settings rather than a plausible physical result.

### B5 — Throttle passes `position: 0.0` to plenum, relying on `minimum_area_m2` side channel
`single_cylinder.rs:387-399` calls `step_throttle` with `throttle_position: 0.0` and bakes the full effective area into `minimum_area_m2`. The `ThrottlePlenumInput` path through `throttle::effective_area_m2(0.0, 0.0, 0.0, 1.0)` then returns 0.0 for the normal throttle contribution and relies entirely on `minimum_area_m2`. This is non-obvious and means the throttle model in the plenum step is not exercising the normal throttle curve.

### B6 — `combustion_event_ratio` has coarse per-cycle resolution
`CycleAccumulator::complete_cycle` records `combustion_event_ratio` as either 0.0 or 1.0 per cycle. The design doc requires it to be "combustion events divided by possible combustion strokes over the tracked cycle window," which would require counting actual spark events vs. fired events, not just whether any combustion occurred in a cycle.

---

## Other Issues

### O1 — `SimulationProfile.one_d_mesh_cells_per_meter` is non-functional
The field is defined, validated, and referenced in tests, but nothing in the engine loop actually reads it to configure pipe geometry. Pipe cell counts and lengths come exclusively from the JSON fixture. This means the real-time vs. render profile distinction has no effect on 1D pipe resolution.

### O2 — `simulation.redline_cut_time_seconds` and `max_added_inertia_kg_m2` are GUI-only
Both fields live in `SimulationDefinition` (engine JSON) but are only consumed by `engine_gui.rs`. The library's `SingleCylinderEngine` does not enforce or use them, making them miscategorised and a confusing API for non-GUI callers.

### O3 — `pipe_cell_to_chamber_flux` is only used in a test
The function in `pipe.rs` is called exclusively in test code. It duplicates `pipe_cell_to_chamber_orifice_flux` without the valve-area scaling. It should be removed or integrated into the test helper.

### O4 — `step_chamber_with_pipe_fluxes` duplicates chamber energy integration logic
The custom function in `single_cylinder.rs` reimplements internal-energy-based chamber stepping that partly duplicates the `chamber.rs` module. The `chamber.rs` `step_rk4` and `step_euler` functions exist specifically to be the authoritative integration path, yet the main engine loop bypasses them.

### O5 — `PendingFrameAccumulator` zero-initialization is repeated verbatim four times
`TelemetryAggregator::new` initialises two `PendingFrameAccumulator` structs by listing every field explicitly; `publish_ready` resets them the same way twice more. There is no `Default` implementation or constructor for `PendingFrameAccumulator`, leading to ~80 lines of identical field lists that are fragile when adding or removing fields.

### O6 — Idle throttle area fallback duplicated
`idle_throttle_maximum_area_m2()` in `single_cylinder.rs` and `EngineControls::throttle_effective_area_fraction()` in `telemetry.rs` both independently implement the same fallback: `throttle_maximum_area_m2 * 0.10`. These will diverge on modification.

### O7 — `mixture_limits.maximum_combustible_air_fuel_ratio` magic value
The default value `26.1101243339254` is not explained or commented anywhere. It corresponds to phi=0.563, which could be a documented parameter rather than an unexplained constant.

### O8 — Module path aliases bypass standard module hierarchy
`lib.rs` uses `#[path = "physics/chamber.rs"]` instead of a proper `pub mod physics { pub mod chamber; ... }`. This unconventional pattern prevents IDE navigation and `use` imports from reflecting the physical directory layout.

---

## Progress

The project has advanced significantly — through Phase 5 — in a short development span. All core Phase 1–3 items are complete and regression-tested. Phase 5 (1D intake/exhaust) is structurally in place.

**Complete:**
- Phase 1: Core interfaces, profiles, sign conventions, deterministic scheduler
- Phase 1A: Unit/integration/convergence/regression test infrastructure
- Phase 2: Single-cylinder 0D loop, slider-crank geometry, valve events, JSON fixture
- Phase 2A: GUI shell, telemetry layer, all required displays and controls
- Phase 3: Species-aware combustion, Wiebe burn, oxygen/fuel/products tracking

**Partially complete (Phase 5):**
- 1D pipe with Rusanov flux, CFL stability, species transport ✅
- Throttle + intake plenum + intake runner coupling ✅
- Exhaust runner with open pressure boundary ✅
- Multi-cylinder exhaust collector/main pipe ❌
- Pressure wave sound generation ❌

**Not started:**
- Phase 4: Multi-cylinder, firing order, shared crank
- Phase 6: Friction/pumping losses, brake torque, calibration
- Phase 7: Audio signal extraction, render-mode sweep outputs

**Open items from Phase 1A:**
- Motored cycle net indicated work near zero (hardest calibration check — still unchecked)
- Real dyno/pressure-trace data fixtures
- GN250 four-valve DOHC head explicit modelling (still uses one aggregate intake, one exhaust boundary)

---

## Improvements

### I1 — Use RK4 for main chamber integration
`chamber.rs` has `step_rk4_with_stage_inputs` that evaluates moving-boundary inputs at each RK4 stage. The main engine loop uses explicit Euler via `step_chamber_with_pipe_fluxes`. Switching to RK4 would reduce integration error in the pressure trace and combustion timing at the cost of ~4× the computation per step, but would remove the need for ad-hoc sub-stepping.

### I2 — Add wall heat transfer (Woschni or simplified)
`ChamberBoundary.heat_rate_w` is always 0.0 in the main loop. Peak temperatures above 3000 K occur during combustion, which is physically unrealistically high. Even a simple fraction-of-indicated-work heat loss per cycle would be more realistic than zero.

### I3 — Piston and connecting-rod mass
The crank integrator does not include piston or rod inertia. `mechanical_linking.rs` already has the `DependentLinearBody` pattern for coupling linear inertia to the crank, but it is unused in `SingleCylinderEngine`. Piston mass for the GN250 (~0.15 kg) is significant at high RPM.

### I4 — Friction model (Phase 6 prerequisite)
There is no friction torque anywhere. Even a simple Chen-Flynn FMEP model (constant + speed-dependent + pressure-dependent terms) would separate indicated from brake torque and make the simulation physically meaningful as a dyno tool.

### I5 — Report torque and pressure from the same instant
Compute `end_pressure_pa` before computing torque, or report `start_pressure_pa`. Using different crank angles for the two values is confusing in traces and could mask subtle bugs.

### I6 — Activate `one_d_mesh_cells_per_meter` in pipe construction
When a `SimulationProfile` is applied to an engine run, pipe cell counts should be derived from `one_d_mesh_cells_per_meter` and each runner's physical length rather than from a hardcoded JSON cell count. This would make render-mode refinement automatic.

---

## Next Steps

1. **Fix the lean torque sign (B4)**: Audit the GN250 exhaust geometry (26 cells × 0.075 m = 1.95 m runner) and heat loss fraction. A quick diagnostic is to run at lambda=1.3 with combustion disabled and check motored torque — if it is also abnormally negative the issue is in flow losses, not combustion.

2. **Fix valve Cd (B1)**: Apply `discharge_coefficient` inside `pipe_cell_to_chamber_orifice_flux` or fold it into `max_effective_area_m2` explicitly in the JSON and remove the field from `ValveEvent`.

3. **Add a minimal friction model (I4)**: Even a constant FMEP term (e.g. 100 kPa) would make indicated-to-brake conversion meaningful for Phase 6.

4. **Make `one_d_mesh_cells_per_meter` functional (I6)**: This is a one-day change that would unlock automatic convergence testing between real-time and render profiles.

5. **Address dead data (O1, O2)**: Either wire `one_d_mesh_cells_per_meter` to pipe construction, or remove it from the profile struct to avoid misleading callers.

6. **Phase 4 (multi-cylinder) before exhaust collector**: The exhaust collector (Phase 5 remainder) requires multi-cylinder exhaust primaries, so Phase 4 should be tackled first to unblock the full exhaust flow path.

7. **Calibrate the GN250 fixture**: Add at least one known analytical reference — peak motored pressure at known compression ratio and intake conditions can be computed from isentropic compression to validate geometry and gas properties before adding combustion complexity.

---

## Simplifications

### S1 — Derive `Default` for `PendingFrameAccumulator`
All fields are numeric zeros or `ScalarFrameAccumulator::default()`. Deriving or implementing `Default` collapses four large manual initializations into `PendingFrameAccumulator::default()`.

### S2 — Remove or integrate `pipe_cell_to_chamber_flux`
The function is only called in one test. The orifice variant (`pipe_cell_to_chamber_orifice_flux`) is the one used in production. Removing the non-orifice variant reduces surface area.

### S3 — Unify idle throttle area fallback
A single function `idle_throttle_maximum_area_m2(definition)` should be the single source of truth, called from both `single_cylinder.rs` and `telemetry.rs`, instead of duplicated inline.

### S4 — Standardise module layout with `mod physics`
Replace the `#[path = "physics/chamber.rs"]` aliases in `lib.rs` with `pub mod physics { pub mod chamber; ... }`. This is a purely mechanical change that makes the crate structure legible to tooling.

### S5 — Consolidate `step_chamber_with_pipe_fluxes` with `chamber.rs`
The energy accounting in the custom per-step function could be replaced with a call to `chamber::state_from_internal_energy` and explicit species updates, removing a ~60-line bespoke integration path in favour of the module designated as authoritative by AGENTS.md.

---

## Conclusion

The project is well-structured, comprehensively tested, and has advanced further than its roadmap phases suggest. All 134 tests pass cleanly. The core simulation loop is correct enough to produce physically meaningful pressure traces, combustion events, and RPM responses, and the test suite protects this behaviour.

The most significant gap is the absence of a friction model and the fact that one validated operating point (lambda=1.3, 3000 rpm) produces negative indicated torque in the regression baseline, which suggests the GN250 fixture needs calibration before Phase 6 can produce trustworthy brake power figures. The valve discharge coefficient bug (B1) affects effective flow area silently and should be the first structural fix. Several code quality issues — duplicated initializers, dead fields, the non-functional profile parameter — are low-risk but worth tidying before the codebase grows further. The physics fundamentals (conservation, Wiebe burn, species tracking, 1D Rusanov pipe) are sound.
