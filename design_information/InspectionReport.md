# Engine Simulator Codebase Inspection Report

**Original date:** 2026-06-17
**Status reviewed:** 2026-06-20 — all tests pass (169 lib + integration/contract suites), `clippy -D warnings` clean.

**2026-07-04 update:** since the last review, `src/multi_cylinder.rs` (Phase 4: multi-cylinder firing order, crank-phase scheduling, torque aggregation) and Phase 7 audio output (real-time GUI audio in `src/bin/engine_gui.rs` plus the offline `src/bin/render_audio.rs` renderer) have landed, along with the engine/handling JSON split (`src/engine_handling.rs`), the tuning GUI (`src/bin/tuning_gui.rs`, `src/tuning.rs`), and shapeable lift-based valve curves (`src/valve.rs`). The "Not started" list and Next Steps below predate this work and should be read as historical context, not current status; see `design_information/TODO.md` for the current roadmap state.

**2026-07-04 repository update:** the active 1D pipe solver is now the sibling `../onedpipes` crate, referenced through Cargo's path dependency mechanism. The earlier integrated project state from `Previous Attempts/enginesim` is preserved as Git branch `depricated_1d` for future reference.

> ✅ marks items completed (either already fixed before this review, or
> implemented on the `inspection-fixes` branch). Items that still have a
> description are open/deferred, with a short note on why where relevant.

---

## Bugs

### B1 — Valve discharge coefficient is dead data
✅

### B2 — Reported cylinder pressure and indicated torque are from different instants
✅

### B3 — `air_flow_g_per_s` silently discards backflow
✅

### B4 — `lean_mean_torque_nm` regression baseline is physically implausible
✅ (lean λ=1.3 @ 3000 rpm now yields positive indicated torque with the current fixture; baseline refreshed.)

### B5 — Throttle passes `position: 0.0` to plenum, relying on `minimum_area_m2` side channel
`single_cylinder.rs` calls `step_throttle` with `throttle_position: 0.0` and bakes the full effective area into `minimum_area_m2`. This is non-obvious and means the throttle model in the plenum step is not exercising the normal throttle curve.
*Deferred: the computed area is correct; only the API is unclear. Clarity-only refactor.*

### B6 — `combustion_event_ratio` has coarse per-cycle resolution
`CycleAccumulator::complete_cycle` records `combustion_event_ratio` as either 0.0 or 1.0 per cycle.
*Deferred: for a single cylinder there is exactly one combustion stroke per cycle, so the windowed average already equals events/possible-strokes. Finer counting matters at Phase 4 (multi-cylinder).*

---

## Other Issues

### O1 — `SimulationProfile.one_d_mesh_cells_per_meter` is non-functional
The field is defined, validated, and referenced in tests, but nothing in the engine loop reads it to configure pipe geometry. Pipe cell counts/lengths come from the JSON fixture.
*Deferred: making mesh density authoritative would override the per-runner `number_of_cells` the tuning GUI now lets users set/save. Needs a design decision first (see I6).*

### O2 — `simulation.redline_cut_time_seconds` and `max_added_inertia_kg_m2` are GUI-only
✅ (`SimulationDefinition` removed; these now live in `EngineHandlingDefinition`, correctly separated from the engine JSON.)

### O3 — `pipe_cell_to_chamber_flux` is only used in a test
✅

### O4 — `step_chamber_with_pipe_fluxes` duplicates chamber energy integration logic
The function in `single_cylinder.rs` reimplements internal-energy-based chamber stepping that partly duplicates `chamber.rs`.
*Partly addressed: the energy integration now calls `chamber::integrate_internal_energy_rk4` (see I1). The species/flux bookkeeping remains bespoke; full consolidation (S5) is deferred to avoid shifting outputs further.*

### O5 — `PendingFrameAccumulator` zero-initialization is repeated verbatim four times
✅

### O6 — Idle throttle area fallback duplicated
✅

### O7 — `mixture_limits.maximum_combustible_air_fuel_ratio` magic value
✅ (documented: lean flammability limit at φ ≈ 0.563.)

### O8 — Module path aliases bypass standard module hierarchy
✅ (replaced `#[path]` aliases with a real `pub mod physics { … }`.)

---

## Progress

The project has advanced through Phase 5. All core Phase 1–3 items are complete
and regression-tested. Phase 5 (1D intake/exhaust) is structurally in place.

**Complete:**
- Phase 1: Core interfaces, profiles, sign conventions, deterministic scheduler
- Phase 1A: Unit/integration/convergence/regression test infrastructure
- Phase 2: Single-cylinder 0D loop, slider-crank geometry, valve events, JSON fixture
- Phase 2A: GUI shell + a dedicated tuning GUI, telemetry layer, all required displays/controls
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
- GN250 four-valve DOHC head explicit modelling (still one aggregate intake, one exhaust boundary)

---

## Improvements

### I1 — Use RK4 for main chamber integration
✅ (chamber internal energy is now integrated with `chamber::integrate_internal_energy_rk4`, sampling the moving cylinder volume at each RK4 stage instead of the previous start-angle Euler piston-work term.)

### I2 — Add wall heat transfer (Woschni or simplified)
`ChamberBoundary.heat_rate_w` is always 0.0 in the main loop (combustion already applies `heat_loss_fraction`, but there is no compression/expansion wall loss). A simple model would lower unrealistically high peak temperatures.
*Deferred: physics feature; changes outputs and needs calibration.*

### I3 — Piston and connecting-rod mass
The crank integrator does not include piston or rod inertia, though `mechanical_linking.rs` already has the `DependentLinearBody` pattern. Significant at high RPM.
*Deferred: physics feature; changes high-rpm dynamics.*

### I4 — Friction model (Phase 6 prerequisite)
No friction torque anywhere. A Chen-Flynn FMEP model would separate indicated from brake torque.
*Deferred: blocked on real friction/dyno measurements before calibrating the model.*

### I5 — Report torque and pressure from the same instant
✅ (same as B2.)

### I6 — Activate `one_d_mesh_cells_per_meter` in pipe construction
Derive pipe cell counts from `one_d_mesh_cells_per_meter` × runner length so render-mode refinement is automatic.
*Deferred: conflicts with user-tuned per-runner `number_of_cells` (see O1).*

---

## Next Steps

1. **Add a minimal friction model (I4)** once measurements are available — even a
   constant FMEP term makes indicated-to-brake conversion meaningful for Phase 6.
2. **Decide the mesh-density question (O1/I6)**: keep JSON `number_of_cells`
   authoritative (current, user-tunable) or derive from profile mesh density.
3. **Phase 4 (multi-cylinder) before exhaust collector**: the collector needs
   multi-cylinder primaries, so Phase 4 unblocks the full exhaust path.
4. **Calibrate the GN250 fixture**: add an analytical reference (e.g. peak motored
   pressure from isentropic compression) to validate geometry/gas properties.

---

## Simplifications

### S1 — Derive `Default` for `PendingFrameAccumulator`
✅

### S2 — Remove or integrate `pipe_cell_to_chamber_flux`
✅

### S3 — Unify idle throttle area fallback
✅ (`IntakeExhaustDefinition::effective_idle_throttle_area_m2()` is the single source of truth.)

### S4 — Standardise module layout with `mod physics`
✅

### S5 — Consolidate `step_chamber_with_pipe_fluxes` with `chamber.rs`
The energy accounting now calls `chamber::integrate_internal_energy_rk4`, but the
species/flux bookkeeping is still bespoke.
*Deferred: full consolidation risks shifting regression baselines; do as a separately verified refactor.*

---

## Conclusion

The project is well-structured, comprehensively tested, and has advanced past its
roadmap phases. All tests pass cleanly (169 lib + integration/contract suites).
The core loop produces physically meaningful pressure traces, combustion events
and RPM responses, and the suite protects this behaviour.

The most significant remaining gap is the absence of a friction model, which is
blocked on real measurements before brake-power figures can be trusted. The
previously flagged structural bugs (valve Cd, torque/pressure instant, air-flow
backflow sign, sim-handling field categorisation) and the dead-code/duplication
issues have been resolved; the chamber integration now uses RK4. The physics
fundamentals (conservation, Wiebe burn, species tracking, 1D Rusanov pipe) are sound.
