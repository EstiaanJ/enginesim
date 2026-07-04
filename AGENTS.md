# AGENTS.md

## Dev environment tips

- This repository is a Rust engine-simulation prototype. Keep internal units SI unless a design document explicitly says otherwise.
- Read `design_information/DesignDoc.md` before changing simulation architecture.
- Read `design_information/TODO.md` before changing physics, thermodynamics, chamber, flow, or mechanical-linking code.
- Read `design_information/Wiebe Burn Model.md` before changing combustion timing, heat release, misfire or partial-burn behavior.
- Follow the testing strategy in `design_information/DesignDoc.md` for unit, integration, convergence and regression tests.
- Treat `src/physics/chamber.rs`, `src/physics/flow.rs`, and `src/physics/gas.rs` as the preferred direction for new 0D gas work unless the task explicitly targets legacy code.
- Keep reusable engine math in focused modules such as `src/engine_geometry.rs`, `src/throttle.rs`, and `src/combustion.rs`.
- Keep GUI-facing aggregation in `src/telemetry.rs`. Do not let GUI widgets reach directly into raw simulation internals.
- Build one deterministic simulation core. Real-time mode and render mode should be different profiles of the same model graph, not separate simulator implementations.
- Prefer a single-threaded fixed-step scheduler until conservation, coupling and event ordering are correct. Add parallelism later only when it preserves deterministic results for the same inputs.
- Engine definitions are JSON-backed through serde. Keep engine data in `data/engines/` and keep all values SI in JSON files.
- Treat `data/engines/gn250.json` as an approximate Phase 2 fixture, not a calibrated GN250 model. Its rod length, valve timing, valve diameters/counts, lambda target and spark schedule are supplied GN250 values; effective flow areas, crank inertia interpretation and combustion energy settings are still approximate.
- Properties that define the physical engine belong in `data/engines/<name>.json` (`EngineDefinition`, `src/engine_config.rs`). Properties that define how the sim/GUI handles that engine (timestep, redline cut time, max added inertia, tuning set-point RPMs) belong in the sibling `data/engines/<name>.handling.json` (`EngineHandlingDefinition`, `src/engine_handling.rs`). Do not add sim-handling fields back into the engine JSON.
- 1D pipe geometry in engine JSON is defined as `total_length_m` and `number_of_cells`, not `cell_length`/`cell_count`. `cell_length_m()` is derived.
- The tuning GUI (`src/bin/tuning_gui.rs`, backed by `src/tuning.rs::TuningSession`) edits a draft engine/handling config separately from the committed one; committing ("Write Changes") rebuilds the sim rather than requiring a recompile.

## Testing instructions

- Run `cargo test` after code changes.
- For physics/math changes, add or update focused tests for conservation, units, signs, and limiting cases.
- Prefer tests that verify physical invariants over tests that only match the current implementation.
- When adding engine-loop behavior, include tests for deterministic repeatability and profile-independent model behavior where practical.
- When implementing a roadmap phase, include that phase's testing and validation items rather than leaving testing only to Phase 1.
- When fixing a physics bug, add a test that would have failed before the fix.
- Put coupled model behavior in integration tests once the relevant modules exist.
- Regression baselines must include model/profile settings, engine configuration, environmental inputs and explicit tolerances.
- If a baseline changes intentionally, document why the new output is more correct.
- Convergence or render-mode tests may be slower than unit tests, but they should be deterministic and clearly separated from fast tests.
- Documentation-only changes do not require a compile/test pass.
- GUI changes should pass `cargo test` and `cargo clippy -- -D warnings`. Use `cargo run --bin engine_gui` or `cargo run --bin tuning_gui` for manual inspection when a local display session is available.
- Use `cargo run --bin engine_tui -- --test <profile.json>` for headless scripted grid/sweep checks without a display.
- Use `cargo run --release --bin render_audio -- --rpm <rpm> --throttle <0-1> --out <file>.wav` to render an offline steady-state exhaust-pulse `.wav` for manual listening checks after acoustic-relevant changes (valve lift shape, collector geometry, pipe grid).

## Directory tree

- `.gitignore` - local/generated file ignore rules.
- `Cargo.toml` - Rust package manifest. Real-time audio deps: `cpal` (audio device I/O), `rtrb` (lock-free SPSC ring buffer). GUI: `eframe`, `egui_plot`. `onedpipes` is a sibling local path dependency (`../onedpipes`) with its own Git history.
- `Cargo.lock` - pinned Rust dependency lockfile.
- `src/lib.rs` - crate module wiring.
- `src/bin/engine_gui.rs` - eframe GUI shell over the library crate, including the real-time audio device stream.
- `src/bin/tuning_gui.rs` - eframe tuning GUI over `src/tuning.rs::TuningSession` for editing engine/handling JSON and 1D pipe geometry without recompiling.
- `src/bin/engine_tui.rs` - headless terminal runner: loads an engine + JSON test profile and prints time-averaged telemetry for scripted grid/sweep testing.
- `src/bin/render_audio.rs` - offline renderer that runs the sim at a fixed RPM/throttle, bins tailpipe exit pressure by cycle angle into one averaged steady-state pulse, and writes it to a looped `.wav` file.
- `src/combustion.rs` - AFR, mixture-efficiency, and Wiebe burn helpers.
- `src/engine_config.rs` - serde-backed engine JSON definitions (physical engine only; see `src/engine_handling.rs` for sim-handling fields). 1D pipe geometry fields are `total_length_m` and `number_of_cells`, with `cell_length_m()` derived.
- `src/engine_geometry.rs` - engine geometry and slider-crank helpers.
- `src/engine_handling.rs` - `EngineHandlingDefinition`: sim-handling settings (timestep, redline cut time, max added inertia, tuning set-point RPMs) that live in a sibling `<engine>.handling.json`, separate from the physical engine JSON.
- `src/engine_loader.rs` - loads paired engine + handling JSON from disk/bundled fixtures.
- `src/multi_cylinder.rs` - `MultiCylinderEngine`: firing order/crank-phase scheduling and torque aggregation across multiple `SingleCylinderEngine` instances sharing one crank.
- `src/profiles.rs` - real-time/render profile settings.
- `src/sign_conventions.rs` - force, torque and angle sign helpers.
- `src/simulation.rs` - deterministic step context, model trait and engine step order.
- `src/single_cylinder.rs` - single-cylinder 0D/1D-coupled engine runner (chamber, valves, intake/exhaust pipes, combustion, crank integration).
- `src/telemetry.rs` - GUI control mapping, telemetry aggregation and plot buffers.
- `src/test_harness.rs` - headless test-profile runner (grid/sweep execution and reporting) used by `engine_tui`.
- `src/throttle.rs` - throttle effective-area and throttle-flow helpers.
- `src/tuning.rs` - `TuningSession`: draft-vs-committed engine/handling config with explicit write/undo, used by the tuning GUI.
- `src/validation.rs` - tolerance, regression baseline and convergence helpers.
- `src/valve.rs` - crank-angle valve events, lift-to-effective-area curves (shapeable opening ramp/plateau/closing ramp) and effective-area helpers.
- `src/physics/chamber.rs` - generic 0D chamber/control-volume model.
- `src/physics/consts.rs` - physics constants.
- `src/physics/flow.rs` - compressible isentropic mass-flow helpers.
- `src/physics/gas.rs` - ideal-gas and heat-capacity helpers.
- `src/physics/linear_mass_integrator.rs` - simple linear mass integrator.
- `src/physics/mechanical_linking.rs` - force/torque coupling and mechanical constraints.
- `src/physics/pipe.rs` - 1D finite-volume pipe (Rusanov flux) including momentum-preserving collector/merge junctions for exhaust primaries.
- `src/physics/rotational_mass_integrator.rs` - simple rotational mass integrator.
- `data/engines/` - JSON engine + handling fixtures (`<name>.json` / `<name>.handling.json`) and source spec notes (`<name>.md`).
- `tests/` - external integration tests for chamber conservation, simulation contracts and the single-cylinder vertical slice.
- `design_information/` - design notes, issues and TODO.
