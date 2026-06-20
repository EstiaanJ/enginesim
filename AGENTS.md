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
- Do not paper over physics problems with clamps unless the clamp represents an intentional physical boundary and is documented.
- Engine definitions are JSON-backed through serde. Keep engine data in `data/engines/` and keep all values SI in JSON files.
- Treat `data/engines/gn250.json` as an approximate Phase 2 fixture, not a calibrated GN250 model. Its rod length, valve timing, valve diameters/counts, lambda target and spark schedule are supplied GN250 values; effective flow areas, crank inertia interpretation and combustion energy settings are still approximate.

## Testing instructions

- Run `cargo test` after code changes.
- For physics/math changes, add or update focused tests for conservation, units, signs, and limiting cases.
- Prefer tests that verify physical invariants over tests that only match the current implementation.
- When adding engine-loop behavior, include tests for deterministic repeatability and profile-independent model behavior where practical.
- When implementing a roadmap phase, include that phase's testing and validation items rather than leaving testing only to Phase 1.
- When fixing a physics bug, add a test that would have failed before the fix.
- Keep unit tests fast and colocated with the module under test.
- Put coupled model behavior in integration tests once the relevant modules exist.
- Regression baselines must include model/profile settings, engine configuration, environmental inputs and explicit tolerances.
- If a baseline changes intentionally, document why the new output is more correct.
- Convergence or render-mode tests may be slower than unit tests, but they should be deterministic and clearly separated from fast tests.
- Documentation-only changes do not require a compile/test pass.
- GUI changes should pass `cargo test` and `cargo clippy -- -D warnings`. Use `cargo run --bin engine_gui` for manual inspection when a local display session is available.

## Directory tree

- `.gitignore` - local/generated file ignore rules.
- `Cargo.toml` - Rust package manifest.
- `Cargo.lock` - pinned Rust dependency lockfile.
- `src/lib.rs` - crate module wiring.
- `src/bin/engine_gui.rs` - eframe GUI shell over the library crate.
- `src/combustion.rs` - AFR, mixture-efficiency, and Wiebe burn helpers.
- `src/engine_config.rs` - serde-backed engine JSON definitions.
- `src/engine_geometry.rs` - engine geometry and slider-crank helpers.
- `src/profiles.rs` - real-time/render profile settings.
- `src/sign_conventions.rs` - force, torque and angle sign helpers.
- `src/simulation.rs` - deterministic step context, model trait and engine step order.
- `src/single_cylinder.rs` - Phase 2 single-cylinder 0D vertical-slice runner.
- `src/telemetry.rs` - GUI control mapping, telemetry aggregation and plot buffers.
- `src/throttle.rs` - throttle effective-area and throttle-flow helpers.
- `src/validation.rs` - tolerance, regression baseline and convergence helpers.
- `src/valve.rs` - crank-angle valve events, lift curves and effective-area helpers.
- `src/physics/chamber.rs` - generic 0D chamber/control-volume model.
- `src/physics/consts.rs` - physics constants.
- `src/physics/flow.rs` - compressible isentropic mass-flow helpers.
- `src/physics/gas.rs` - ideal-gas and heat-capacity helpers.
- `src/physics/linear_mass_integrator.rs` - simple linear mass integrator.
- `src/physics/mechanical_linking.rs` - force/torque coupling and mechanical constraints.
- `src/physics/rotational_mass_integrator.rs` - simple rotational mass integrator.
- `data/engines/` - JSON engine fixtures and source spec notes.
- `tests/` - external integration tests for chamber conservation, simulation contracts and the single-cylinder vertical slice.
- `design_information/` - design notes, issues and TODO.
