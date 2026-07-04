//! Wall-clock timing harness for the simulation hot path.
//!
//! These are not correctness tests: they print timing to stderr and don't
//! assert thresholds, since wall-clock time is machine-dependent. They're
//! `#[ignore]`d so `cargo test` stays fast; run explicitly with:
//!
//!   cargo test --release --test performance_profile -- --ignored --nocapture
//!
//! Every case here runs a fixed, small number of engine steps rather than a
//! fixed number of *cycles*: a full cycle at low RPM (e.g. 1 RPM) needs over
//! a million steps at the real-time timestep, which would make this suite
//! take minutes instead of the sub-second budget it's meant to have.
use std::time::Instant;

use enginesim::engine_config::EngineDefinition;
use enginesim::profiles::SimulationProfile;
use enginesim::single_cylinder::{
    SingleCylinderEngine, SingleCylinderStepInputs, rpm_to_rad_per_s,
};

const TIMED_STEPS: usize = 500;

fn gn250_definition() -> EngineDefinition {
    EngineDefinition::from_json_str(include_str!("../data/engines/gn250.json"))
        .expect("GN250 JSON should parse")
}

fn timed_steps(
    label: &str,
    definition: &EngineDefinition,
    profile: SimulationProfile,
    rpm: f64,
    steps: usize,
    inputs: SingleCylinderStepInputs,
) {
    let mut engine =
        SingleCylinderEngine::from_definition_with_profile(definition.clone(), profile);
    let crank_speed_rad_per_s = rpm_to_rad_per_s(rpm);

    let start = Instant::now();
    for _ in 0..steps {
        engine.step(SingleCylinderStepInputs {
            fixed_crank_speed_rad_per_s: Some(crank_speed_rad_per_s),
            ..inputs
        });
    }
    let elapsed = start.elapsed();
    let per_step = elapsed / steps.max(1) as u32;
    eprintln!("{label}: rpm={rpm:>6} steps={steps} elapsed={elapsed:?} per_step={per_step:?}");
}

#[test]
#[ignore]
fn profile_one_cycle_full_simulation() {
    let definition = gn250_definition();
    let profile = SimulationProfile::real_time();
    // One cycle at 3000 rpm and the real-time timestep is ~400 steps.
    timed_steps(
        "1-cycle full sim",
        &definition,
        profile,
        3000.0,
        400,
        SingleCylinderStepInputs::default(),
    );
}

#[test]
#[ignore]
fn profile_five_cycles_full_simulation() {
    let definition = gn250_definition();
    let profile = SimulationProfile::real_time();
    timed_steps(
        "5-cycle full sim",
        &definition,
        profile,
        3000.0,
        2000,
        SingleCylinderStepInputs::default(),
    );
}

#[test]
#[ignore]
fn profile_mechanical_only_no_combustion_no_fueling() {
    // Approximates the "mechanical" cost floor: crank/valve kinematics and
    // pipe/gas-dynamics still run, but combustion heat release is skipped.
    let definition = gn250_definition();
    let profile = SimulationProfile::real_time();
    timed_steps(
        "mechanical+gas-dynamics (no combustion)",
        &definition,
        profile,
        3000.0,
        TIMED_STEPS,
        SingleCylinderStepInputs {
            spark_enabled: false,
            fuel_enabled: false,
            ..SingleCylinderStepInputs::default()
        },
    );
}

#[test]
#[ignore]
fn profile_combustion_overhead() {
    // Diffing this against profile_mechanical_only_no_combustion_no_fueling
    // isolates the combustion/fueling model's share of per-step cost.
    let definition = gn250_definition();
    let profile = SimulationProfile::real_time();
    timed_steps(
        "full sim (for combustion-overhead diff)",
        &definition,
        profile,
        3000.0,
        TIMED_STEPS,
        SingleCylinderStepInputs::default(),
    );
}

#[test]
#[ignore]
fn profile_rpm_and_forced_map_grid() {
    let definition = gn250_definition();
    let profile = SimulationProfile::real_time();
    let rpm_points = [1.0, 10.0, 100.0, 1000.0, 5000.0, 8000.0];
    let forced_map_points_pa = [10_000.0, 50_000.0, 100_000.0, 110_000.0, 200_000.0];

    for rpm in rpm_points {
        for forced_map_pressure_pa in forced_map_points_pa {
            timed_steps(
                "grid",
                &definition,
                profile,
                rpm,
                TIMED_STEPS,
                SingleCylinderStepInputs {
                    forced_map_pressure_pa: Some(forced_map_pressure_pa),
                    ..SingleCylinderStepInputs::default()
                },
            );
        }
    }
}
