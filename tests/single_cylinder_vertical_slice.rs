use enginesim::engine_config::EngineDefinition;
use enginesim::profiles::SimulationProfile;
use enginesim::single_cylinder::{
    SingleCylinderEngine, SingleCylinderStepInputs, rpm_to_rad_per_s, run_fixed_speed_cycles,
    run_fixed_speed_cycles_with_profile, run_fixed_speed_torque_sweep,
};
use enginesim::validation::{Tolerance, compare_convergence};

fn gn250_definition() -> EngineDefinition {
    EngineDefinition::from_json_str(include_str!("../data/engines/gn250.json"))
        .expect("GN250 JSON should parse")
}

fn motored_definition() -> EngineDefinition {
    let mut definition = gn250_definition();
    definition.combustion.enabled = false;
    definition
}

fn assert_close(actual: f64, expected: f64, tolerance: f64) {
    let difference = (actual - expected).abs();
    assert!(
        difference <= tolerance,
        "expected {expected}, got {actual}; difference {difference} exceeded {tolerance}",
    );
}

#[test]
fn fixed_rpm_single_cylinder_run_is_repeatable() {
    let definition = gn250_definition();
    let first = run_fixed_speed_cycles(&definition, 3000.0, 2);
    let second = run_fixed_speed_cycles(&definition, 3000.0, 2);

    assert_eq!(first.steps, second.steps);
    assert_close(
        first.indicated_work_j,
        second.indicated_work_j,
        first.indicated_work_j.abs().max(1.0) * 1.0e-12,
    );
    assert_close(
        first.mean_indicated_torque_nm,
        second.mean_indicated_torque_nm,
        first.mean_indicated_torque_nm.abs().max(1.0) * 1.0e-12,
    );
}

#[test]
fn fired_cycle_adds_indicated_work_relative_to_motored_cycle() {
    let fired = run_fixed_speed_cycles(&gn250_definition(), 3000.0, 2);
    let motored = run_fixed_speed_cycles(&motored_definition(), 3000.0, 2);

    assert!(fired.indicated_work_j > motored.indicated_work_j);
    assert!(fired.mean_indicated_torque_nm > motored.mean_indicated_torque_nm);
}

#[test]
fn full_motored_cycle_returns_finite_outputs() {
    let summary = run_fixed_speed_cycles(&motored_definition(), 3000.0, 1);

    assert!(summary.indicated_work_j.is_finite());
    assert!(summary.mean_indicated_torque_nm.is_finite());
    assert!(summary.steps > 0);
}

#[test]
fn torque_sweep_outputs_plot_ready_power_values() {
    let points = run_fixed_speed_torque_sweep(&gn250_definition(), &[2500.0, 3500.0, 4500.0], 1);

    assert_eq!(points.len(), 3);
    for point in points {
        assert_close(
            point.indicated_power_kw,
            point.mean_indicated_torque_nm * rpm_to_rad_per_s(point.rpm) / 1000.0,
            point.indicated_power_kw.abs().max(1.0) * 1.0e-12,
        );
    }
}

#[test]
fn cycle_work_matches_mean_torque_accounting() {
    let summary = run_fixed_speed_cycles(&gn250_definition(), 3000.0, 1);
    let expected_work_j = summary.mean_indicated_torque_nm * summary.integrated_angle_rad;

    assert_close(
        summary.indicated_work_j,
        expected_work_j,
        summary.indicated_work_j.abs().max(1.0) * 1.0e-12,
    );
}

#[test]
fn fixed_speed_cycle_trims_last_step_to_requested_angle() {
    let mut profile = SimulationProfile::real_time();
    profile.timestep_seconds = 0.000073;
    let summary = run_fixed_speed_cycles_with_profile(&gn250_definition(), profile, 3123.0, 1);

    assert_close(
        summary.integrated_angle_rad,
        std::f64::consts::TAU * 2.0,
        1.0e-12,
    );
    assert_close(
        summary.elapsed_time_seconds,
        summary.integrated_angle_rad / rpm_to_rad_per_s(summary.rpm),
        1.0e-12,
    );
}

#[test]
fn profile_timestep_drives_direct_engine_step() {
    let mut profile = SimulationProfile::render();
    profile.timestep_seconds = 0.00002;
    let mut engine =
        SingleCylinderEngine::from_definition_with_profile(gn250_definition(), profile);
    let output = engine.step(SingleCylinderStepInputs {
        fixed_crank_speed_rad_per_s: Some(rpm_to_rad_per_s(3000.0)),
        ..SingleCylinderStepInputs::default()
    });

    assert_close(
        output.elapsed_time_seconds,
        profile.timestep_seconds,
        1.0e-12,
    );
}

#[test]
fn timestep_refinement_keeps_mean_torque_in_same_operating_region() {
    let coarse = gn250_definition();
    let mut coarse_profile = SimulationProfile::real_time();
    coarse_profile.timestep_seconds = coarse.simulation.timestep_seconds;
    let mut fine_profile = coarse_profile;
    fine_profile.timestep_seconds = coarse_profile.timestep_seconds * 0.5;

    let coarse_summary = run_fixed_speed_cycles_with_profile(&coarse, coarse_profile, 3000.0, 1);
    let fine_summary = run_fixed_speed_cycles_with_profile(&coarse, fine_profile, 3000.0, 1);
    let check = compare_convergence(
        coarse_summary.mean_indicated_torque_nm,
        fine_summary.mean_indicated_torque_nm,
        Tolerance::combined(2.0, 0.35),
    );

    assert!(
        check.passed,
        "mean torque refinement check failed: {check:?}"
    );
}

#[test]
fn dynamic_crank_step_responds_to_external_load_torque() {
    let mut engine = SingleCylinderEngine::from_definition(gn250_definition());
    let initial_speed = engine.crank_speed_rad_per_s();
    let output = engine.step(SingleCylinderStepInputs {
        external_load_torque_nm: 1000.0,
        fixed_crank_speed_rad_per_s: None,
        ..SingleCylinderStepInputs::default()
    });

    assert!(output.crank_speed_rad_per_s < initial_speed);
}
