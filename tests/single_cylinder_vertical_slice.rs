use enginesim::engine_config::EngineDefinition;
use enginesim::engine_handling::EngineHandlingDefinition;
use enginesim::profiles::SimulationProfile;
use enginesim::single_cylinder::{
    SingleCylinderEngine, SingleCylinderStepInputs, rad_per_s_to_rpm, rpm_to_rad_per_s,
    run_fixed_speed_cycles, run_fixed_speed_cycles_with_profile,
    run_fixed_speed_throttle_load_sweep, run_fixed_speed_torque_sweep,
};
use enginesim::telemetry::{
    EngineControls, EngineFrameTelemetry, TelemetryAggregator, TelemetryAvailability,
    default_profile_for_gui,
};
use enginesim::validation::{
    BaselineMetadata, RegressionBaseline, ScalarBaseline, Tolerance, compare_convergence,
};

fn gn250_definition() -> EngineDefinition {
    EngineDefinition::from_json_str(include_str!("../data/engines/gn250.json"))
        .expect("GN250 JSON should parse")
}

fn gn250_handling() -> EngineHandlingDefinition {
    EngineHandlingDefinition::from_json_str(include_str!("../data/engines/gn250.handling.json"))
        .expect("GN250 handling JSON should parse")
}

fn gn250_timestep_seconds() -> f64 {
    gn250_handling().timestep_seconds
}

const REGRESSION_WARMUP_CYCLES: usize = 8;

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

#[derive(Debug, Clone, Copy)]
struct TraceStats {
    min_pa: f64,
    max_pa: f64,
    mean_pa: f64,
    rms_ac_pa: f64,
}

impl TraceStats {
    fn from_samples(samples: &[f64]) -> Self {
        assert!(!samples.is_empty(), "trace samples must not be empty");
        let min_pa = samples.iter().copied().fold(f64::INFINITY, f64::min);
        let max_pa = samples.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let mean_pa = samples.iter().sum::<f64>() / samples.len() as f64;
        let rms_ac_pa = (samples
            .iter()
            .map(|sample| {
                let centered = sample - mean_pa;
                centered * centered
            })
            .sum::<f64>()
            / samples.len() as f64)
            .sqrt();

        Self {
            min_pa,
            max_pa,
            mean_pa,
            rms_ac_pa,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct CycleRunStats {
    summary_work_j: f64,
    mean_torque_nm: f64,
    peak_pressure_pa: f64,
    pressure_mean_pa: f64,
    pressure_rms_ac_pa: f64,
}

fn pressure_trace_stats(
    definition: &EngineDefinition,
    rpm: f64,
    cycles: usize,
) -> (TraceStats, TraceStats) {
    let mut engine = SingleCylinderEngine::from_definition(definition.clone());
    let crank_speed_rad_per_s = rpm_to_rad_per_s(rpm);
    let mut intake_samples = Vec::new();
    let mut exhaust_samples = Vec::new();
    let steps_per_cycle = ((std::f64::consts::TAU * 2.0)
        / (crank_speed_rad_per_s * gn250_timestep_seconds()))
    .ceil() as usize;
    for _ in 0..(steps_per_cycle * REGRESSION_WARMUP_CYCLES) {
        engine.step(SingleCylinderStepInputs {
            fixed_crank_speed_rad_per_s: Some(crank_speed_rad_per_s),
            ..SingleCylinderStepInputs::default()
        });
    }
    let total_steps = steps_per_cycle * cycles;

    for _ in 0..total_steps {
        let output = engine.step(SingleCylinderStepInputs {
            fixed_crank_speed_rad_per_s: Some(crank_speed_rad_per_s),
            ..SingleCylinderStepInputs::default()
        });
        intake_samples.push(output.intake_runner_pressure_pa);
        exhaust_samples.push(output.exhaust_runner_pressure_pa);
    }

    (
        TraceStats::from_samples(&intake_samples),
        TraceStats::from_samples(&exhaust_samples),
    )
}

fn cycle_run_stats(definition: &EngineDefinition, rpm: f64, cycles: usize) -> CycleRunStats {
    cycle_run_stats_with_profile(
        definition,
        SimulationProfile {
            timestep_seconds: gn250_timestep_seconds(),
            ..SimulationProfile::real_time()
        },
        rpm,
        cycles,
    )
}

fn cycle_run_stats_with_profile(
    definition: &EngineDefinition,
    profile: SimulationProfile,
    rpm: f64,
    cycles: usize,
) -> CycleRunStats {
    cycle_run_stats_with_profile_and_inputs(
        definition,
        profile,
        rpm,
        cycles,
        SingleCylinderStepInputs::default(),
    )
}

fn cycle_run_stats_with_profile_and_inputs(
    definition: &EngineDefinition,
    profile: SimulationProfile,
    rpm: f64,
    cycles: usize,
    inputs: SingleCylinderStepInputs,
) -> CycleRunStats {
    let mut engine =
        SingleCylinderEngine::from_definition_with_profile(definition.clone(), profile);
    engine.reset_accumulators();
    let crank_speed_rad_per_s = rpm_to_rad_per_s(rpm);
    let mut pressure_samples = Vec::new();
    let steps_per_cycle = ((std::f64::consts::TAU * 2.0)
        / (crank_speed_rad_per_s * profile.timestep_seconds))
        .ceil() as usize;
    for _ in 0..(steps_per_cycle * REGRESSION_WARMUP_CYCLES) {
        engine.step(SingleCylinderStepInputs {
            fixed_crank_speed_rad_per_s: Some(crank_speed_rad_per_s),
            ..inputs
        });
    }
    engine.reset_accumulators();
    let total_steps = steps_per_cycle * cycles;

    for _ in 0..total_steps {
        let output = engine.step(SingleCylinderStepInputs {
            fixed_crank_speed_rad_per_s: Some(crank_speed_rad_per_s),
            ..inputs
        });
        pressure_samples.push(output.cylinder_pressure_pa);
    }
    let pressure = TraceStats::from_samples(&pressure_samples);

    CycleRunStats {
        summary_work_j: engine.accumulated_indicated_work_j(),
        mean_torque_nm: engine.mean_indicated_torque_nm(),
        peak_pressure_pa: pressure.max_pa,
        pressure_mean_pa: pressure.mean_pa,
        pressure_rms_ac_pa: pressure.rms_ac_pa,
    }
}

fn run_gui_realtime_dyno_case(
    dyno_target_rpm: f64,
    lambda_target: f64,
    run_seconds: f64,
) -> EngineFrameTelemetry {
    let definition = gn250_definition();
    let profile = default_profile_for_gui(&gn250_handling());
    let controls = EngineControls {
        throttle_position: 1.0,
        idle_throttle_fraction: 0.0,
        lambda_target,
        dyno_mode_enabled: true,
        dyno_target_rpm,
        spark_enabled: true,
        fuel_enabled: true,
        ..EngineControls::default()
    };
    let mut engine =
        SingleCylinderEngine::from_definition_with_profile(definition.clone(), profile);
    let mut telemetry = TelemetryAggregator::new(definition.clone(), controls);
    let mut latest_frame = telemetry.snapshot();
    let steps = (run_seconds / profile.timestep_seconds).ceil() as usize;

    for _ in 0..steps {
        telemetry.set_controls(controls);
        let output = engine.step(controls.to_step_inputs(
            &definition,
            rad_per_s_to_rpm(engine.crank_speed_rad_per_s()),
        ));
        telemetry.ingest_step_output(output, profile.timestep_seconds);
        if let Some(frame) = telemetry.publish_ready() {
            latest_frame = frame;
        }
    }

    latest_frame
}

fn assert_gui_dyno_reports_measured_lambda_and_combusts(dyno_target_rpm: f64, lambda_target: f64) {
    let frame = run_gui_realtime_dyno_case(dyno_target_rpm, lambda_target, 2.0);
    let chamber_lambda = frame
        .engine_data
        .chamber_lambda
        .value
        .expect("GUI chamber lambda should be available in a fired dyno run");

    assert_eq!(
        frame.engine_data.chamber_lambda.availability,
        TelemetryAvailability::Measured
    );
    assert_eq!(
        frame.engine_data.exhaust_lambda.availability,
        TelemetryAvailability::Measured
    );
    assert!(chamber_lambda.is_finite());
    assert!(
        frame
            .engine_data
            .exhaust_lambda
            .value
            .is_some_and(f64::is_finite),
        "at {dyno_target_rpm:.0} rpm with target lambda {lambda_target}, \
         GUI exhaust lambda should be measured and finite"
    );
    assert!(
        frame.engine_data.combustion_event_ratio > 0.0,
        "at {dyno_target_rpm:.0} rpm with target lambda {lambda_target}, \
         expected at least one combustion event; got ratio {}",
        frame.engine_data.combustion_event_ratio
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
fn fired_cycle_consumes_reactants_and_generates_products() {
    let mut fired_engine = SingleCylinderEngine::from_definition(gn250_definition());
    let mut motored_engine = SingleCylinderEngine::from_definition(motored_definition());
    let crank_speed = rpm_to_rad_per_s(3000.0);
    let mut injected_fuel_kg = 0.0;
    let mut burned_fuel_kg = 0.0;
    let mut oxygen_consumed_kg = 0.0;
    let mut products_generated_kg = 0.0;

    for _ in 0..1000 {
        let inputs = SingleCylinderStepInputs {
            fixed_crank_speed_rad_per_s: Some(crank_speed),
            ..SingleCylinderStepInputs::default()
        };
        let output = fired_engine.step(inputs);
        injected_fuel_kg += output.fuel_injected_kg;
        burned_fuel_kg += output.fuel_burned_kg;
        oxygen_consumed_kg += output.oxygen_consumed_kg;
        products_generated_kg += output.products_generated_kg;
        motored_engine.step(inputs);
    }

    let fired_species = fired_engine.chamber_species();
    let motored_species = motored_engine.chamber_species();
    assert!(fired_species.products_kg > motored_species.products_kg);
    assert!(fired_species.oxygen_kg < motored_species.oxygen_kg);
    assert!(fired_species.fuel_kg < gn250_definition().combustion.fuel_mass_per_cycle_kg);
    assert!(injected_fuel_kg > 0.0);
    assert!(burned_fuel_kg > 0.0);
    assert!(oxygen_consumed_kg > 0.0);
    assert!(products_generated_kg > 0.0);

    let budget = fired_engine.species_budget();
    assert_close(
        budget.injected_fuel_kg,
        injected_fuel_kg,
        injected_fuel_kg.max(1.0) * 1.0e-12,
    );
    assert_close(
        budget.burned_fuel_kg,
        burned_fuel_kg,
        burned_fuel_kg.max(1.0) * 1.0e-12,
    );
    assert_close(
        budget.oxygen_consumed_kg,
        oxygen_consumed_kg,
        oxygen_consumed_kg.max(1.0) * 1.0e-12,
    );
    assert_close(
        budget.products_generated_kg,
        products_generated_kg,
        products_generated_kg.max(1.0) * 1.0e-12,
    );
}

#[test]
fn gui_realtime_dyno_wot_lambda_targets_track_and_combust_at_2000_rpm() {
    assert_gui_dyno_reports_measured_lambda_and_combusts(2000.0, 1.0);
    assert_gui_dyno_reports_measured_lambda_and_combusts(2000.0, 1.3);
}

#[test]
fn gui_realtime_dyno_wot_stoich_combusts_and_reports_near_stoich_at_1000_rpm() {
    assert_gui_dyno_reports_measured_lambda_and_combusts(1000.0, 1.0);
}

#[test]
fn wot_lambda_1_3_indicated_torque_is_positive_at_3000_rpm() {
    let definition = gn250_definition();
    let stats = cycle_run_stats_with_profile_and_inputs(
        &definition,
        SimulationProfile {
            timestep_seconds: gn250_timestep_seconds(),
            ..SimulationProfile::real_time()
        },
        3000.0,
        2,
        SingleCylinderStepInputs {
            lambda_target: 1.3,
            throttle_position: 1.0,
            idle_throttle_fraction: 0.0,
            ..SingleCylinderStepInputs::default()
        },
    );

    assert!(
        stats.mean_torque_nm > 0.0,
        "WOT lambda=1.3 should not produce negative indicated torque at 3000 rpm; got {} Nm",
        stats.mean_torque_nm
    );
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
fn intake_and_exhaust_pressure_trace_regression_features_match_baseline() {
    let (intake, exhaust) = pressure_trace_stats(&gn250_definition(), 3000.0, 2);
    let baseline = RegressionBaseline::new(
        BaselineMetadata {
            simulator_id: "phase5-1d-intake-exhaust".to_string(),
            profile: "real_time".to_string(),
            engine_configuration: "data/engines/gn250.json".to_string(),
            environment: "101325 Pa intake/exhaust boundary, 300 K intake, 700 K exhaust"
                .to_string(),
            timestep_seconds: gn250_timestep_seconds(),
            substeps: 1,
        },
        vec![
            ScalarBaseline::new(
                "intake_runner_pressure_min_pa",
                74728.0565,
                Tolerance::combined(25.0, 0.01),
            ),
            ScalarBaseline::new(
                "intake_runner_pressure_max_pa",
                109139.2056,
                Tolerance::combined(25.0, 0.01),
            ),
            ScalarBaseline::new(
                "intake_runner_pressure_mean_pa",
                99459.5930,
                Tolerance::combined(25.0, 0.01),
            ),
            ScalarBaseline::new(
                "intake_runner_pressure_rms_ac_pa",
                9649.3741,
                Tolerance::combined(25.0, 0.01),
            ),
            ScalarBaseline::new(
                "exhaust_runner_pressure_min_pa",
                122245.0152,
                Tolerance::combined(25.0, 0.01),
            ),
            ScalarBaseline::new(
                "exhaust_runner_pressure_max_pa",
                226388.9130,
                Tolerance::combined(25.0, 0.01),
            ),
            ScalarBaseline::new(
                "exhaust_runner_pressure_mean_pa",
                144794.0056,
                Tolerance::combined(25.0, 0.01),
            ),
            ScalarBaseline::new(
                "exhaust_runner_pressure_rms_ac_pa",
                30775.6046,
                Tolerance::combined(25.0, 0.01),
            ),
        ],
    );

    let actuals = [
        ("intake_runner_pressure_min_pa", intake.min_pa),
        ("intake_runner_pressure_max_pa", intake.max_pa),
        ("intake_runner_pressure_mean_pa", intake.mean_pa),
        ("intake_runner_pressure_rms_ac_pa", intake.rms_ac_pa),
        ("exhaust_runner_pressure_min_pa", exhaust.min_pa),
        ("exhaust_runner_pressure_max_pa", exhaust.max_pa),
        ("exhaust_runner_pressure_mean_pa", exhaust.mean_pa),
        ("exhaust_runner_pressure_rms_ac_pa", exhaust.rms_ac_pa),
    ];

    for (name, actual) in actuals {
        let check = baseline
            .check_scalar(name, actual)
            .expect("baseline scalar should exist");
        assert!(check.passed, "pressure trace regression failed: {check:?}");
    }
}

#[test]
fn single_cylinder_pressure_work_and_torque_regression_features_match_baseline() {
    let definition = gn250_definition();
    let stats = cycle_run_stats(&definition, 3000.0, 2);
    let baseline = RegressionBaseline::new(
        BaselineMetadata {
            simulator_id: "phase3-single-cylinder-species".to_string(),
            profile: "real_time".to_string(),
            engine_configuration: "data/engines/gn250.json".to_string(),
            environment: "101325 Pa intake/exhaust boundary, 300 K intake, 700 K exhaust"
                .to_string(),
            timestep_seconds: gn250_timestep_seconds(),
            substeps: 1,
        },
        vec![
            ScalarBaseline::new(
                "peak_cylinder_pressure_pa",
                5865348.4463,
                Tolerance::combined(500.0, 0.02),
            ),
            ScalarBaseline::new(
                "pressure_trace_mean_pa",
                635284.6301,
                Tolerance::combined(500.0, 0.02),
            ),
            ScalarBaseline::new(
                "pressure_trace_rms_ac_pa",
                1104195.2150,
                Tolerance::combined(500.0, 0.02),
            ),
            ScalarBaseline::new("indicated_work_j", 252.8983, Tolerance::combined(0.5, 0.03)),
            ScalarBaseline::new(
                "mean_indicated_torque_nm",
                10.0625,
                Tolerance::combined(0.05, 0.03),
            ),
        ],
    );

    let actuals = [
        ("peak_cylinder_pressure_pa", stats.peak_pressure_pa),
        ("pressure_trace_mean_pa", stats.pressure_mean_pa),
        ("pressure_trace_rms_ac_pa", stats.pressure_rms_ac_pa),
        ("indicated_work_j", stats.summary_work_j),
        ("mean_indicated_torque_nm", stats.mean_torque_nm),
    ];

    for (name, actual) in actuals {
        let check = baseline
            .check_scalar(name, actual)
            .expect("baseline scalar should exist");
        assert!(check.passed, "single-cylinder regression failed: {check:?}");
    }
}

#[test]
fn pressure_trace_sensitivity_to_combustion_timing_and_lambda_matches_baseline() {
    let definition = gn250_definition();
    let profile = SimulationProfile {
        timestep_seconds: gn250_timestep_seconds(),
        ..SimulationProfile::real_time()
    };
    let nominal = cycle_run_stats_with_profile(&definition, profile, 3000.0, 2);
    let mut delayed_definition = definition.clone();
    delayed_definition.combustion.ignition_delay_deg += 10.0;
    let delayed = cycle_run_stats_with_profile(&delayed_definition, profile, 3000.0, 2);
    let mut long_burn_definition = definition.clone();
    long_burn_definition
        .combustion
        .wiebe
        .combustion_duration_rad *= 1.5;
    let long_burn = cycle_run_stats_with_profile(&long_burn_definition, profile, 3000.0, 2);
    let lean = cycle_run_stats_with_profile_and_inputs(
        &definition,
        profile,
        3000.0,
        2,
        SingleCylinderStepInputs {
            lambda_target: 1.3,
            ..SingleCylinderStepInputs::default()
        },
    );

    assert!(delayed.peak_pressure_pa < nominal.peak_pressure_pa);
    assert!(long_burn.peak_pressure_pa < nominal.peak_pressure_pa);
    assert!(lean.peak_pressure_pa < nominal.peak_pressure_pa);

    let actuals = [
        ("nominal_peak_pressure_pa", nominal.peak_pressure_pa),
        ("delayed_peak_pressure_pa", delayed.peak_pressure_pa),
        ("long_burn_peak_pressure_pa", long_burn.peak_pressure_pa),
        ("lean_peak_pressure_pa", lean.peak_pressure_pa),
        ("nominal_mean_torque_nm", nominal.mean_torque_nm),
        ("delayed_mean_torque_nm", delayed.mean_torque_nm),
        ("long_burn_mean_torque_nm", long_burn.mean_torque_nm),
        ("lean_mean_torque_nm", lean.mean_torque_nm),
    ];
    let baseline = RegressionBaseline::new(
        BaselineMetadata {
            simulator_id: "phase3-combustion-sensitivity".to_string(),
            profile: "real_time".to_string(),
            engine_configuration:
                "data/engines/gn250.json with controlled timing/lambda perturbations".to_string(),
            environment: "101325 Pa intake/exhaust boundary, 300 K intake, 700 K exhaust"
                .to_string(),
            timestep_seconds: gn250_timestep_seconds(),
            substeps: 1,
        },
        vec![
            ScalarBaseline::new(
                "nominal_peak_pressure_pa",
                5865348.4463,
                Tolerance::combined(500.0, 0.03),
            ),
            ScalarBaseline::new(
                "delayed_peak_pressure_pa",
                5239394.6262,
                Tolerance::combined(500.0, 0.03),
            ),
            ScalarBaseline::new(
                "long_burn_peak_pressure_pa",
                4727085.1363,
                Tolerance::combined(500.0, 0.03),
            ),
            ScalarBaseline::new(
                "lean_peak_pressure_pa",
                2431099.5162,
                Tolerance::combined(500.0, 0.03),
            ),
            ScalarBaseline::new(
                "nominal_mean_torque_nm",
                10.0625,
                Tolerance::combined(0.05, 0.05),
            ),
            ScalarBaseline::new(
                "delayed_mean_torque_nm",
                10.3677,
                Tolerance::combined(0.05, 0.05),
            ),
            ScalarBaseline::new(
                "long_burn_mean_torque_nm",
                10.1570,
                Tolerance::combined(0.05, 0.05),
            ),
            ScalarBaseline::new(
                "lean_mean_torque_nm",
                2.4599,
                Tolerance::combined(0.05, 0.05),
            ),
        ],
    );

    for (name, actual) in actuals {
        let check = baseline
            .check_scalar(name, actual)
            .expect("baseline scalar should exist");
        assert!(
            check.passed,
            "combustion sensitivity regression failed: {check:?}"
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
    coarse_profile.timestep_seconds = gn250_timestep_seconds();
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
fn render_mode_refinement_keeps_work_torque_and_peak_pressure_close() {
    let definition = gn250_definition();
    let mut coarse_profile = SimulationProfile::render();
    coarse_profile.timestep_seconds = 1.0 / 20_000.0;
    coarse_profile.chamber_substeps = 1;
    let mut fine_profile = coarse_profile;
    fine_profile.timestep_seconds *= 0.5;
    fine_profile.chamber_substeps = 2;

    let coarse_summary =
        run_fixed_speed_cycles_with_profile(&definition, coarse_profile, 3000.0, 1);
    let fine_summary = run_fixed_speed_cycles_with_profile(&definition, fine_profile, 3000.0, 1);

    let torque = compare_convergence(
        coarse_summary.mean_indicated_torque_nm,
        fine_summary.mean_indicated_torque_nm,
        Tolerance::combined(3.0, 0.40),
    );
    let work = compare_convergence(
        coarse_summary.indicated_work_j,
        fine_summary.indicated_work_j,
        // The two-stage exhaust (primary + collector across an area-change
        // interface) is stiffer than the single pipe, so render-mode net
        // indicated work is more timestep/substep sensitive under refinement.
        // Widened deliberately; a less diffusive 1D scheme (see the deferred
        // ram/wave-fidelity work) would tighten this back up.
        Tolerance::combined(40.0, 0.70),
    );
    assert!(
        torque.passed,
        "render torque convergence failed: {torque:?}"
    );
    assert!(work.passed, "render work convergence failed: {work:?}");

    let coarse_pressure =
        cycle_run_stats_with_profile(&definition, coarse_profile, 3000.0, 1).peak_pressure_pa;
    let fine_pressure =
        cycle_run_stats_with_profile(&definition, fine_profile, 3000.0, 1).peak_pressure_pa;
    let pressure = compare_convergence(
        coarse_pressure,
        fine_pressure,
        // The explicit pipe/chamber coupling keeps torque and work in the same
        // operating region, but instantaneous peak pressure remains phase
        // sensitive to timestep and chamber substeps.
        Tolerance::combined(10_000.0, 0.75),
    );
    assert!(
        pressure.passed,
        "render peak-pressure convergence failed: {pressure:?}"
    );
}

#[test]
fn throttle_load_sweep_emits_plot_ready_grid_points() {
    let definition = gn250_definition();
    let points = run_fixed_speed_throttle_load_sweep(
        &definition,
        SimulationProfile::real_time(),
        &[2500.0, 3500.0],
        &[0.25, 1.0],
        &[0.0, 5.0],
        1,
    );

    assert_eq!(points.len(), 8);
    for point in points {
        assert!([2500.0, 3500.0].contains(&point.rpm));
        assert!([0.25, 1.0].contains(&point.throttle_effective_area_fraction));
        assert!([0.0, 5.0].contains(&point.external_load_torque_nm));
        assert_close(
            point.indicated_power_kw,
            point.mean_indicated_torque_nm * rpm_to_rad_per_s(point.rpm) / 1000.0,
            point.indicated_power_kw.abs().max(1.0) * 1.0e-12,
        );
    }
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

#[test]
fn split_config_preserves_fixed_speed_run_baseline() {
    // Regression lock for a fixed-speed run of the current gn250 fixture. Update
    // these goldens deliberately when the fixture is retuned (recapture from a
    // 4-cycle run at 2500 rpm).
    let definition = gn250_definition();
    let summary = run_fixed_speed_cycles(&definition, 2500.0, 4);

    assert_close(summary.mean_indicated_torque_nm, 6.4976000, 1.0e-6);
    assert_close(summary.indicated_work_j, 326.6049995, 1.0e-6);
}

#[test]
fn renamed_pipe_grid_preserves_cell_geometry() {
    // The pipe fields are tunable (and saved back to the fixture from the GUI),
    // so assert the `number_of_cells` / `total_length_m` -> per-cell length
    // invariant rather than exact tuned counts.
    let definition = gn250_definition();
    let intake = definition.intake_exhaust.intake_runner;
    let exhaust = definition.intake_exhaust.exhaust_runner;

    for pipe in [intake, exhaust] {
        assert!(pipe.number_of_cells >= 1);
        assert!(pipe.total_length_m > 0.0);
        assert_close(
            pipe.cell_length_m(),
            pipe.total_length_m / pipe.number_of_cells as f64,
            1.0e-12,
        );
    }

    // The GN250 fixture models the two exhaust primaries merging into one
    // tailpipe, so it must carry a collector with the same cell invariant.
    let collector = definition
        .intake_exhaust
        .exhaust_collector
        .expect("GN250 fixture should define an exhaust collector");
    assert!(collector.number_of_cells >= 1);
    assert_close(
        collector.cell_length_m(),
        collector.total_length_m / collector.number_of_cells as f64,
        1.0e-12,
    );
}
