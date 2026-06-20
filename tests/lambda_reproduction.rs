use enginesim::engine_config::EngineDefinition;
use enginesim::profiles::SimulationProfile;
use enginesim::single_cylinder::{
    SingleCylinderEngine, SingleCylinderStepInputs, rpm_to_rad_per_s,
};

fn gn250_definition() -> EngineDefinition {
    EngineDefinition::from_json_str(include_str!("../data/engines/gn250.json"))
        .expect("GN250 JSON should parse")
}

struct Stats {
    min: f64,
    max: f64,
    last: f64,
    count_finite: usize,
    count_none: usize,
}

fn drive(rpm: f64, throttle: f64, lambda_target: f64) {
    let definition = gn250_definition();
    let profile: SimulationProfile = SimulationProfile::real_time();
    let mut engine =
        SingleCylinderEngine::from_definition_with_profile(definition.clone(), profile);
    engine.reset_accumulators();

    let crank_speed = rpm_to_rad_per_s(rpm);
    let dt = profile.timestep_seconds;

    let inputs = SingleCylinderStepInputs {
        fixed_crank_speed_rad_per_s: Some(crank_speed),
        throttle_position: throttle,
        lambda_target,
        ..Default::default()
    };

    // warm up for several cycles, then sample the last 2 cycles.
    let cycle_rad = 4.0 * std::f64::consts::PI;
    let warmup_cycles = 30.0;
    let sample_cycles = 4.0;
    let warmup_steps = (warmup_cycles * cycle_rad / (crank_speed * dt)) as usize;
    let sample_steps = (sample_cycles * cycle_rad / (crank_speed * dt)) as usize;

    for _ in 0..warmup_steps {
        engine.step(inputs);
    }

    let mut chamber = Stats {
        min: f64::INFINITY,
        max: f64::NEG_INFINITY,
        last: f64::NAN,
        count_finite: 0,
        count_none: 0,
    };
    let mut exhaust = Stats {
        min: f64::INFINITY,
        max: f64::NEG_INFINITY,
        last: f64::NAN,
        count_finite: 0,
        count_none: 0,
    };

    for _ in 0..sample_steps {
        let out = engine.step(inputs);
        match out.chamber_lambda {
            Some(v) => {
                chamber.min = chamber.min.min(v);
                chamber.max = chamber.max.max(v);
                chamber.last = v;
                chamber.count_finite += 1;
            }
            None => chamber.count_none += 1,
        }
        match out.exhaust_lambda {
            Some(v) => {
                exhaust.min = exhaust.min.min(v);
                exhaust.max = exhaust.max.max(v);
                exhaust.last = v;
                exhaust.count_finite += 1;
            }
            None => exhaust.count_none += 1,
        }
    }

    println!(
        "rpm={rpm} throttle={throttle} lambda_target={lambda_target}\n  chamber_lambda: min={:.3} max={:.3} last={:.3} finite={} none={}\n  exhaust_lambda: min={:.3} max={:.3} last={:.3} finite={} none={}",
        chamber.min,
        chamber.max,
        chamber.last,
        chamber.count_finite,
        chamber.count_none,
        exhaust.min,
        exhaust.max,
        exhaust.last,
        exhaust.count_finite,
        exhaust.count_none,
    );
}

#[test]
fn reproduce_lambda_measurement_blowup() {
    drive(2200.0, 1.0, 1.17);
    drive(2200.0, 0.25, 1.17);
    drive(1000.0, 1.0, 1.0);
}
