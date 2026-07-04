//! Self-contained CPU profiler for the simulation hot path.
//!
//! Uses `pprof`'s signal-based sampling profiler instead of `cargo flamegraph`
//! (which shells out to `perf`) so it works in sandboxes without `perf`
//! installed or elevated `perf_event_paranoid` permissions.
//!
//! Run with:
//!   cargo run --profile profiling --example flamegraph_profile
//!
//! Writes `flamegraph.svg` to the current directory.
use std::fs::File;
use std::time::Instant;

use enginesim::engine_config::EngineDefinition;
use enginesim::profiles::SimulationProfile;
use enginesim::single_cylinder::{
    SingleCylinderStepInputs, run_fixed_speed_cycles_with_profile_and_inputs,
};

fn gn250_definition() -> EngineDefinition {
    EngineDefinition::from_json_str(include_str!("../data/engines/gn250.json"))
        .expect("GN250 JSON should parse")
}

fn main() {
    let definition = gn250_definition();
    let profile = SimulationProfile {
        timestep_seconds: 5.0e-6,
        ..SimulationProfile::real_time()
    };

    let guard = pprof::ProfilerGuardBuilder::default()
        .frequency(1000)
        .blocklist(&["libc", "libgcc", "pthread", "vdso"])
        .build()
        .expect("failed to start pprof profiler");

    let start = Instant::now();
    // A representative mixed workload: sweep a few RPM/MAP operating points so
    // the flamegraph reflects time spent across the throttle/pipe/combustion
    // paths rather than a single narrow condition.
    for rpm in [1000.0, 3000.0, 5000.0, 8000.0] {
        for forced_map_pressure_pa in [30_000.0, 101_325.0, 200_000.0] {
            run_fixed_speed_cycles_with_profile_and_inputs(
                &definition,
                profile,
                rpm,
                20,
                SingleCylinderStepInputs {
                    forced_map_pressure_pa: Some(forced_map_pressure_pa),
                    ..SingleCylinderStepInputs::default()
                },
            );
        }
    }
    let elapsed = start.elapsed();
    eprintln!("workload completed in {elapsed:?}");

    match guard.report().build() {
        Ok(report) => {
            let svg = File::create("flamegraph.svg").expect("failed to create flamegraph.svg");
            report
                .flamegraph(svg)
                .expect("failed to write flamegraph.svg");
            eprintln!("wrote flamegraph.svg");
        }
        Err(error) => eprintln!("failed to build pprof report: {error}"),
    }
}
