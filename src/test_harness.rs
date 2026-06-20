//! JSON-configured headless test harness.
//!
//! Exercises an engine definition at a grid of steady operating points and/or a
//! ramped dyno sweep, then reports the time-averaged values of the same data the
//! real-time GUI displays. Driven from the `engine_tui` binary via a JSON test
//! profile so LLMs and humans can probe the model without the GUI.

use serde::Deserialize;

use crate::engine_config::EngineDefinition;
use crate::engine_handling::EngineHandlingDefinition;
use crate::profiles::SimulationProfile;
use crate::single_cylinder::{SingleCylinderEngine, SingleCylinderStepInputs, rpm_to_rad_per_s};
use crate::telemetry::{
    EngineControls, EngineInstantSample, TelemetryAvailability, default_profile_for_gui,
};

const DEFAULT_SETTLE_FRACTION: f64 = 0.5;
const DEFAULT_SWEEP_RPM_BIN: f64 = 250.0;

// ---------------------------------------------------------------------------
// JSON configuration
// ---------------------------------------------------------------------------

/// Which built-in simulation profile (timestep / resolution) the test uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum ProfileSelection {
    #[default]
    RealTime,
    Render,
}

impl ProfileSelection {
    pub fn to_profile(self, handling: &EngineHandlingDefinition) -> SimulationProfile {
        match self {
            ProfileSelection::RealTime => default_profile_for_gui(handling),
            ProfileSelection::Render => SimulationProfile::render(),
        }
    }
}

/// How a parameter axis spaces its points between `min` and `max`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum Spacing {
    #[default]
    Linear,
    /// Natural-log (geometric) spacing; requires positive bounds.
    Log,
}

/// A single swept parameter: `count` points from `min` to `max` inclusive.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
pub struct AxisSpec {
    pub count: usize,
    pub min: f64,
    pub max: f64,
    #[serde(default)]
    pub spacing: Spacing,
}

impl AxisSpec {
    /// Expand the axis into its concrete set points.
    pub fn values(&self) -> Vec<f64> {
        let count = self.count.max(1);
        if count == 1 {
            return vec![self.min];
        }

        let last = (count - 1) as f64;
        match self.spacing {
            Spacing::Linear => (0..count)
                .map(|index| self.min + (self.max - self.min) * (index as f64 / last))
                .collect(),
            Spacing::Log => {
                let low = self.min.max(f64::MIN_POSITIVE);
                let high = self.max.max(f64::MIN_POSITIVE);
                let (ln_low, ln_high) = (low.ln(), high.ln());
                (0..count)
                    .map(|index| (ln_low + (ln_high - ln_low) * (index as f64 / last)).exp())
                    .collect()
            }
        }
    }
}

/// Grid test: every combination of the load axis, RPM and target lambda.
#[derive(Debug, Clone, Deserialize)]
pub struct GridConfig {
    /// Throttle fraction axis (0..1). Mutually exclusive with `map_kpa`.
    #[serde(default)]
    pub throttle: Option<AxisSpec>,
    /// Manifold absolute pressure axis in kPa. Mutually exclusive with `throttle`.
    #[serde(default)]
    pub map_kpa: Option<AxisSpec>,
    pub rpm: AxisSpec,
    pub lambda: AxisSpec,
    /// Seconds spent at each operating point.
    pub dwell_seconds: f64,
    /// Fraction of the dwell spent settling before averaging begins.
    #[serde(default = "default_settle_fraction")]
    pub settle_fraction: f64,
}

/// Ramped dyno-sweep test, optionally averaged over several runs.
#[derive(Debug, Clone, Deserialize)]
pub struct SweepConfig {
    /// How many times each pull is run; per-RPM-bin data is averaged across runs.
    pub runs: usize,
    /// Wall-clock length of one RPM ramp.
    pub duration_seconds: f64,
    /// Number of throttle pulls, spread linearly across 0..1 (1 => WOT only).
    pub throttle_points: usize,
    /// Number of lambda pulls, spread across `lambda_min`..`lambda_max`.
    pub lambda_points: usize,
    pub rpm_min: f64,
    pub rpm_max: f64,
    #[serde(default = "default_sweep_lambda")]
    pub lambda_min: f64,
    #[serde(default = "default_sweep_lambda")]
    pub lambda_max: f64,
    /// RPM width of each reporting bin.
    #[serde(default = "default_sweep_rpm_bin")]
    pub rpm_bin: f64,
}

/// Top-level test profile loaded from JSON.
#[derive(Debug, Clone, Deserialize)]
pub struct TestProfile {
    #[serde(default)]
    pub profile: ProfileSelection,
    #[serde(default)]
    pub grid: Option<GridConfig>,
    #[serde(default)]
    pub sweep: Option<SweepConfig>,
}

impl TestProfile {
    pub fn from_json_str(json: &str) -> serde_json::Result<Self> {
        serde_json::from_str(json)
    }

    /// Reject configurations that cannot be run before any simulation starts.
    pub fn validate(&self) -> Result<(), String> {
        if self.grid.is_none() && self.sweep.is_none() {
            return Err("test profile defines neither a `grid` nor a `sweep`".to_string());
        }
        if let Some(grid) = &self.grid {
            match (&grid.throttle, &grid.map_kpa) {
                (Some(_), Some(_)) => {
                    return Err(
                        "grid must specify either `throttle` or `map_kpa`, not both".to_string()
                    );
                }
                (None, None) => {
                    return Err("grid must specify either `throttle` or `map_kpa`".to_string());
                }
                _ => {}
            }
            if grid.dwell_seconds <= 0.0 {
                return Err("grid `dwell_seconds` must be positive".to_string());
            }
        }
        if let Some(sweep) = &self.sweep {
            if sweep.runs == 0 {
                return Err("sweep `runs` must be at least 1".to_string());
            }
            if sweep.duration_seconds <= 0.0 {
                return Err("sweep `duration_seconds` must be positive".to_string());
            }
            if sweep.rpm_max <= sweep.rpm_min {
                return Err("sweep `rpm_max` must exceed `rpm_min`".to_string());
            }
            if sweep.rpm_bin <= 0.0 {
                return Err("sweep `rpm_bin` must be positive".to_string());
            }
        }
        Ok(())
    }
}

fn default_settle_fraction() -> f64 {
    DEFAULT_SETTLE_FRACTION
}

fn default_sweep_rpm_bin() -> f64 {
    DEFAULT_SWEEP_RPM_BIN
}

fn default_sweep_lambda() -> f64 {
    1.0
}

// ---------------------------------------------------------------------------
// Operating points and results
// ---------------------------------------------------------------------------

/// The load setpoint for an operating point: throttle position or target MAP.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Load {
    Throttle(f64),
    ManifoldPressureKpa(f64),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OperatingPoint {
    pub rpm: f64,
    pub load: Load,
    pub lambda_target: f64,
}

/// Time-averaged telemetry for one operating point (mirrors the GUI panels).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PointMetrics {
    pub rpm: f64,
    pub lambda_target: f64,
    pub throttle: f64,
    pub map_kpa: f64,
    pub torque_nm: f64,
    pub power_kw: f64,
    pub fuel_flow_mg_per_s: f64,
    pub air_flow_g_per_s: f64,
    pub bmep_kpa: f64,
    pub volumetric_efficiency_percent: f64,
    pub chamber_lambda: Option<f64>,
    pub exhaust_lambda: Option<f64>,
    pub peak_pressure_kpa_rel: f64,
    pub peak_temperature_c: f64,
    pub combustion_event_ratio: f64,
}

/// One averaged RPM bin within a sweep pull.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SweepBin {
    pub rpm: f64,
    pub torque_nm: f64,
    pub power_kw: f64,
    pub bmep_kpa: f64,
    pub volumetric_efficiency_percent: f64,
    pub chamber_lambda: Option<f64>,
}

/// One throttle/lambda pull of a sweep, binned by RPM.
#[derive(Debug, Clone, PartialEq)]
pub struct SweepPull {
    pub throttle: f64,
    pub lambda_target: f64,
    pub bins: Vec<SweepBin>,
}

// ---------------------------------------------------------------------------
// Running operating points
// ---------------------------------------------------------------------------

#[derive(Default)]
struct SampleAccumulator {
    count: f64,
    rpm: f64,
    throttle: f64,
    map_pa: f64,
    torque_nm: f64,
    power_kw: f64,
    fuel_flow_mg_per_s: f64,
    air_flow_g_per_s: f64,
    bmep_kpa: f64,
    volumetric_efficiency_percent: f64,
    chamber_lambda_sum: f64,
    chamber_lambda_count: f64,
    exhaust_lambda_sum: f64,
    exhaust_lambda_count: f64,
    peak_pressure_pa: f64,
    peak_temperature_k: f64,
    cycles: f64,
    combusting_cycles: f64,
}

impl SampleAccumulator {
    fn record(&mut self, sample: &EngineInstantSample, throttle: f64) {
        self.count += 1.0;
        self.rpm += sample.rpm;
        self.throttle += throttle;
        self.map_pa += sample.map_pa;
        self.torque_nm += sample.torque_nm;
        self.power_kw += sample.power_kw;
        self.fuel_flow_mg_per_s += sample.fuel_flow_mg_per_s;
        self.air_flow_g_per_s += sample.air_flow_g_per_s;
        self.bmep_kpa += sample.bmep_kpa;
        self.volumetric_efficiency_percent += sample.volumetric_efficiency_percent;
        record_scalar(
            sample.chamber_lambda,
            &mut self.chamber_lambda_sum,
            &mut self.chamber_lambda_count,
        );
        record_scalar(
            sample.exhaust_lambda,
            &mut self.exhaust_lambda_sum,
            &mut self.exhaust_lambda_count,
        );
        self.peak_pressure_pa = self.peak_pressure_pa.max(sample.pressure_pa);
        self.peak_temperature_k = self.peak_temperature_k.max(sample.temperature_k);
    }
}

fn record_scalar(scalar: crate::telemetry::TelemetryScalar, sum: &mut f64, count: &mut f64) {
    if scalar.availability == TelemetryAvailability::Measured
        && let Some(value) = scalar.value.filter(|value| value.is_finite())
    {
        *sum += value;
        *count += 1.0;
    }
}

fn point_controls(lambda_target: f64, throttle: f64) -> EngineControls {
    EngineControls {
        throttle_position: throttle,
        idle_throttle_fraction: 0.0,
        lambda_target,
        fuel_enabled: true,
        spark_enabled: true,
        ..EngineControls::default()
    }
}

fn step_inputs(
    crank_speed_rad_per_s: f64,
    throttle: f64,
    lambda_target: f64,
) -> SingleCylinderStepInputs {
    SingleCylinderStepInputs {
        fixed_crank_speed_rad_per_s: Some(crank_speed_rad_per_s),
        throttle_position: throttle.clamp(0.0, 1.0),
        idle_throttle_fraction: 0.0,
        lambda_target,
        spark_enabled: true,
        fuel_enabled: true,
        ..SingleCylinderStepInputs::default()
    }
}

/// Run a single steady operating point and return its averaged telemetry.
///
/// The engine is held at a fixed crank speed (dyno style). A throttle setpoint
/// runs at that throttle on the engine's normal intake supply. Because
/// throttling does not draw the modelled intake plenum below ambient, a MAP
/// setpoint is imposed instead by overriding the intake supply pressure and
/// running wide-open — the plenum then settles to the requested pressure.
pub fn run_operating_point(
    definition: &EngineDefinition,
    profile: SimulationProfile,
    point: OperatingPoint,
    dwell_seconds: f64,
    settle_fraction: f64,
) -> PointMetrics {
    let (effective_definition, throttle) = match point.load {
        Load::Throttle(value) => (definition.clone(), value.clamp(0.0, 1.0)),
        Load::ManifoldPressureKpa(target_kpa) => {
            let mut overridden = definition.clone();
            overridden.boundaries.intake_pressure_pa = (target_kpa * 1000.0).max(1.0);
            (overridden, 1.0)
        }
    };

    let mut engine =
        SingleCylinderEngine::from_definition_with_profile(effective_definition.clone(), profile);
    let timestep_seconds = profile.timestep_seconds;
    let crank_speed_rad_per_s = rpm_to_rad_per_s(point.rpm.max(1.0));
    let crankcase_pressure_pa = effective_definition.boundaries.crankcase_pressure_pa;

    let total_steps = ((dwell_seconds / timestep_seconds).round() as usize).max(1);
    let settle_steps = ((total_steps as f64) * settle_fraction.clamp(0.0, 0.95)).round() as usize;

    let controls = point_controls(point.lambda_target, throttle);
    let mut accumulator = SampleAccumulator::default();
    let mut previous_angle_rad: Option<f64> = None;
    let mut current_cycle_combusting = false;

    for step_index in 0..total_steps {
        let inputs = step_inputs(crank_speed_rad_per_s, throttle, point.lambda_target);
        let output = engine.step(inputs);
        let measuring = step_index >= settle_steps;

        let combusting = output.combustion_heat_added_j > 0.0;
        current_cycle_combusting |= combusting;
        let wrapped = previous_angle_rad.is_some_and(|previous| output.crank_angle_rad < previous);
        if wrapped {
            if measuring {
                accumulator.cycles += 1.0;
                if current_cycle_combusting {
                    accumulator.combusting_cycles += 1.0;
                }
            }
            current_cycle_combusting = false;
        }
        previous_angle_rad = Some(output.crank_angle_rad);

        if measuring {
            let sample =
                EngineInstantSample::from_step_output(output, controls, &effective_definition);
            accumulator.record(&sample, throttle);
        }
    }

    finalize_point(
        &accumulator,
        point,
        crankcase_pressure_pa,
        current_cycle_combusting,
    )
}

fn finalize_point(
    accumulator: &SampleAccumulator,
    point: OperatingPoint,
    crankcase_pressure_pa: f64,
    trailing_cycle_combusting: bool,
) -> PointMetrics {
    let count = accumulator.count.max(1.0);
    let combustion_event_ratio = if accumulator.cycles > 0.0 {
        accumulator.combusting_cycles / accumulator.cycles
    } else if trailing_cycle_combusting {
        1.0
    } else {
        0.0
    };

    PointMetrics {
        rpm: accumulator.rpm / count,
        lambda_target: point.lambda_target,
        throttle: accumulator.throttle / count,
        map_kpa: accumulator.map_pa / count / 1000.0,
        torque_nm: accumulator.torque_nm / count,
        power_kw: accumulator.power_kw / count,
        fuel_flow_mg_per_s: accumulator.fuel_flow_mg_per_s / count,
        air_flow_g_per_s: accumulator.air_flow_g_per_s / count,
        bmep_kpa: accumulator.bmep_kpa / count,
        volumetric_efficiency_percent: accumulator.volumetric_efficiency_percent / count,
        chamber_lambda: mean_scalar(
            accumulator.chamber_lambda_sum,
            accumulator.chamber_lambda_count,
        ),
        exhaust_lambda: mean_scalar(
            accumulator.exhaust_lambda_sum,
            accumulator.exhaust_lambda_count,
        ),
        peak_pressure_kpa_rel: (accumulator.peak_pressure_pa - crankcase_pressure_pa) / 1000.0,
        peak_temperature_c: accumulator.peak_temperature_k - 273.15,
        combustion_event_ratio,
    }
}

fn mean_scalar(sum: f64, count: f64) -> Option<f64> {
    (count > 0.0).then_some(sum / count)
}

/// Expand and run a grid test into one metrics row per operating point.
pub fn run_grid(
    definition: &EngineDefinition,
    profile: SimulationProfile,
    grid: &GridConfig,
) -> Vec<PointMetrics> {
    let loads: Vec<Load> = if let Some(throttle) = &grid.throttle {
        throttle
            .values()
            .into_iter()
            .map(|value| Load::Throttle(value.clamp(0.0, 1.0)))
            .collect()
    } else if let Some(map_kpa) = &grid.map_kpa {
        map_kpa
            .values()
            .into_iter()
            .map(Load::ManifoldPressureKpa)
            .collect()
    } else {
        Vec::new()
    };

    let rpms = grid.rpm.values();
    let lambdas = grid.lambda.values();

    let mut results = Vec::with_capacity(loads.len() * rpms.len() * lambdas.len());
    for &load in &loads {
        for &rpm in &rpms {
            for &lambda_target in &lambdas {
                let point = OperatingPoint {
                    rpm,
                    load,
                    lambda_target,
                };
                results.push(run_operating_point(
                    definition,
                    profile,
                    point,
                    grid.dwell_seconds,
                    grid.settle_fraction,
                ));
            }
        }
    }
    results
}

// ---------------------------------------------------------------------------
// Running sweeps
// ---------------------------------------------------------------------------

#[derive(Default, Clone, Copy)]
struct SweepBinAccumulator {
    count: f64,
    rpm: f64,
    torque_nm: f64,
    power_kw: f64,
    bmep_kpa: f64,
    volumetric_efficiency_percent: f64,
    chamber_lambda_sum: f64,
    chamber_lambda_count: f64,
}

fn linspace_points(count: usize, min: f64, max: f64, single: f64) -> Vec<f64> {
    match count {
        0 => Vec::new(),
        1 => vec![single],
        n => {
            let last = (n - 1) as f64;
            (0..n)
                .map(|index| min + (max - min) * (index as f64 / last))
                .collect()
        }
    }
}

/// Run a sweep, returning one binned pull per throttle/lambda combination.
pub fn run_sweep(
    definition: &EngineDefinition,
    profile: SimulationProfile,
    sweep: &SweepConfig,
) -> Vec<SweepPull> {
    // A single throttle pull conventionally means WOT; a single lambda means stoich.
    let throttles = linspace_points(sweep.throttle_points, 0.0, 1.0, 1.0);
    let lambdas = linspace_points(sweep.lambda_points, sweep.lambda_min, sweep.lambda_max, 1.0);

    let bin_count = (((sweep.rpm_max - sweep.rpm_min) / sweep.rpm_bin).ceil() as usize).max(1);
    let timestep_seconds = profile.timestep_seconds;
    let total_steps = ((sweep.duration_seconds / timestep_seconds).round() as usize).max(2);

    let mut pulls = Vec::with_capacity(throttles.len() * lambdas.len());
    for &lambda_target in &lambdas {
        for &throttle in &throttles {
            let mut bins = vec![SweepBinAccumulator::default(); bin_count];

            for _ in 0..sweep.runs {
                let mut engine =
                    SingleCylinderEngine::from_definition_with_profile(definition.clone(), profile);
                let controls = point_controls(lambda_target, throttle);

                for step_index in 0..total_steps {
                    let fraction = step_index as f64 / (total_steps - 1) as f64;
                    let rpm = sweep.rpm_min + (sweep.rpm_max - sweep.rpm_min) * fraction;
                    let crank_speed_rad_per_s = rpm_to_rad_per_s(rpm.max(1.0));
                    let inputs = step_inputs(crank_speed_rad_per_s, throttle, lambda_target);
                    let output = engine.step(inputs);
                    let sample =
                        EngineInstantSample::from_step_output(output, controls, definition);

                    let bin_index =
                        (((rpm - sweep.rpm_min) / sweep.rpm_bin) as usize).min(bin_count - 1);
                    let bin = &mut bins[bin_index];
                    bin.count += 1.0;
                    bin.rpm += sample.rpm;
                    bin.torque_nm += sample.torque_nm;
                    bin.power_kw += sample.power_kw;
                    bin.bmep_kpa += sample.bmep_kpa;
                    bin.volumetric_efficiency_percent += sample.volumetric_efficiency_percent;
                    record_scalar(
                        sample.chamber_lambda,
                        &mut bin.chamber_lambda_sum,
                        &mut bin.chamber_lambda_count,
                    );
                }
            }

            let bins = bins
                .into_iter()
                .filter(|bin| bin.count > 0.0)
                .map(|bin| SweepBin {
                    rpm: bin.rpm / bin.count,
                    torque_nm: bin.torque_nm / bin.count,
                    power_kw: bin.power_kw / bin.count,
                    bmep_kpa: bin.bmep_kpa / bin.count,
                    volumetric_efficiency_percent: bin.volumetric_efficiency_percent / bin.count,
                    chamber_lambda: mean_scalar(bin.chamber_lambda_sum, bin.chamber_lambda_count),
                })
                .collect();

            pulls.push(SweepPull {
                throttle,
                lambda_target,
                bins,
            });
        }
    }
    pulls
}

// ---------------------------------------------------------------------------
// Reporting
// ---------------------------------------------------------------------------

/// Run the whole test profile against a definition and render a text report.
pub fn run_and_report(definition: &EngineDefinition, test: &TestProfile) -> String {
    let profile = test
        .profile
        .to_profile(&EngineHandlingDefinition::default());
    let mut report = String::new();

    report.push_str(&format!(
        "Engine: {}\nProfile: {:?} (timestep {:.1} us, {} substeps)\n",
        definition.metadata.name,
        test.profile,
        profile.timestep_seconds * 1.0e6,
        profile.chamber_substeps,
    ));

    if let Some(grid) = &test.grid {
        let metrics = run_grid(definition, profile, grid);
        report.push('\n');
        report.push_str(&format_grid_report(&metrics));
    }

    if let Some(sweep) = &test.sweep {
        let pulls = run_sweep(definition, profile, sweep);
        report.push('\n');
        report.push_str(&format_sweep_report(&pulls));
    }

    report
}

fn format_optional(value: Option<f64>) -> String {
    value
        .map(|value| format!("{value:.2}"))
        .unwrap_or_else(|| "  -".to_string())
}

fn format_grid_report(metrics: &[PointMetrics]) -> String {
    let mut out = String::from("=== Grid operating points ===\n");
    let header = format!(
        "{:>7} {:>6} {:>5} {:>6} {:>8} {:>7} {:>8} {:>7} {:>7} {:>5} {:>7} {:>7} {:>8} {:>8} {:>5}",
        "rpm",
        "lam*",
        "thr",
        "MAP",
        "torque",
        "power",
        "fuel",
        "air",
        "bmep",
        "VE",
        "lam_ch",
        "lam_ex",
        "pk_pres",
        "pk_temp",
        "burn",
    );
    out.push_str(&header);
    out.push('\n');
    out.push_str(&format!(
        "{:>7} {:>6} {:>5} {:>6} {:>8} {:>7} {:>8} {:>7} {:>7} {:>5} {:>7} {:>7} {:>8} {:>8} {:>5}\n",
        "", "", "", "kPa", "Nm", "kW", "mg/s", "g/s", "kPa", "%", "", "", "kPa", "C", "",
    ));
    for metric in metrics {
        out.push_str(&format!(
            "{:>7.0} {:>6.2} {:>5.2} {:>6.1} {:>8.2} {:>7.2} {:>8.1} {:>7.2} {:>7.0} {:>5.0} {:>7} {:>7} {:>8.0} {:>8.0} {:>5.2}\n",
            metric.rpm,
            metric.lambda_target,
            metric.throttle,
            metric.map_kpa,
            metric.torque_nm,
            metric.power_kw,
            metric.fuel_flow_mg_per_s,
            metric.air_flow_g_per_s,
            metric.bmep_kpa,
            metric.volumetric_efficiency_percent,
            format_optional(metric.chamber_lambda),
            format_optional(metric.exhaust_lambda),
            metric.peak_pressure_kpa_rel,
            metric.peak_temperature_c,
            metric.combustion_event_ratio,
        ));
    }
    out
}

fn format_sweep_report(pulls: &[SweepPull]) -> String {
    let mut out = String::from("=== Sweep pulls ===\n");
    for pull in pulls {
        out.push_str(&format!(
            "\n-- throttle {:.2}, lambda target {:.2} --\n",
            pull.throttle, pull.lambda_target,
        ));
        out.push_str(&format!(
            "{:>7} {:>8} {:>7} {:>7} {:>5} {:>7}\n",
            "rpm", "torque", "power", "bmep", "VE", "lam_ch",
        ));
        out.push_str(&format!(
            "{:>7} {:>8} {:>7} {:>7} {:>5} {:>7}\n",
            "", "Nm", "kW", "kPa", "%", "",
        ));
        for bin in &pull.bins {
            out.push_str(&format!(
                "{:>7.0} {:>8.2} {:>7.2} {:>7.0} {:>5.0} {:>7}\n",
                bin.rpm,
                bin.torque_nm,
                bin.power_kw,
                bin.bmep_kpa,
                bin.volumetric_efficiency_percent,
                format_optional(bin.chamber_lambda),
            ));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn definition() -> EngineDefinition {
        EngineDefinition::from_json_str(include_str!("../data/engines/gn250.json"))
            .expect("GN250 JSON should parse")
    }

    #[test]
    fn linear_axis_includes_endpoints() {
        let axis = AxisSpec {
            count: 5,
            min: 0.0,
            max: 1.0,
            spacing: Spacing::Linear,
        };
        assert_eq!(axis.values(), vec![0.0, 0.25, 0.5, 0.75, 1.0]);
    }

    #[test]
    fn single_point_axis_returns_min() {
        let axis = AxisSpec {
            count: 1,
            min: 0.3,
            max: 1.0,
            spacing: Spacing::Linear,
        };
        assert_eq!(axis.values(), vec![0.3]);
    }

    #[test]
    fn log_axis_is_geometric() {
        let axis = AxisSpec {
            count: 3,
            min: 1.0,
            max: 100.0,
            spacing: Spacing::Log,
        };
        let values = axis.values();
        assert!((values[0] - 1.0).abs() < 1.0e-9);
        assert!((values[1] - 10.0).abs() < 1.0e-6);
        assert!((values[2] - 100.0).abs() < 1.0e-6);
    }

    #[test]
    fn grid_expands_to_cartesian_product() {
        let grid = GridConfig {
            throttle: Some(AxisSpec {
                count: 2,
                min: 0.5,
                max: 1.0,
                spacing: Spacing::Linear,
            }),
            map_kpa: None,
            rpm: AxisSpec {
                count: 2,
                min: 2000.0,
                max: 3000.0,
                spacing: Spacing::Linear,
            },
            lambda: AxisSpec {
                count: 1,
                min: 1.0,
                max: 1.0,
                spacing: Spacing::Linear,
            },
            dwell_seconds: 0.05,
            settle_fraction: 0.5,
        };
        let definition = definition();
        let profile = ProfileSelection::RealTime.to_profile(&EngineHandlingDefinition::default());
        let metrics = run_grid(&definition, profile, &grid);
        assert_eq!(metrics.len(), 4);
        for metric in &metrics {
            assert!(metric.rpm > 0.0);
            assert!(metric.torque_nm.is_finite());
        }
    }

    #[test]
    fn map_setpoint_is_tracked_by_the_manifold() {
        let definition = definition();
        let profile = ProfileSelection::RealTime.to_profile(&EngineHandlingDefinition::default());
        let target_kpa = 60.0;

        let metrics = run_operating_point(
            &definition,
            profile,
            OperatingPoint {
                rpm: 3000.0,
                load: Load::ManifoldPressureKpa(target_kpa),
                lambda_target: 1.0,
            },
            0.4,
            0.5,
        );

        assert!(
            (metrics.map_kpa - target_kpa).abs() < 3.0,
            "manifold settled at {:.1} kPa, target {:.1} kPa",
            metrics.map_kpa,
            target_kpa,
        );
    }

    #[test]
    fn lower_map_reduces_trapped_air_and_torque() {
        let definition = definition();
        let profile = ProfileSelection::RealTime.to_profile(&EngineHandlingDefinition::default());
        let point = |map_kpa: f64| {
            run_operating_point(
                &definition,
                profile,
                OperatingPoint {
                    rpm: 3000.0,
                    load: Load::ManifoldPressureKpa(map_kpa),
                    lambda_target: 1.0,
                },
                0.4,
                0.5,
            )
        };
        assert!(point(50.0).torque_nm < point(100.0).torque_nm);
    }

    #[test]
    fn sweep_produces_binned_pulls() {
        let definition = definition();
        let profile = ProfileSelection::RealTime.to_profile(&EngineHandlingDefinition::default());
        let sweep = SweepConfig {
            runs: 2,
            duration_seconds: 0.1,
            throttle_points: 1,
            lambda_points: 1,
            rpm_min: 2000.0,
            rpm_max: 4000.0,
            lambda_min: 1.0,
            lambda_max: 1.0,
            rpm_bin: 500.0,
        };
        let pulls = run_sweep(&definition, profile, &sweep);
        assert_eq!(pulls.len(), 1);
        assert_eq!(pulls[0].throttle, 1.0);
        assert!(!pulls[0].bins.is_empty());
        assert!(pulls[0].bins.iter().all(|bin| bin.rpm >= 2000.0));
    }
}
