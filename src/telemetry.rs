use std::collections::VecDeque;

use crate::engine_config::EngineDefinition;
use crate::engine_handling::EngineHandlingDefinition;
use crate::physics::chamber::ChamberSpeciesMasses;
use crate::profiles::SimulationProfile;
use crate::single_cylinder::{
    SingleCylinderStepInputs, SingleCylinderStepOutput, rpm_to_rad_per_s,
};
use crate::valve::ENGINE_CYCLE_RADIANS;

const ENGINE_PANEL_RATE_HZ: f64 = 15.0;
const ANGLE_PLOT_RATE_HZ: f64 = 15.0;
const TIME_PLOT_RATE_HZ: f64 = 30.0;
const RPM_PLOT_RATE_HZ: f64 = 30.0;
const ENGINE_PANEL_ROLLING_FRAMES: usize = 15;
const TIME_HISTORY_CAPACITY: usize = 300;
const CYCLE_HISTORY_CAPACITY: usize = 16;
const ANGLE_TRACE_BINS: usize = 181;
const RPM_SAMPLE_INTERVAL: f64 = 100.0;
const RPM_SAMPLE_TOLERANCE: f64 = 20.0;
const RPM_SAMPLE_MAX: f64 = 12_000.0;
const RPM_SAMPLE_COUNT: usize = (RPM_SAMPLE_MAX as usize) / (RPM_SAMPLE_INTERVAL as usize);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TelemetryAvailability {
    Measured,
    Placeholder,
    Unavailable,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TelemetryScalar {
    pub value: Option<f64>,
    pub availability: TelemetryAvailability,
}

impl TelemetryScalar {
    pub fn measured(value: f64) -> Self {
        Self {
            value: Some(value),
            availability: TelemetryAvailability::Measured,
        }
    }

    pub fn placeholder(value: f64) -> Self {
        Self {
            value: Some(value),
            availability: TelemetryAvailability::Placeholder,
        }
    }

    pub fn unavailable() -> Self {
        Self {
            value: None,
            availability: TelemetryAvailability::Unavailable,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EngineControls {
    pub throttle_position: f64,
    pub idle_throttle_fraction: f64,
    pub lambda_target: f64,
    pub starter_torque_nm: f64,
    pub starter_speed_limit_rpm: f64,
    pub starter_enabled: bool,
    pub added_inertia_kg_m2: f64,
    pub added_torque_load_nm: f64,
    pub spark_enabled: bool,
    pub fuel_enabled: bool,
    pub redline_spark_cut_rpm: f64,
    pub dyno_mode_enabled: bool,
    pub dyno_target_rpm: f64,
    /// Direct MAP override: when enabled, the intake plenum is pinned to
    /// `map_override_pa` instead of being set by the throttle.
    pub map_override_enabled: bool,
    pub map_override_pa: f64,
}

/// Largest manifold absolute pressure the MAP override accepts (2000 kPa).
pub const MAP_OVERRIDE_MAX_PA: f64 = 2_000_000.0;

impl Default for EngineControls {
    fn default() -> Self {
        Self {
            throttle_position: 1.0,
            idle_throttle_fraction: 0.0,
            lambda_target: 1.0,
            starter_torque_nm: 20.0,
            starter_speed_limit_rpm: 900.0,
            starter_enabled: false,
            added_inertia_kg_m2: 0.0,
            added_torque_load_nm: 0.0,
            spark_enabled: true,
            fuel_enabled: true,
            redline_spark_cut_rpm: 8_500.0,
            dyno_mode_enabled: false,
            dyno_target_rpm: 3_000.0,
            map_override_enabled: false,
            map_override_pa: 101_325.0,
        }
    }
}

impl EngineControls {
    pub fn to_step_inputs(
        self,
        definition: &EngineDefinition,
        current_rpm: f64,
    ) -> SingleCylinderStepInputs {
        self.to_step_inputs_with_redline_cut_active(definition, current_rpm, false)
    }

    pub fn to_step_inputs_with_redline_cut_active(
        self,
        _definition: &EngineDefinition,
        current_rpm: f64,
        redline_cut_active: bool,
    ) -> SingleCylinderStepInputs {
        let spark_enabled =
            self.spark_enabled && current_rpm < self.redline_spark_cut_rpm && !redline_cut_active;
        SingleCylinderStepInputs {
            external_load_torque_nm: self.added_torque_load_nm.max(0.0),
            fixed_crank_speed_rad_per_s: self
                .dyno_mode_enabled
                .then(|| rpm_to_rad_per_s(self.dyno_target_rpm.max(1.0))),
            starter_torque_nm: self.starter_torque_nm.max(0.0),
            starter_speed_limit_rpm: self.starter_speed_limit_rpm.max(0.0),
            starter_enabled: self.starter_enabled,
            added_inertia_kg_m2: self.added_inertia_kg_m2.max(0.0),
            spark_enabled,
            fuel_enabled: self.fuel_enabled,
            lambda_target: self.lambda_target.clamp(0.1, 2.0),
            throttle_position: self.throttle_position.clamp(0.0, 1.0),
            idle_throttle_fraction: self.idle_throttle_fraction.clamp(0.0, 1.0),
            throttle_effective_area_fraction: None,
            forced_map_pressure_pa: self
                .map_override_enabled
                .then(|| self.map_override_pa.clamp(1.0, MAP_OVERRIDE_MAX_PA)),
            repinned_plenum_pressure_pa: None,
        }
    }

    pub fn throttle_effective_area_fraction(self, definition: &EngineDefinition) -> f64 {
        let throttle_position = self.throttle_position.clamp(0.0, 1.0);
        let idle_throttle_fraction = self.idle_throttle_fraction.clamp(0.0, 1.0);
        let throttle_area_m2 = definition
            .intake_exhaust
            .throttle_maximum_area_m2
            .max(1.0e-6);
        let maximum_idle_area_m2 = definition.intake_exhaust.effective_idle_throttle_area_m2();
        let requested_area_m2 = throttle_area_m2 * throttle_position.powi(2)
            + maximum_idle_area_m2 * idle_throttle_fraction.powi(2);

        (requested_area_m2 / (throttle_area_m2 + maximum_idle_area_m2)).clamp(0.0, 1.0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EngineInstantSample {
    pub elapsed_time_seconds: f64,
    pub crank_angle_rad: f64,
    pub rpm: f64,
    pub pressure_pa: f64,
    pub temperature_k: f64,
    pub chamber_mass_kg: f64,
    pub intake_effective_area_m2: f64,
    pub exhaust_effective_area_m2: f64,
    pub intake_mass_flow_kg_per_s: f64,
    pub exhaust_mass_flow_kg_per_s: f64,
    /// Mean gas velocity through the intake valve (signed: positive into the
    /// cylinder). Derived from mass flow, effective area and upstream density.
    pub intake_valve_velocity_m_per_s: f64,
    /// Mean gas velocity through the exhaust valve (signed: positive out of the
    /// cylinder).
    pub exhaust_valve_velocity_m_per_s: f64,
    pub torque_nm: f64,
    pub power_kw: f64,
    pub map_pa: f64,
    pub exhaust_exit_pressure_pa: f64,
    pub fuel_flow_mg_per_s: f64,
    pub air_flow_g_per_s: f64,
    pub bmep_kpa: f64,
    pub volumetric_efficiency_percent: f64,
    pub combustion_event: bool,
    pub chamber_lambda: TelemetryScalar,
    pub exhaust_lambda: TelemetryScalar,
    /// Reconstructed charge air mass in grams (combustion-invariant). Paired
    /// with `chamber_charge_fuel_scaled_g` for the AFR overlay trace.
    pub chamber_charge_air_mass_g: f64,
    /// Reconstructed charge fuel mass scaled by the stoichiometric AFR, in
    /// grams. Drawn against `chamber_charge_air_mass_g` so the two traces
    /// overlap when the trapped mixture sits at lambda = 1.
    pub chamber_charge_fuel_scaled_g: f64,
}

impl EngineInstantSample {
    pub fn from_step_output(
        output: SingleCylinderStepOutput,
        controls: EngineControls,
        definition: &EngineDefinition,
    ) -> Self {
        let angular_speed_rad_per_s = output.crank_speed_rad_per_s;
        let power_kw = output.indicated_torque_nm * angular_speed_rad_per_s / 1000.0;
        let fuel_flow_mg_per_s = if controls.fuel_enabled {
            fuel_mass_for_lambda(definition, controls.lambda_target) * output.rpm / 120.0 * 1.0e6
        } else {
            0.0
        };
        let bmep_kpa = mean_effective_pressure_kpa(output.indicated_torque_nm, definition);
        let volumetric_efficiency_percent =
            volumetric_efficiency_percent(output.intake_mass_flow_kg_per_s, output.rpm, definition);

        let stoichiometric_air_fuel_ratio = definition
            .combustion
            .mixture_limits
            .stoichiometric_air_fuel_ratio;
        let chamber_species = ChamberSpeciesMasses {
            oxygen_kg: output.chamber_oxygen_mass_kg,
            fuel_kg: output.chamber_fuel_mass_kg,
            inert_kg: output.chamber_inert_mass_kg,
            products_kg: output.chamber_products_mass_kg,
        };
        let (charge_air_kg, charge_fuel_kg) =
            chamber_species.charge_air_and_fuel_kg(stoichiometric_air_fuel_ratio);

        let gas_constant = definition.gas.gas_constant_j_per_kg_k;
        // Intake fills from the runner; exhaust blows down from the cylinder.
        // Use the appropriate upstream density for the mean port-gas velocity.
        let intake_density_kg_per_m3 = gas_density_kg_per_m3(
            output.intake_runner_pressure_pa,
            definition.boundaries.intake_temperature_k,
            gas_constant,
        );
        let cylinder_density_kg_per_m3 = gas_density_kg_per_m3(
            output.cylinder_pressure_pa,
            output.cylinder_temperature_k,
            gas_constant,
        );
        let intake_valve_velocity_m_per_s = valve_gas_velocity_m_per_s(
            output.intake_mass_flow_kg_per_s,
            output.intake_effective_area_m2,
            intake_density_kg_per_m3,
        );
        let exhaust_valve_velocity_m_per_s = valve_gas_velocity_m_per_s(
            output.exhaust_mass_flow_kg_per_s,
            output.exhaust_effective_area_m2,
            cylinder_density_kg_per_m3,
        );

        Self {
            elapsed_time_seconds: output.elapsed_time_seconds,
            crank_angle_rad: output.crank_angle_rad,
            rpm: output.rpm,
            pressure_pa: output.cylinder_pressure_pa,
            temperature_k: output.cylinder_temperature_k,
            chamber_mass_kg: output.chamber_mass_kg,
            intake_effective_area_m2: output.intake_effective_area_m2,
            exhaust_effective_area_m2: output.exhaust_effective_area_m2,
            intake_mass_flow_kg_per_s: output.intake_mass_flow_kg_per_s,
            exhaust_mass_flow_kg_per_s: output.exhaust_mass_flow_kg_per_s,
            intake_valve_velocity_m_per_s,
            exhaust_valve_velocity_m_per_s,
            torque_nm: output.indicated_torque_nm,
            power_kw,
            map_pa: output.intake_plenum_pressure_pa,
            exhaust_exit_pressure_pa: output.exhaust_exit_pressure_pa,
            fuel_flow_mg_per_s,
            air_flow_g_per_s: output.intake_mass_flow_kg_per_s * 1000.0,
            bmep_kpa,
            volumetric_efficiency_percent,
            combustion_event: output.combustion_heat_added_j > 0.0,
            chamber_lambda: output
                .chamber_lambda
                .map(TelemetryScalar::measured)
                .unwrap_or_else(|| TelemetryScalar::placeholder(controls.lambda_target)),
            exhaust_lambda: output
                .exhaust_lambda
                .map(TelemetryScalar::measured)
                .unwrap_or_else(TelemetryScalar::unavailable),
            chamber_charge_air_mass_g: charge_air_kg * 1000.0,
            chamber_charge_fuel_scaled_g: charge_fuel_kg * stoichiometric_air_fuel_ratio * 1000.0,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct EngineDataSnapshot {
    pub rpm: f64,
    pub rpm_delta: f64,
    pub torque_nm: f64,
    pub power_kw: f64,
    pub fuel_flow_mg_per_s: f64,
    pub air_flow_g_per_s: f64,
    pub bmep_kpa: f64,
    pub volumetric_efficiency_percent: f64,
    pub chamber_lambda: TelemetryScalar,
    pub exhaust_lambda: TelemetryScalar,
    pub peak_cylinder_temperature_k: f64,
    pub peak_cylinder_pressure_pa: f64,
    pub ambient_pressure_pa: f64,
    pub map_pa: f64,
    pub combustion_event_ratio: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct AngleTraceSnapshot {
    pub pressure_pa: Vec<[f64; 2]>,
    pub temperature_k: Vec<[f64; 2]>,
    pub intake_effective_area_m2: Vec<[f64; 2]>,
    pub exhaust_effective_area_m2: Vec<[f64; 2]>,
    pub exhaust_exit_pressure_pa: Vec<[f64; 2]>,
    pub chamber_air_mass_g: Vec<[f64; 2]>,
    pub chamber_fuel_scaled_g: Vec<[f64; 2]>,
    pub intake_valve_velocity_m_per_s: Vec<[f64; 2]>,
    pub exhaust_valve_velocity_m_per_s: Vec<[f64; 2]>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TimePlotPoint {
    pub time_seconds: f64,
    pub rpm: f64,
    pub map_pa: f64,
    pub exhaust_exit_pressure_pa: f64,
    pub torque_nm: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RpmPlotPoint {
    pub age_index: usize,
    pub rpm: f64,
    pub torque_nm: f64,
    pub power_kw: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct EngineFrameTelemetry {
    pub engine_data: EngineDataSnapshot,
    pub angle_trace: AngleTraceSnapshot,
    pub time_history: Vec<TimePlotPoint>,
    pub rpm_history: Vec<RpmPlotPoint>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct RollingWindow<T> {
    capacity: usize,
    entries: VecDeque<T>,
}

impl<T> RollingWindow<T> {
    pub fn new(capacity: usize) -> Self {
        assert!(capacity > 0, "rolling window capacity must be positive");
        Self {
            capacity,
            entries: VecDeque::with_capacity(capacity),
        }
    }

    pub fn push(&mut self, value: T) {
        if self.entries.len() == self.capacity {
            self.entries.pop_front();
        }
        self.entries.push_back(value);
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn iter(&self) -> impl Iterator<Item = &T> {
        self.entries.iter()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct EngineDataFrame {
    rpm: f64,
    torque_nm: f64,
    power_kw: f64,
    fuel_flow_mg_per_s: f64,
    air_flow_g_per_s: f64,
    bmep_kpa: f64,
    volumetric_efficiency_percent: f64,
    map_pa: f64,
    exhaust_exit_pressure_pa: f64,
    chamber_lambda: TelemetryScalar,
    exhaust_lambda: TelemetryScalar,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct PendingFrameAccumulator {
    duration_seconds: f64,
    rpm_weighted_sum: f64,
    torque_weighted_sum: f64,
    power_weighted_sum: f64,
    fuel_flow_weighted_sum: f64,
    air_flow_weighted_sum: f64,
    bmep_weighted_sum: f64,
    volumetric_efficiency_weighted_sum: f64,
    map_weighted_sum: f64,
    exhaust_exit_pressure_weighted_sum: f64,
    chamber_lambda: ScalarFrameAccumulator,
    exhaust_lambda: ScalarFrameAccumulator,
}

impl PendingFrameAccumulator {
    fn push(&mut self, sample: EngineInstantSample, sample_duration_seconds: f64) {
        self.duration_seconds += sample_duration_seconds;
        self.rpm_weighted_sum += sample.rpm * sample_duration_seconds;
        self.torque_weighted_sum += sample.torque_nm * sample_duration_seconds;
        self.power_weighted_sum += sample.power_kw * sample_duration_seconds;
        self.fuel_flow_weighted_sum += sample.fuel_flow_mg_per_s * sample_duration_seconds;
        self.air_flow_weighted_sum += sample.air_flow_g_per_s * sample_duration_seconds;
        self.bmep_weighted_sum += sample.bmep_kpa * sample_duration_seconds;
        self.volumetric_efficiency_weighted_sum +=
            sample.volumetric_efficiency_percent * sample_duration_seconds;
        self.map_weighted_sum += sample.map_pa * sample_duration_seconds;
        self.exhaust_exit_pressure_weighted_sum +=
            sample.exhaust_exit_pressure_pa * sample_duration_seconds;
        self.chamber_lambda
            .push(sample.chamber_lambda, sample_duration_seconds);
        self.exhaust_lambda
            .push(sample.exhaust_lambda, sample_duration_seconds);
    }

    fn finalize(self) -> Option<EngineDataFrame> {
        if self.duration_seconds <= 0.0 {
            return None;
        }

        Some(EngineDataFrame {
            rpm: self.rpm_weighted_sum / self.duration_seconds,
            torque_nm: self.torque_weighted_sum / self.duration_seconds,
            power_kw: self.power_weighted_sum / self.duration_seconds,
            fuel_flow_mg_per_s: self.fuel_flow_weighted_sum / self.duration_seconds,
            air_flow_g_per_s: self.air_flow_weighted_sum / self.duration_seconds,
            bmep_kpa: self.bmep_weighted_sum / self.duration_seconds,
            volumetric_efficiency_percent: self.volumetric_efficiency_weighted_sum
                / self.duration_seconds,
            map_pa: self.map_weighted_sum / self.duration_seconds,
            exhaust_exit_pressure_pa: self.exhaust_exit_pressure_weighted_sum
                / self.duration_seconds,
            chamber_lambda: self.chamber_lambda.finalize(),
            exhaust_lambda: self.exhaust_lambda.finalize(),
        })
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct ScalarFrameAccumulator {
    measured_weighted_sum: f64,
    measured_duration_seconds: f64,
    placeholder_weighted_sum: f64,
    placeholder_duration_seconds: f64,
}

impl ScalarFrameAccumulator {
    fn push(&mut self, scalar: TelemetryScalar, sample_duration_seconds: f64) {
        let Some(value) = scalar.value else {
            return;
        };
        match scalar.availability {
            TelemetryAvailability::Measured => {
                self.measured_weighted_sum += value * sample_duration_seconds;
                self.measured_duration_seconds += sample_duration_seconds;
            }
            TelemetryAvailability::Placeholder => {
                self.placeholder_weighted_sum += value * sample_duration_seconds;
                self.placeholder_duration_seconds += sample_duration_seconds;
            }
            TelemetryAvailability::Unavailable => {}
        }
    }

    fn finalize(self) -> TelemetryScalar {
        if self.measured_duration_seconds > 0.0 {
            TelemetryScalar::measured(self.measured_weighted_sum / self.measured_duration_seconds)
        } else if self.placeholder_duration_seconds > 0.0 {
            TelemetryScalar::placeholder(
                self.placeholder_weighted_sum / self.placeholder_duration_seconds,
            )
        } else {
            TelemetryScalar::unavailable()
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct CycleAccumulator {
    current_rpm_min: f64,
    current_rpm_max: f64,
    current_duration_seconds: f64,
    current_rpm_weighted_sum: f64,
    current_torque_weighted_sum: f64,
    current_power_weighted_sum: f64,
    current_peak_pressure_pa: f64,
    current_peak_temperature_k: f64,
    current_combustion_event_seen: bool,
    current_trace: AngleTraceBuffers,
    last_completed_trace: Option<AngleTraceBuffers>,
    completed_cycles: RollingWindow<CycleSummary>,
    rpm_curve_sampler: RpmCurveSampler,
    previous_angle_rad: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct CycleSummary {
    rpm_delta: f64,
    peak_pressure_pa: f64,
    peak_temperature_k: f64,
    combustion_event_ratio: f64,
    mean_rpm: f64,
    mean_torque_nm: f64,
    mean_power_kw: f64,
}

#[derive(Debug, Clone, PartialEq)]
struct RpmCurveSampler {
    samples: [Option<RpmPlotPoint>; RPM_SAMPLE_COUNT],
}

impl RpmCurveSampler {
    fn new() -> Self {
        Self {
            samples: [None; RPM_SAMPLE_COUNT],
        }
    }

    fn record_cycle(&mut self, cycle: CycleSummary) {
        let sample_rpm = (cycle.mean_rpm / RPM_SAMPLE_INTERVAL).round() * RPM_SAMPLE_INTERVAL;
        if !(RPM_SAMPLE_INTERVAL..=RPM_SAMPLE_MAX).contains(&sample_rpm) {
            return;
        }
        if (cycle.mean_rpm - sample_rpm).abs() > RPM_SAMPLE_TOLERANCE {
            return;
        }

        let index = (sample_rpm / RPM_SAMPLE_INTERVAL) as usize - 1;
        self.samples[index] = Some(RpmPlotPoint {
            age_index: 0,
            rpm: sample_rpm,
            torque_nm: cycle.mean_torque_nm,
            power_kw: cycle.mean_torque_nm * rpm_to_rad_per_s(sample_rpm) / 1000.0,
        });
    }

    fn points(&self) -> Vec<RpmPlotPoint> {
        self.samples.iter().flatten().copied().collect()
    }
}

#[derive(Debug, Clone, PartialEq)]
struct AngleTraceBuffers {
    pressure_pa: [f64; ANGLE_TRACE_BINS],
    temperature_k: [f64; ANGLE_TRACE_BINS],
    intake_effective_area_m2: [f64; ANGLE_TRACE_BINS],
    exhaust_effective_area_m2: [f64; ANGLE_TRACE_BINS],
    exhaust_exit_pressure_pa: [f64; ANGLE_TRACE_BINS],
    chamber_air_mass_g: [f64; ANGLE_TRACE_BINS],
    chamber_fuel_scaled_g: [f64; ANGLE_TRACE_BINS],
    intake_valve_velocity_m_per_s: [f64; ANGLE_TRACE_BINS],
    exhaust_valve_velocity_m_per_s: [f64; ANGLE_TRACE_BINS],
    filled: [bool; ANGLE_TRACE_BINS],
}

impl Default for AngleTraceBuffers {
    fn default() -> Self {
        Self {
            pressure_pa: [0.0; ANGLE_TRACE_BINS],
            temperature_k: [0.0; ANGLE_TRACE_BINS],
            intake_effective_area_m2: [0.0; ANGLE_TRACE_BINS],
            exhaust_effective_area_m2: [0.0; ANGLE_TRACE_BINS],
            exhaust_exit_pressure_pa: [0.0; ANGLE_TRACE_BINS],
            chamber_air_mass_g: [0.0; ANGLE_TRACE_BINS],
            chamber_fuel_scaled_g: [0.0; ANGLE_TRACE_BINS],
            intake_valve_velocity_m_per_s: [0.0; ANGLE_TRACE_BINS],
            exhaust_valve_velocity_m_per_s: [0.0; ANGLE_TRACE_BINS],
            filled: [false; ANGLE_TRACE_BINS],
        }
    }
}

impl AngleTraceBuffers {
    fn record(&mut self, sample: EngineInstantSample) {
        let index = angle_trace_index(sample.crank_angle_rad);
        self.pressure_pa[index] = sample.pressure_pa;
        self.temperature_k[index] = sample.temperature_k;
        self.intake_effective_area_m2[index] = sample.intake_effective_area_m2;
        self.exhaust_effective_area_m2[index] = sample.exhaust_effective_area_m2;
        self.exhaust_exit_pressure_pa[index] = sample.exhaust_exit_pressure_pa;
        self.chamber_air_mass_g[index] = sample.chamber_charge_air_mass_g;
        self.chamber_fuel_scaled_g[index] = sample.chamber_charge_fuel_scaled_g;
        self.intake_valve_velocity_m_per_s[index] = sample.intake_valve_velocity_m_per_s;
        self.exhaust_valve_velocity_m_per_s[index] = sample.exhaust_valve_velocity_m_per_s;
        self.filled[index] = true;
    }

    fn to_snapshot(&self) -> AngleTraceSnapshot {
        AngleTraceSnapshot {
            pressure_pa: points_from_trace(&self.pressure_pa, &self.filled),
            temperature_k: points_from_trace(&self.temperature_k, &self.filled),
            intake_effective_area_m2: points_from_trace(
                &self.intake_effective_area_m2,
                &self.filled,
            ),
            exhaust_effective_area_m2: points_from_trace(
                &self.exhaust_effective_area_m2,
                &self.filled,
            ),
            exhaust_exit_pressure_pa: points_from_trace(
                &self.exhaust_exit_pressure_pa,
                &self.filled,
            ),
            chamber_air_mass_g: points_from_trace(&self.chamber_air_mass_g, &self.filled),
            chamber_fuel_scaled_g: points_from_trace(&self.chamber_fuel_scaled_g, &self.filled),
            intake_valve_velocity_m_per_s: points_from_trace(
                &self.intake_valve_velocity_m_per_s,
                &self.filled,
            ),
            exhaust_valve_velocity_m_per_s: points_from_trace(
                &self.exhaust_valve_velocity_m_per_s,
                &self.filled,
            ),
        }
    }
}

impl CycleAccumulator {
    pub fn new() -> Self {
        Self {
            current_rpm_min: f64::INFINITY,
            current_rpm_max: f64::NEG_INFINITY,
            current_duration_seconds: 0.0,
            current_rpm_weighted_sum: 0.0,
            current_torque_weighted_sum: 0.0,
            current_power_weighted_sum: 0.0,
            current_peak_pressure_pa: 0.0,
            current_peak_temperature_k: 0.0,
            current_combustion_event_seen: false,
            current_trace: AngleTraceBuffers::default(),
            last_completed_trace: None,
            completed_cycles: RollingWindow::new(CYCLE_HISTORY_CAPACITY),
            rpm_curve_sampler: RpmCurveSampler::new(),
            previous_angle_rad: None,
        }
    }

    fn push(&mut self, sample: EngineInstantSample, sample_duration_seconds: f64) {
        let wrapped = self
            .previous_angle_rad
            .is_some_and(|previous| sample.crank_angle_rad < previous);
        if wrapped {
            self.complete_cycle();
        }

        self.current_duration_seconds += sample_duration_seconds;
        self.current_rpm_weighted_sum += sample.rpm * sample_duration_seconds;
        self.current_torque_weighted_sum += sample.torque_nm * sample_duration_seconds;
        self.current_power_weighted_sum += sample.power_kw * sample_duration_seconds;
        self.current_rpm_min = self.current_rpm_min.min(sample.rpm);
        self.current_rpm_max = self.current_rpm_max.max(sample.rpm);
        self.current_peak_pressure_pa = self.current_peak_pressure_pa.max(sample.pressure_pa);
        self.current_peak_temperature_k = self.current_peak_temperature_k.max(sample.temperature_k);
        self.current_combustion_event_seen |= sample.combustion_event;
        self.current_trace.record(sample);
        self.previous_angle_rad = Some(sample.crank_angle_rad);
    }

    fn complete_cycle(&mut self) {
        if self.current_duration_seconds <= 0.0 {
            self.reset_current_cycle();
            return;
        }

        let summary = CycleSummary {
            rpm_delta: (self.current_rpm_max - self.current_rpm_min).max(0.0),
            peak_pressure_pa: self.current_peak_pressure_pa,
            peak_temperature_k: self.current_peak_temperature_k,
            combustion_event_ratio: if self.current_combustion_event_seen {
                1.0
            } else {
                0.0
            },
            mean_rpm: self.current_rpm_weighted_sum / self.current_duration_seconds,
            mean_torque_nm: self.current_torque_weighted_sum / self.current_duration_seconds,
            mean_power_kw: self.current_power_weighted_sum / self.current_duration_seconds,
        };
        self.completed_cycles.push(summary);
        self.rpm_curve_sampler.record_cycle(summary);
        self.last_completed_trace = Some(self.current_trace.clone());
        self.reset_current_cycle();
    }

    fn reset_current_cycle(&mut self) {
        self.current_rpm_min = f64::INFINITY;
        self.current_rpm_max = f64::NEG_INFINITY;
        self.current_duration_seconds = 0.0;
        self.current_rpm_weighted_sum = 0.0;
        self.current_torque_weighted_sum = 0.0;
        self.current_power_weighted_sum = 0.0;
        self.current_peak_pressure_pa = 0.0;
        self.current_peak_temperature_k = 0.0;
        self.current_combustion_event_seen = false;
        self.current_trace = AngleTraceBuffers::default();
    }

    fn latest_rpm_delta(&self) -> f64 {
        self.completed_cycles
            .iter()
            .last()
            .map(|cycle| cycle.rpm_delta)
            .unwrap_or_else(|| (self.current_rpm_max - self.current_rpm_min).max(0.0))
    }

    fn latest_peak_pressure_pa(&self) -> f64 {
        self.completed_cycles
            .iter()
            .last()
            .map(|cycle| cycle.peak_pressure_pa)
            .unwrap_or(self.current_peak_pressure_pa)
    }

    fn latest_peak_temperature_k(&self) -> f64 {
        self.completed_cycles
            .iter()
            .last()
            .map(|cycle| cycle.peak_temperature_k)
            .unwrap_or(self.current_peak_temperature_k)
    }

    fn combustion_event_ratio(&self) -> f64 {
        if self.completed_cycles.is_empty() {
            return if self.current_combustion_event_seen {
                1.0
            } else {
                0.0
            };
        }

        let sum: f64 = self
            .completed_cycles
            .iter()
            .map(|cycle| cycle.combustion_event_ratio)
            .sum();
        sum / self.completed_cycles.len() as f64
    }

    fn trace_snapshot(&self) -> AngleTraceSnapshot {
        if self.current_duration_seconds >= 1.0 / ANGLE_PLOT_RATE_HZ
            || self.last_completed_trace.is_none()
        {
            self.current_trace.to_snapshot()
        } else {
            self.last_completed_trace
                .as_ref()
                .expect("checked above")
                .to_snapshot()
        }
    }

    fn rpm_history(&self) -> Vec<RpmPlotPoint> {
        self.rpm_curve_sampler.points()
    }
}

impl Default for CycleAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct TelemetryAggregator {
    definition: EngineDefinition,
    controls: EngineControls,
    pending_engine_panel: PendingFrameAccumulator,
    pending_time_plot: PendingFrameAccumulator,
    elapsed_since_engine_panel_publish_seconds: f64,
    elapsed_since_time_publish_seconds: f64,
    elapsed_since_angle_publish_seconds: f64,
    elapsed_since_rpm_publish_seconds: f64,
    published_engine_frames: RollingWindow<EngineDataFrame>,
    cycle_accumulator: CycleAccumulator,
    time_history: RollingWindow<TimePlotPoint>,
    latest_sample: Option<EngineInstantSample>,
}

impl TelemetryAggregator {
    pub fn new(definition: EngineDefinition, controls: EngineControls) -> Self {
        Self {
            definition,
            controls,
            pending_engine_panel: PendingFrameAccumulator::default(),
            pending_time_plot: PendingFrameAccumulator::default(),
            elapsed_since_engine_panel_publish_seconds: 0.0,
            elapsed_since_time_publish_seconds: 0.0,
            elapsed_since_angle_publish_seconds: 0.0,
            elapsed_since_rpm_publish_seconds: 0.0,
            published_engine_frames: RollingWindow::new(ENGINE_PANEL_ROLLING_FRAMES),
            cycle_accumulator: CycleAccumulator::new(),
            time_history: RollingWindow::new(TIME_HISTORY_CAPACITY),
            latest_sample: None,
        }
    }

    pub fn controls(&self) -> EngineControls {
        self.controls
    }

    pub fn set_controls(&mut self, controls: EngineControls) {
        self.controls = controls;
    }

    pub fn ingest_step_output(
        &mut self,
        output: SingleCylinderStepOutput,
        sample_duration_seconds: f64,
    ) {
        let sample = EngineInstantSample::from_step_output(output, self.controls, &self.definition);
        self.cycle_accumulator.push(sample, sample_duration_seconds);
        self.pending_engine_panel
            .push(sample, sample_duration_seconds);
        self.pending_time_plot.push(sample, sample_duration_seconds);
        self.elapsed_since_engine_panel_publish_seconds += sample_duration_seconds;
        self.elapsed_since_time_publish_seconds += sample_duration_seconds;
        self.elapsed_since_angle_publish_seconds += sample_duration_seconds;
        self.elapsed_since_rpm_publish_seconds += sample_duration_seconds;
        self.latest_sample = Some(sample);
    }

    pub fn publish_ready(&mut self) -> Option<EngineFrameTelemetry> {
        let latest_sample = self.latest_sample?;
        if self.elapsed_since_engine_panel_publish_seconds < 1.0 / ENGINE_PANEL_RATE_HZ
            && self.elapsed_since_time_publish_seconds < 1.0 / TIME_PLOT_RATE_HZ
            && self.elapsed_since_angle_publish_seconds < 1.0 / ANGLE_PLOT_RATE_HZ
            && self.elapsed_since_rpm_publish_seconds < 1.0 / RPM_PLOT_RATE_HZ
        {
            return None;
        }

        if self.elapsed_since_engine_panel_publish_seconds >= 1.0 / ENGINE_PANEL_RATE_HZ {
            if let Some(frame) = self.pending_engine_panel.finalize() {
                self.published_engine_frames.push(frame);
            }
            self.pending_engine_panel = PendingFrameAccumulator::default();
            self.elapsed_since_engine_panel_publish_seconds = 0.0;
            self.elapsed_since_angle_publish_seconds = 0.0;
            self.elapsed_since_rpm_publish_seconds = 0.0;
        }

        if self.elapsed_since_time_publish_seconds >= 1.0 / TIME_PLOT_RATE_HZ {
            if let Some(frame) = self.pending_time_plot.finalize() {
                self.time_history.push(TimePlotPoint {
                    time_seconds: latest_sample.elapsed_time_seconds,
                    rpm: frame.rpm,
                    map_pa: frame.map_pa,
                    exhaust_exit_pressure_pa: frame.exhaust_exit_pressure_pa,
                    torque_nm: frame.torque_nm,
                });
            }
            self.pending_time_plot = PendingFrameAccumulator::default();
            self.elapsed_since_time_publish_seconds = 0.0;
        }

        Some(self.snapshot())
    }

    pub fn snapshot(&self) -> EngineFrameTelemetry {
        let averaged_frame = average_engine_data_frames(&self.published_engine_frames);
        EngineFrameTelemetry {
            engine_data: EngineDataSnapshot {
                rpm: averaged_frame.rpm,
                rpm_delta: self.cycle_accumulator.latest_rpm_delta(),
                torque_nm: averaged_frame.torque_nm,
                power_kw: averaged_frame.power_kw,
                fuel_flow_mg_per_s: averaged_frame.fuel_flow_mg_per_s,
                air_flow_g_per_s: averaged_frame.air_flow_g_per_s,
                bmep_kpa: averaged_frame.bmep_kpa,
                volumetric_efficiency_percent: averaged_frame.volumetric_efficiency_percent,
                chamber_lambda: engine_data_scalar_or_latest(
                    averaged_frame.chamber_lambda,
                    self.latest_sample.map(|sample| sample.chamber_lambda),
                    TelemetryScalar::placeholder(self.controls.lambda_target),
                ),
                exhaust_lambda: engine_data_scalar_or_latest(
                    averaged_frame.exhaust_lambda,
                    self.latest_sample.map(|sample| sample.exhaust_lambda),
                    TelemetryScalar::unavailable(),
                ),
                peak_cylinder_temperature_k: self.cycle_accumulator.latest_peak_temperature_k(),
                peak_cylinder_pressure_pa: self.cycle_accumulator.latest_peak_pressure_pa(),
                ambient_pressure_pa: self.definition.boundaries.crankcase_pressure_pa,
                map_pa: averaged_frame.map_pa,
                combustion_event_ratio: self.cycle_accumulator.combustion_event_ratio(),
            },
            angle_trace: self.cycle_accumulator.trace_snapshot(),
            time_history: self.time_history.iter().copied().collect(),
            rpm_history: self.cycle_accumulator.rpm_history(),
        }
    }
}

pub fn default_profile_for_gui(handling: &EngineHandlingDefinition) -> SimulationProfile {
    handling.to_profile()
}

fn average_engine_data_frames(frames: &RollingWindow<EngineDataFrame>) -> EngineDataFrame {
    if frames.is_empty() {
        return EngineDataFrame {
            rpm: 0.0,
            torque_nm: 0.0,
            power_kw: 0.0,
            fuel_flow_mg_per_s: 0.0,
            air_flow_g_per_s: 0.0,
            bmep_kpa: 0.0,
            volumetric_efficiency_percent: 0.0,
            map_pa: 0.0,
            exhaust_exit_pressure_pa: 0.0,
            chamber_lambda: TelemetryScalar::unavailable(),
            exhaust_lambda: TelemetryScalar::unavailable(),
        };
    }

    let count = frames.len() as f64;
    EngineDataFrame {
        rpm: frames.iter().map(|frame| frame.rpm).sum::<f64>() / count,
        torque_nm: frames.iter().map(|frame| frame.torque_nm).sum::<f64>() / count,
        power_kw: frames.iter().map(|frame| frame.power_kw).sum::<f64>() / count,
        fuel_flow_mg_per_s: frames
            .iter()
            .map(|frame| frame.fuel_flow_mg_per_s)
            .sum::<f64>()
            / count,
        air_flow_g_per_s: frames
            .iter()
            .map(|frame| frame.air_flow_g_per_s)
            .sum::<f64>()
            / count,
        bmep_kpa: frames.iter().map(|frame| frame.bmep_kpa).sum::<f64>() / count,
        volumetric_efficiency_percent: frames
            .iter()
            .map(|frame| frame.volumetric_efficiency_percent)
            .sum::<f64>()
            / count,
        map_pa: frames.iter().map(|frame| frame.map_pa).sum::<f64>() / count,
        exhaust_exit_pressure_pa: frames
            .iter()
            .map(|frame| frame.exhaust_exit_pressure_pa)
            .sum::<f64>()
            / count,
        chamber_lambda: average_scalars(frames.iter().map(|frame| frame.chamber_lambda)),
        exhaust_lambda: average_scalars(frames.iter().map(|frame| frame.exhaust_lambda)),
    }
}

fn average_scalars(scalars: impl Iterator<Item = TelemetryScalar>) -> TelemetryScalar {
    let mut accumulator = ScalarFrameAccumulator::default();
    for scalar in scalars {
        accumulator.push(scalar, 1.0);
    }
    accumulator.finalize()
}

fn engine_data_scalar_or_latest(
    frame_scalar: TelemetryScalar,
    latest_scalar: Option<TelemetryScalar>,
    fallback: TelemetryScalar,
) -> TelemetryScalar {
    if frame_scalar.availability != TelemetryAvailability::Unavailable {
        return frame_scalar;
    }
    latest_scalar
        .filter(|scalar| scalar.availability != TelemetryAvailability::Unavailable)
        .unwrap_or(fallback)
}

fn gas_density_kg_per_m3(
    pressure_pa: f64,
    temperature_k: f64,
    gas_constant_j_per_kg_k: f64,
) -> f64 {
    if temperature_k <= 0.0 || gas_constant_j_per_kg_k <= 0.0 {
        return 0.0;
    }
    (pressure_pa / (gas_constant_j_per_kg_k * temperature_k)).max(0.0)
}

/// Mean gas velocity through a valve from its mass flow, effective open area and
/// the upstream density. Returns 0 when the valve is effectively shut.
fn valve_gas_velocity_m_per_s(
    mass_flow_kg_per_s: f64,
    area_m2: f64,
    density_kg_per_m3: f64,
) -> f64 {
    if area_m2 <= 1.0e-9 || density_kg_per_m3 <= 0.0 {
        return 0.0;
    }
    mass_flow_kg_per_s / (density_kg_per_m3 * area_m2)
}

fn fuel_mass_for_lambda(definition: &EngineDefinition, lambda_target: f64) -> f64 {
    definition.combustion.fuel_mass_per_cycle_kg / lambda_target.clamp(0.1, 2.0)
}

fn mean_effective_pressure_kpa(torque_nm: f64, definition: &EngineDefinition) -> f64 {
    let displacement_m3 = cylinder_displacement_m3(definition);
    if displacement_m3 <= 0.0 {
        return 0.0;
    }

    4.0 * std::f64::consts::PI * torque_nm / displacement_m3 / 1000.0
}

fn volumetric_efficiency_percent(
    intake_mass_flow_kg_per_s: f64,
    rpm: f64,
    definition: &EngineDefinition,
) -> f64 {
    let cycles_per_second = rpm.max(0.0) / 120.0;
    let displacement_m3 = cylinder_displacement_m3(definition);
    let intake_air_density_kg_per_m3 = definition.boundaries.intake_pressure_pa
        / (definition.gas.gas_constant_j_per_kg_k * definition.boundaries.intake_temperature_k);
    let ideal_air_flow_kg_per_s =
        displacement_m3 * intake_air_density_kg_per_m3 * cycles_per_second;
    if ideal_air_flow_kg_per_s <= f64::EPSILON {
        return 0.0;
    }

    intake_mass_flow_kg_per_s / ideal_air_flow_kg_per_s * 100.0
}

fn cylinder_displacement_m3(definition: &EngineDefinition) -> f64 {
    std::f64::consts::PI
        * definition.geometry.bore_m
        * definition.geometry.bore_m
        * 0.25
        * definition.geometry.stroke_m
}

fn angle_trace_index(crank_angle_rad: f64) -> usize {
    let normalized = crank_angle_rad.rem_euclid(ENGINE_CYCLE_RADIANS);
    let fraction = normalized / ENGINE_CYCLE_RADIANS;
    ((fraction * (ANGLE_TRACE_BINS as f64 - 1.0)).round() as usize).min(ANGLE_TRACE_BINS - 1)
}

fn points_from_trace(
    values: &[f64; ANGLE_TRACE_BINS],
    filled: &[bool; ANGLE_TRACE_BINS],
) -> Vec<[f64; 2]> {
    values
        .iter()
        .zip(filled.iter())
        .enumerate()
        .filter_map(|(index, (value, filled))| {
            filled.then_some([
                index as f64 * 720.0 / (ANGLE_TRACE_BINS as f64 - 1.0),
                *value,
            ])
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine_config::EngineDefinition;
    use crate::physics::chamber::ChamberSpeciesMasses;
    use crate::single_cylinder::rad_per_s_to_rpm;

    fn definition() -> EngineDefinition {
        EngineDefinition::from_json_str(include_str!("../data/engines/gn250.json"))
            .expect("GN250 JSON should parse")
    }

    fn sample(
        angle_deg: f64,
        rpm: f64,
        pressure_pa: f64,
        temperature_k: f64,
    ) -> EngineInstantSample {
        sample_with_torque(angle_deg, rpm, pressure_pa, temperature_k, 15.0)
    }

    fn sample_with_torque(
        angle_deg: f64,
        rpm: f64,
        pressure_pa: f64,
        temperature_k: f64,
        torque_nm: f64,
    ) -> EngineInstantSample {
        EngineInstantSample {
            elapsed_time_seconds: angle_deg / 7200.0,
            crank_angle_rad: angle_deg.to_radians(),
            rpm,
            pressure_pa,
            temperature_k,
            chamber_mass_kg: 0.0004,
            intake_effective_area_m2: 0.0001,
            exhaust_effective_area_m2: 0.0002,
            intake_mass_flow_kg_per_s: 0.01,
            exhaust_mass_flow_kg_per_s: 0.0,
            intake_valve_velocity_m_per_s: 30.0,
            exhaust_valve_velocity_m_per_s: 0.0,
            torque_nm,
            power_kw: 5.0,
            map_pa: 101_325.0,
            exhaust_exit_pressure_pa: 102_000.0,
            fuel_flow_mg_per_s: 200.0,
            air_flow_g_per_s: 12.0,
            bmep_kpa: 100.0,
            volumetric_efficiency_percent: 85.0,
            combustion_event: false,
            chamber_lambda: TelemetryScalar::placeholder(1.0),
            exhaust_lambda: TelemetryScalar::unavailable(),
            chamber_charge_air_mass_g: 0.4,
            chamber_charge_fuel_scaled_g: 0.4,
        }
    }

    #[test]
    fn rolling_window_keeps_most_recent_entries() {
        let mut window = RollingWindow::new(2);
        window.push(1);
        window.push(2);
        window.push(3);

        assert_eq!(window.iter().copied().collect::<Vec<_>>(), vec![2, 3]);
    }

    #[test]
    fn cycle_accumulator_tracks_cycle_peaks_and_rpm_delta() {
        let mut cycle = CycleAccumulator::new();

        cycle.push(sample(100.0, 2950.0, 2.0e6, 700.0), 0.001);
        cycle.push(sample(710.0, 3050.0, 3.5e6, 900.0), 0.001);
        cycle.push(sample(5.0, 3000.0, 1.5e6, 650.0), 0.001);

        assert_eq!(cycle.latest_rpm_delta(), 100.0);
        assert_eq!(cycle.latest_peak_pressure_pa(), 3.5e6);
        assert_eq!(cycle.latest_peak_temperature_k(), 900.0);
    }

    #[test]
    fn rpm_curve_uses_fixed_sample_points_and_updates_inside_tolerance() {
        let mut cycle = CycleAccumulator::new();

        cycle.push(sample_with_torque(100.0, 198.0, 2.0e6, 700.0, 10.0), 0.001);
        cycle.push(sample_with_torque(710.0, 202.0, 2.0e6, 700.0, 10.0), 0.001);
        cycle.push(sample_with_torque(5.0, 200.0, 2.0e6, 700.0, 10.0), 0.001);

        assert_eq!(cycle.rpm_history().len(), 1);
        assert_eq!(cycle.rpm_history()[0].rpm, 200.0);
        assert_eq!(cycle.rpm_history()[0].torque_nm, 10.0);

        cycle.push(sample_with_torque(100.0, 250.0, 2.0e6, 700.0, 30.0), 0.001);
        cycle.push(sample_with_torque(710.0, 250.0, 2.0e6, 700.0, 30.0), 0.001);

        assert_eq!(cycle.rpm_history().len(), 1);
        assert_eq!(cycle.rpm_history()[0].torque_nm, 10.0);

        cycle.push(sample_with_torque(100.0, 303.0, 2.0e6, 700.0, 20.0), 0.001);
        cycle.push(sample_with_torque(710.0, 303.0, 2.0e6, 700.0, 20.0), 0.001);
        cycle.push(sample_with_torque(5.0, 303.0, 2.0e6, 700.0, 20.0), 0.001);

        let history = cycle.rpm_history();
        assert_eq!(history.len(), 2);
        assert_eq!(history[0].rpm, 200.0);
        assert_eq!(history[1].rpm, 300.0);
        assert_eq!(history[1].torque_nm, 20.0);
        assert_eq!(history[1].power_kw, 20.0 * rpm_to_rad_per_s(300.0) / 1000.0);
    }

    #[test]
    fn cycle_accumulator_prefers_last_completed_trace_after_wrap() {
        let mut cycle = CycleAccumulator::new();

        cycle.push(sample(700.0, 3000.0, 2.0e6, 800.0), 0.001);
        cycle.push(sample(10.0, 3000.0, 1.0e6, 600.0), 0.001);
        let trace = cycle.trace_snapshot();

        assert!(
            trace
                .pressure_pa
                .iter()
                .any(|point| point[0] >= 700.0 && point[1] == 2.0e6)
        );
    }

    #[test]
    fn cycle_accumulator_switches_to_slow_live_trace_after_display_interval() {
        let mut cycle = CycleAccumulator::new();

        cycle.push(sample(700.0, 3000.0, 2.0e6, 800.0), 0.001);
        cycle.push(sample(10.0, 3000.0, 1.0e6, 600.0), 0.001);
        cycle.push(
            sample(20.0, 250.0, 1.2e6, 610.0),
            1.0 / ANGLE_PLOT_RATE_HZ + 0.001,
        );
        let trace = cycle.trace_snapshot();

        assert!(
            trace
                .pressure_pa
                .iter()
                .any(|point| point[0] <= 20.0 && point[1] == 1.2e6)
        );
        assert!(
            !trace
                .pressure_pa
                .iter()
                .any(|point| point[0] >= 700.0 && point[1] == 2.0e6)
        );
    }

    #[test]
    fn controls_map_to_documented_step_inputs() {
        let controls = EngineControls {
            starter_enabled: true,
            starter_torque_nm: 25.0,
            starter_speed_limit_rpm: 950.0,
            added_inertia_kg_m2: 0.01,
            added_torque_load_nm: 5.0,
            lambda_target: 1.4,
            spark_enabled: false,
            fuel_enabled: true,
            dyno_mode_enabled: true,
            dyno_target_rpm: 3200.0,
            ..EngineControls::default()
        };
        let definition = definition();
        let inputs = controls.to_step_inputs(&definition, 3200.0);

        assert_eq!(inputs.external_load_torque_nm, 5.0);
        assert_eq!(inputs.starter_torque_nm, 25.0);
        assert_eq!(inputs.starter_speed_limit_rpm, 950.0);
        assert_eq!(inputs.added_inertia_kg_m2, 0.01);
        assert_eq!(inputs.lambda_target, 1.4);
        assert_eq!(inputs.idle_throttle_fraction, 0.0);
        assert!(inputs.throttle_effective_area_fraction.is_none());
        assert!(!inputs.spark_enabled);
        assert!(inputs.fuel_enabled);
        assert_eq!(
            rad_per_s_to_rpm(
                inputs
                    .fixed_crank_speed_rad_per_s
                    .expect("dyno mode should force RPM")
            ),
            3200.0
        );
    }

    #[test]
    fn controls_clamp_lambda_and_dyno_floor_for_simulation_inputs() {
        let low = EngineControls {
            lambda_target: 0.01,
            dyno_mode_enabled: true,
            dyno_target_rpm: 0.0,
            ..EngineControls::default()
        }
        .to_step_inputs(&definition(), 0.0);
        assert_eq!(low.lambda_target, 0.1);
        assert_eq!(
            rad_per_s_to_rpm(
                low.fixed_crank_speed_rad_per_s
                    .expect("dyno mode should force RPM")
            ),
            1.0
        );

        let high = EngineControls {
            lambda_target: 3.0,
            ..EngineControls::default()
        }
        .to_step_inputs(&definition(), 0.0);
        assert_eq!(high.lambda_target, 2.0);
    }

    #[test]
    fn controls_map_override_maps_to_forced_map_pressure() {
        let definition = definition();

        let off = EngineControls {
            map_override_enabled: false,
            map_override_pa: 150_000.0,
            ..EngineControls::default()
        }
        .to_step_inputs(&definition, 3000.0);
        assert!(off.forced_map_pressure_pa.is_none());

        let on = EngineControls {
            map_override_enabled: true,
            map_override_pa: 150_000.0,
            ..EngineControls::default()
        }
        .to_step_inputs(&definition, 3000.0);
        assert_eq!(on.forced_map_pressure_pa, Some(150_000.0));

        // Clamps to the documented 2000 kPa ceiling and a positive floor.
        let clamped = EngineControls {
            map_override_enabled: true,
            map_override_pa: 5_000_000.0,
            ..EngineControls::default()
        }
        .to_step_inputs(&definition, 3000.0);
        assert_eq!(clamped.forced_map_pressure_pa, Some(MAP_OVERRIDE_MAX_PA));
    }

    #[test]
    fn controls_combine_throttle_and_idle_throttle_into_effective_area_command() {
        let definition = definition();
        let closed = EngineControls {
            throttle_position: 0.0,
            idle_throttle_fraction: 0.0,
            ..EngineControls::default()
        };
        let idle = EngineControls {
            throttle_position: 0.0,
            idle_throttle_fraction: 1.0,
            ..EngineControls::default()
        };
        let part_throttle = EngineControls {
            throttle_position: 0.5,
            idle_throttle_fraction: 1.0,
            ..EngineControls::default()
        };

        assert_eq!(closed.throttle_effective_area_fraction(&definition), 0.0);
        assert!(idle.throttle_effective_area_fraction(&definition) > 0.0);
        assert!(
            part_throttle.throttle_effective_area_fraction(&definition)
                > idle.throttle_effective_area_fraction(&definition)
        );
        assert!(
            part_throttle
                .to_step_inputs(&definition, 1000.0)
                .throttle_effective_area_fraction
                .is_none()
        );
    }

    #[test]
    fn configured_idle_throttle_area_defaults_to_parallel_ten_percent_valve() {
        let mut definition = definition();
        definition.intake_exhaust.throttle_maximum_area_m2 = 0.0005;
        definition.intake_exhaust.idle_throttle_maximum_area_m2 = None;
        let idle_only = EngineControls {
            throttle_position: 0.0,
            idle_throttle_fraction: 1.0,
            ..EngineControls::default()
        };

        let actual = idle_only.throttle_effective_area_fraction(&definition);
        let expected = 0.00005 / 0.00055;
        assert!((actual - expected).abs() < 1.0e-12);
    }

    #[test]
    fn redline_cut_timer_can_hold_spark_off_below_cut_rpm() {
        let definition = definition();
        let controls = EngineControls {
            spark_enabled: true,
            redline_spark_cut_rpm: 8000.0,
            ..EngineControls::default()
        };

        assert!(
            controls
                .to_step_inputs_with_redline_cut_active(&definition, 4000.0, false)
                .spark_enabled
        );
        assert!(
            !controls
                .to_step_inputs_with_redline_cut_active(&definition, 4000.0, true)
                .spark_enabled
        );
    }

    #[test]
    fn telemetry_aggregator_publishes_bounded_time_history() {
        let definition = definition();
        let controls = EngineControls::default();
        let mut aggregator = TelemetryAggregator::new(definition, controls);

        for index in 0..400 {
            let mut output = crate::single_cylinder::SingleCylinderStepOutput {
                elapsed_time_seconds: index as f64 * 0.1,
                crank_angle_rad: (index as f64).to_radians(),
                crank_speed_rad_per_s: rpm_to_rad_per_s(3000.0),
                rpm: 3000.0,
                cylinder_volume_m3: 0.0002,
                volume_rate_m3_per_s: 0.0,
                cylinder_pressure_pa: 1.0e6,
                cylinder_temperature_k: 700.0,
                chamber_mass_kg: 0.0004,
                chamber_oxygen_mass_kg: 0.0000928,
                chamber_fuel_mass_kg: 0.0,
                chamber_inert_mass_kg: 0.0003072,
                chamber_products_mass_kg: 0.0,
                chamber_lambda: None,
                intake_plenum_pressure_pa: 101_325.0,
                intake_runner_pressure_pa: 101_325.0,
                exhaust_runner_pressure_pa: 101_325.0,
                exhaust_collector_pressure_pa: None,
                exhaust_exit_pressure_pa: 101_325.0 + index as f64,
                intake_effective_area_m2: 0.0001,
                exhaust_effective_area_m2: 0.0002,
                intake_mass_flow_kg_per_s: 0.01,
                exhaust_mass_flow_kg_per_s: 0.0,
                intake_species_flow_kg_per_s: ChamberSpeciesMasses::from_dry_air_mass(0.01),
                exhaust_species_flow_kg_per_s: ChamberSpeciesMasses::default(),
                fuel_injected_kg: 0.0,
                fuel_burned_kg: 0.0,
                oxygen_consumed_kg: 0.0,
                products_generated_kg: 0.0,
                species_budget: crate::single_cylinder::EngineSpeciesBudget::default(),
                exhaust_lambda: None,
                gas_force_n: 0.0,
                indicated_torque_nm: 20.0,
                load_torque_nm: 0.0,
                combustion_heat_added_j: 0.0,
                indicated_work_j: 0.0,
                mean_indicated_torque_nm: 20.0,
            };
            output.crank_angle_rad = (index as f64).to_radians().rem_euclid(ENGINE_CYCLE_RADIANS);
            aggregator.ingest_step_output(output, 1.0 / 30.0);
            let _ = aggregator.publish_ready();
        }

        let snapshot = aggregator.snapshot();
        assert!(snapshot.time_history.len() <= TIME_HISTORY_CAPACITY);
        assert!(
            snapshot
                .time_history
                .iter()
                .any(|point| point.exhaust_exit_pressure_pa > 101_325.0)
        );
    }

    #[test]
    fn telemetry_aggregator_accumulates_cycle_data_between_publishes() {
        let definition = definition();
        let controls = EngineControls::default();
        let mut aggregator = TelemetryAggregator::new(definition, controls);

        for angle_deg in (0..720).step_by(4) {
            let rpm = if angle_deg < 360 { 3000.0 } else { 3075.0 };
            aggregator.ingest_step_output(step_output(angle_deg as f64, rpm), 0.0001);
        }
        aggregator.ingest_step_output(step_output(2.0, 3020.0), 0.0001);

        let snapshot = aggregator.snapshot();
        assert_eq!(snapshot.engine_data.rpm_delta, 75.0);
        assert!(snapshot.angle_trace.pressure_pa.len() > 120);
        assert!(snapshot.angle_trace.exhaust_exit_pressure_pa.len() > 120);
        assert!(
            snapshot
                .angle_trace
                .pressure_pa
                .iter()
                .all(|point| point[0] >= 0.0 && point[0] <= 720.0)
        );
    }

    #[test]
    fn engine_panel_uses_rolling_average_and_preserves_measured_lambda() {
        let definition = definition();
        let controls = EngineControls::default();
        let mut aggregator = TelemetryAggregator::new(definition, controls);
        let mut first = step_output(10.0, 1000.0);
        first.chamber_lambda = Some(0.9);
        let mut second = step_output(20.0, 2000.0);
        second.chamber_lambda = Some(1.1);

        aggregator.ingest_step_output(first, 1.0 / 30.0);
        assert!(aggregator.publish_ready().is_some());
        aggregator.ingest_step_output(second, 1.0 / 30.0);
        let frame = aggregator
            .publish_ready()
            .expect("second sample should publish engine panel");

        assert_eq!(aggregator.published_engine_frames.len(), 1);
        assert_eq!(frame.engine_data.rpm, 1500.0);
        assert_eq!(
            frame.engine_data.chamber_lambda.availability,
            TelemetryAvailability::Measured
        );
        assert_eq!(frame.engine_data.chamber_lambda.value, Some(1.0));
    }

    #[test]
    fn telemetry_reports_placeholder_and_unavailable_scalar_status() {
        let definition = definition();
        let controls = EngineControls {
            lambda_target: 1.3,
            ..EngineControls::default()
        };
        let mut aggregator = TelemetryAggregator::new(definition, controls);

        aggregator.ingest_step_output(step_output(10.0, 1000.0), 1.0 / 30.0);
        let frame = aggregator
            .publish_ready()
            .expect("time plot cadence should publish a frame");

        assert_eq!(
            frame.engine_data.chamber_lambda,
            TelemetryScalar::placeholder(1.3)
        );
        assert_eq!(
            frame.engine_data.exhaust_lambda.availability,
            TelemetryAvailability::Unavailable
        );
    }

    #[test]
    fn instant_sample_preserves_intake_backflow_sign_for_air_flow() {
        let definition = definition();
        let controls = EngineControls::default();
        let mut output = step_output(390.0, 3000.0);
        output.intake_mass_flow_kg_per_s = -0.012;

        let sample = EngineInstantSample::from_step_output(output, controls, &definition);

        assert_eq!(sample.air_flow_g_per_s, -12.0);
        assert!(
            sample.volumetric_efficiency_percent < 0.0,
            "VE should expose net reverse intake flow instead of silently clamping it"
        );
    }

    #[test]
    fn telemetry_uses_separate_engine_panel_and_time_plot_cadences() {
        let definition = definition();
        let controls = EngineControls::default();
        let mut aggregator = TelemetryAggregator::new(definition, controls);

        aggregator.ingest_step_output(step_output(10.0, 1000.0), 1.0 / 30.0);
        let first = aggregator
            .publish_ready()
            .expect("time plot cadence should publish first");
        assert_eq!(first.time_history.len(), 1);
        assert_eq!(aggregator.published_engine_frames.len(), 0);

        aggregator.ingest_step_output(step_output(20.0, 2000.0), 1.0 / 30.0);
        let second = aggregator
            .publish_ready()
            .expect("engine panel cadence should publish second");
        assert_eq!(second.time_history.len(), 2);
        assert_eq!(aggregator.published_engine_frames.len(), 1);
    }

    fn step_output(angle_deg: f64, rpm: f64) -> crate::single_cylinder::SingleCylinderStepOutput {
        crate::single_cylinder::SingleCylinderStepOutput {
            elapsed_time_seconds: angle_deg / 7200.0,
            crank_angle_rad: angle_deg.to_radians().rem_euclid(ENGINE_CYCLE_RADIANS),
            crank_speed_rad_per_s: rpm_to_rad_per_s(rpm),
            rpm,
            cylinder_volume_m3: 0.0002,
            volume_rate_m3_per_s: 0.0,
            cylinder_pressure_pa: 1.0e5 + angle_deg * 1000.0,
            cylinder_temperature_k: 300.0 + angle_deg,
            chamber_mass_kg: 0.0004,
            chamber_oxygen_mass_kg: 0.0000928,
            chamber_fuel_mass_kg: 0.0,
            chamber_inert_mass_kg: 0.0003072,
            chamber_products_mass_kg: 0.0,
            chamber_lambda: None,
            intake_plenum_pressure_pa: 101_325.0,
            intake_runner_pressure_pa: 101_325.0,
            exhaust_runner_pressure_pa: 101_325.0,
            exhaust_collector_pressure_pa: None,
            exhaust_exit_pressure_pa: 101_325.0 + angle_deg,
            intake_effective_area_m2: 0.0001,
            exhaust_effective_area_m2: 0.0002,
            intake_mass_flow_kg_per_s: 0.01,
            exhaust_mass_flow_kg_per_s: 0.0,
            intake_species_flow_kg_per_s: ChamberSpeciesMasses::from_dry_air_mass(0.01),
            exhaust_species_flow_kg_per_s: ChamberSpeciesMasses::default(),
            fuel_injected_kg: 0.0,
            fuel_burned_kg: 0.0,
            oxygen_consumed_kg: 0.0,
            products_generated_kg: 0.0,
            species_budget: crate::single_cylinder::EngineSpeciesBudget::default(),
            exhaust_lambda: None,
            gas_force_n: 0.0,
            indicated_torque_nm: 20.0,
            load_torque_nm: 0.0,
            combustion_heat_added_j: 0.0,
            indicated_work_j: 0.0,
            mean_indicated_torque_nm: 20.0,
        }
    }
}
