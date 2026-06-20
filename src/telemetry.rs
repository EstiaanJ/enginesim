use std::collections::VecDeque;

use crate::engine_config::EngineDefinition;
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
    pub idle_leak_fraction: f64,
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
}

impl Default for EngineControls {
    fn default() -> Self {
        Self {
            throttle_position: 1.0,
            idle_leak_fraction: 0.0,
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
        }
    }
}

impl EngineControls {
    pub fn to_step_inputs(self, current_rpm: f64) -> SingleCylinderStepInputs {
        let spark_enabled = self.spark_enabled && current_rpm < self.redline_spark_cut_rpm;
        SingleCylinderStepInputs {
            external_load_torque_nm: self.added_torque_load_nm.max(0.0),
            fixed_crank_speed_rad_per_s: self
                .dyno_mode_enabled
                .then(|| rpm_to_rad_per_s(self.dyno_target_rpm.max(0.0))),
            starter_torque_nm: self.starter_torque_nm.max(0.0),
            starter_speed_limit_rpm: self.starter_speed_limit_rpm.max(0.0),
            starter_enabled: self.starter_enabled,
            added_inertia_kg_m2: self.added_inertia_kg_m2.max(0.0),
            spark_enabled,
            fuel_enabled: self.fuel_enabled,
        }
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
    pub torque_nm: f64,
    pub power_kw: f64,
    pub map_pa: f64,
    pub fuel_flow_mg_per_s: f64,
    pub air_flow_g_per_s: f64,
    pub combustion_event: bool,
    pub chamber_lambda: TelemetryScalar,
    pub exhaust_lambda: TelemetryScalar,
    pub fuel_mass_placeholder_mg: TelemetryScalar,
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
            definition.combustion.fuel_mass_per_cycle_kg * output.rpm / 120.0 * 1.0e6
        } else {
            0.0
        };

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
            torque_nm: output.indicated_torque_nm,
            power_kw,
            map_pa: definition.boundaries.intake_pressure_pa,
            fuel_flow_mg_per_s,
            air_flow_g_per_s: output.intake_mass_flow_kg_per_s.max(0.0) * 1000.0,
            combustion_event: output.combustion_heat_added_j > 0.0,
            chamber_lambda: TelemetryScalar::placeholder(controls.lambda_target),
            exhaust_lambda: TelemetryScalar::unavailable(),
            fuel_mass_placeholder_mg: TelemetryScalar::placeholder(
                definition.combustion.fuel_mass_per_cycle_kg * 1.0e6,
            ),
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
    pub chamber_lambda: TelemetryScalar,
    pub exhaust_lambda: TelemetryScalar,
    pub peak_cylinder_temperature_k: f64,
    pub peak_cylinder_pressure_pa: f64,
    pub map_pa: f64,
    pub combustion_event_ratio: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct AngleTraceSnapshot {
    pub pressure_pa: Vec<[f64; 2]>,
    pub temperature_k: Vec<[f64; 2]>,
    pub intake_effective_area_m2: Vec<[f64; 2]>,
    pub exhaust_effective_area_m2: Vec<[f64; 2]>,
    pub chamber_air_mass_g: Vec<[f64; 2]>,
    pub fuel_mass_placeholder_mg: Vec<[f64; 2]>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TimePlotPoint {
    pub time_seconds: f64,
    pub rpm: f64,
    pub map_pa: f64,
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
    map_pa: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct PendingFrameAccumulator {
    duration_seconds: f64,
    rpm_weighted_sum: f64,
    torque_weighted_sum: f64,
    power_weighted_sum: f64,
    fuel_flow_weighted_sum: f64,
    air_flow_weighted_sum: f64,
    map_weighted_sum: f64,
}

impl PendingFrameAccumulator {
    fn push(&mut self, sample: EngineInstantSample, sample_duration_seconds: f64) {
        self.duration_seconds += sample_duration_seconds;
        self.rpm_weighted_sum += sample.rpm * sample_duration_seconds;
        self.torque_weighted_sum += sample.torque_nm * sample_duration_seconds;
        self.power_weighted_sum += sample.power_kw * sample_duration_seconds;
        self.fuel_flow_weighted_sum += sample.fuel_flow_mg_per_s * sample_duration_seconds;
        self.air_flow_weighted_sum += sample.air_flow_g_per_s * sample_duration_seconds;
        self.map_weighted_sum += sample.map_pa * sample_duration_seconds;
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
            map_pa: self.map_weighted_sum / self.duration_seconds,
        })
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct CycleAccumulator {
    current_rpm_min: f64,
    current_rpm_max: f64,
    current_peak_pressure_pa: f64,
    current_peak_temperature_k: f64,
    current_combustion_event_seen: bool,
    current_trace: AngleTraceBuffers,
    last_completed_trace: Option<AngleTraceBuffers>,
    completed_cycles: RollingWindow<CycleSummary>,
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
struct AngleTraceBuffers {
    pressure_pa: [f64; ANGLE_TRACE_BINS],
    temperature_k: [f64; ANGLE_TRACE_BINS],
    intake_effective_area_m2: [f64; ANGLE_TRACE_BINS],
    exhaust_effective_area_m2: [f64; ANGLE_TRACE_BINS],
    chamber_air_mass_g: [f64; ANGLE_TRACE_BINS],
    fuel_mass_placeholder_mg: [f64; ANGLE_TRACE_BINS],
    filled: [bool; ANGLE_TRACE_BINS],
}

impl Default for AngleTraceBuffers {
    fn default() -> Self {
        Self {
            pressure_pa: [0.0; ANGLE_TRACE_BINS],
            temperature_k: [0.0; ANGLE_TRACE_BINS],
            intake_effective_area_m2: [0.0; ANGLE_TRACE_BINS],
            exhaust_effective_area_m2: [0.0; ANGLE_TRACE_BINS],
            chamber_air_mass_g: [0.0; ANGLE_TRACE_BINS],
            fuel_mass_placeholder_mg: [0.0; ANGLE_TRACE_BINS],
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
        self.chamber_air_mass_g[index] = sample.chamber_mass_kg * 1000.0;
        self.fuel_mass_placeholder_mg[index] = sample.fuel_mass_placeholder_mg.value.unwrap_or(0.0);
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
            chamber_air_mass_g: points_from_trace(&self.chamber_air_mass_g, &self.filled),
            fuel_mass_placeholder_mg: points_from_trace(
                &self.fuel_mass_placeholder_mg,
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
            current_peak_pressure_pa: 0.0,
            current_peak_temperature_k: 0.0,
            current_combustion_event_seen: false,
            current_trace: AngleTraceBuffers::default(),
            last_completed_trace: None,
            completed_cycles: RollingWindow::new(CYCLE_HISTORY_CAPACITY),
            previous_angle_rad: None,
        }
    }

    fn push(&mut self, sample: EngineInstantSample, frame: EngineDataFrame) {
        let wrapped = self
            .previous_angle_rad
            .is_some_and(|previous| sample.crank_angle_rad < previous);
        if wrapped {
            self.complete_cycle(frame);
        }

        self.current_rpm_min = self.current_rpm_min.min(sample.rpm);
        self.current_rpm_max = self.current_rpm_max.max(sample.rpm);
        self.current_peak_pressure_pa = self.current_peak_pressure_pa.max(sample.pressure_pa);
        self.current_peak_temperature_k = self.current_peak_temperature_k.max(sample.temperature_k);
        self.current_combustion_event_seen |= sample.combustion_event;
        self.current_trace.record(sample);
        self.previous_angle_rad = Some(sample.crank_angle_rad);
    }

    fn complete_cycle(&mut self, frame: EngineDataFrame) {
        let summary = CycleSummary {
            rpm_delta: (self.current_rpm_max - self.current_rpm_min).max(0.0),
            peak_pressure_pa: self.current_peak_pressure_pa,
            peak_temperature_k: self.current_peak_temperature_k,
            combustion_event_ratio: if self.current_combustion_event_seen {
                1.0
            } else {
                0.0
            },
            mean_rpm: frame.rpm,
            mean_torque_nm: frame.torque_nm,
            mean_power_kw: frame.power_kw,
        };
        self.completed_cycles.push(summary);
        self.last_completed_trace = Some(self.current_trace.clone());
        self.current_rpm_min = f64::INFINITY;
        self.current_rpm_max = f64::NEG_INFINITY;
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
        self.last_completed_trace
            .as_ref()
            .unwrap_or(&self.current_trace)
            .to_snapshot()
    }

    fn rpm_history(&self) -> Vec<RpmPlotPoint> {
        self.completed_cycles
            .iter()
            .enumerate()
            .map(|(index, cycle)| RpmPlotPoint {
                age_index: self.completed_cycles.len() - index - 1,
                rpm: cycle.mean_rpm,
                torque_nm: cycle.mean_torque_nm,
                power_kw: cycle.mean_power_kw,
            })
            .collect()
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
            pending_engine_panel: PendingFrameAccumulator {
                duration_seconds: 0.0,
                rpm_weighted_sum: 0.0,
                torque_weighted_sum: 0.0,
                power_weighted_sum: 0.0,
                fuel_flow_weighted_sum: 0.0,
                air_flow_weighted_sum: 0.0,
                map_weighted_sum: 0.0,
            },
            pending_time_plot: PendingFrameAccumulator {
                duration_seconds: 0.0,
                rpm_weighted_sum: 0.0,
                torque_weighted_sum: 0.0,
                power_weighted_sum: 0.0,
                fuel_flow_weighted_sum: 0.0,
                air_flow_weighted_sum: 0.0,
                map_weighted_sum: 0.0,
            },
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
                self.cycle_accumulator.push(latest_sample, frame);
                self.published_engine_frames.push(frame);
            }
            self.pending_engine_panel = PendingFrameAccumulator {
                duration_seconds: 0.0,
                rpm_weighted_sum: 0.0,
                torque_weighted_sum: 0.0,
                power_weighted_sum: 0.0,
                fuel_flow_weighted_sum: 0.0,
                air_flow_weighted_sum: 0.0,
                map_weighted_sum: 0.0,
            };
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
                    torque_nm: frame.torque_nm,
                });
            }
            self.pending_time_plot = PendingFrameAccumulator {
                duration_seconds: 0.0,
                rpm_weighted_sum: 0.0,
                torque_weighted_sum: 0.0,
                power_weighted_sum: 0.0,
                fuel_flow_weighted_sum: 0.0,
                air_flow_weighted_sum: 0.0,
                map_weighted_sum: 0.0,
            };
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
                chamber_lambda: TelemetryScalar::placeholder(self.controls.lambda_target),
                exhaust_lambda: TelemetryScalar::unavailable(),
                peak_cylinder_temperature_k: self.cycle_accumulator.latest_peak_temperature_k(),
                peak_cylinder_pressure_pa: self.cycle_accumulator.latest_peak_pressure_pa(),
                map_pa: averaged_frame.map_pa,
                combustion_event_ratio: self.cycle_accumulator.combustion_event_ratio(),
            },
            angle_trace: self.cycle_accumulator.trace_snapshot(),
            time_history: self.time_history.iter().copied().collect(),
            rpm_history: self.cycle_accumulator.rpm_history(),
        }
    }
}

pub fn default_profile_for_gui(definition: &EngineDefinition) -> SimulationProfile {
    SimulationProfile {
        timestep_seconds: definition.simulation.timestep_seconds,
        ..SimulationProfile::real_time()
    }
}

fn average_engine_data_frames(frames: &RollingWindow<EngineDataFrame>) -> EngineDataFrame {
    if frames.is_empty() {
        return EngineDataFrame {
            rpm: 0.0,
            torque_nm: 0.0,
            power_kw: 0.0,
            fuel_flow_mg_per_s: 0.0,
            air_flow_g_per_s: 0.0,
            map_pa: 0.0,
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
        map_pa: frames.iter().map(|frame| frame.map_pa).sum::<f64>() / count,
    }
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
            torque_nm: 15.0,
            power_kw: 5.0,
            map_pa: 101_325.0,
            fuel_flow_mg_per_s: 200.0,
            air_flow_g_per_s: 12.0,
            combustion_event: false,
            chamber_lambda: TelemetryScalar::placeholder(1.0),
            exhaust_lambda: TelemetryScalar::unavailable(),
            fuel_mass_placeholder_mg: TelemetryScalar::placeholder(12.0),
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
        let frame = EngineDataFrame {
            rpm: 3000.0,
            torque_nm: 20.0,
            power_kw: 6.0,
            fuel_flow_mg_per_s: 200.0,
            air_flow_g_per_s: 10.0,
            map_pa: 101_325.0,
        };

        cycle.push(sample(100.0, 2950.0, 2.0e6, 700.0), frame);
        cycle.push(sample(710.0, 3050.0, 3.5e6, 900.0), frame);
        cycle.push(sample(5.0, 3000.0, 1.5e6, 650.0), frame);

        assert_eq!(cycle.latest_rpm_delta(), 100.0);
        assert_eq!(cycle.latest_peak_pressure_pa(), 3.5e6);
        assert_eq!(cycle.latest_peak_temperature_k(), 900.0);
    }

    #[test]
    fn cycle_accumulator_prefers_last_completed_trace_after_wrap() {
        let mut cycle = CycleAccumulator::new();
        let frame = EngineDataFrame {
            rpm: 3000.0,
            torque_nm: 20.0,
            power_kw: 6.0,
            fuel_flow_mg_per_s: 200.0,
            air_flow_g_per_s: 10.0,
            map_pa: 101_325.0,
        };

        cycle.push(sample(700.0, 3000.0, 2.0e6, 800.0), frame);
        cycle.push(sample(10.0, 3000.0, 1.0e6, 600.0), frame);
        let trace = cycle.trace_snapshot();

        assert!(
            trace
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
            spark_enabled: false,
            fuel_enabled: true,
            dyno_mode_enabled: true,
            dyno_target_rpm: 3200.0,
            ..EngineControls::default()
        };
        let inputs = controls.to_step_inputs(3200.0);

        assert_eq!(inputs.external_load_torque_nm, 5.0);
        assert_eq!(inputs.starter_torque_nm, 25.0);
        assert_eq!(inputs.starter_speed_limit_rpm, 950.0);
        assert_eq!(inputs.added_inertia_kg_m2, 0.01);
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
                intake_effective_area_m2: 0.0001,
                exhaust_effective_area_m2: 0.0002,
                intake_mass_flow_kg_per_s: 0.01,
                exhaust_mass_flow_kg_per_s: 0.0,
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
    }
}
