use crate::chamber::{
    ChamberBoundary, ChamberProperties, ChamberState, FlowBoundary, chamber_derivatives,
    chamber_pressure_pa, step_euler,
};
use crate::combustion::cumulative_wiebe_burned_fraction;
use crate::engine_config::{EngineDefinition, SparkTimingDefinition, ValveDefinition};
use crate::engine_geometry::{
    bore_area_m2, clearance_volume_m3, crank_radius_m, slider_crank_cylinder_volume_m3,
    slider_crank_dx_dtheta_m_per_rad, slider_crank_volume_rate_m3_per_s,
};
use crate::profiles::SimulationProfile;
use crate::simulation::{StepContext, StepModel};
use crate::valve::{
    ENGINE_CYCLE_RADIANS, ValveEvent, crossed_cycle_angle_rad, normalize_cycle_angle_rad,
    positive_cycle_delta_rad,
};

#[derive(Debug, Clone)]
pub struct SingleCylinderEngine {
    definition: EngineDefinition,
    profile: SimulationProfile,
    chamber_state: ChamberState,
    crank_angle_rad: f64,
    crank_speed_rad_per_s: f64,
    active_combustion: Option<ActiveCombustion>,
    accumulated_indicated_work_j: f64,
    accumulated_crank_angle_rad: f64,
    elapsed_time_seconds: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct ActiveCombustion {
    start_angle_rad: f64,
    burned_fraction: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SingleCylinderStepInputs {
    pub external_load_torque_nm: f64,
    pub fixed_crank_speed_rad_per_s: Option<f64>,
    pub starter_torque_nm: f64,
    pub starter_speed_limit_rpm: f64,
    pub starter_enabled: bool,
    pub added_inertia_kg_m2: f64,
    pub spark_enabled: bool,
    pub fuel_enabled: bool,
}

impl Default for SingleCylinderStepInputs {
    fn default() -> Self {
        Self {
            external_load_torque_nm: 0.0,
            fixed_crank_speed_rad_per_s: None,
            starter_torque_nm: 0.0,
            starter_speed_limit_rpm: 0.0,
            starter_enabled: false,
            added_inertia_kg_m2: 0.0,
            spark_enabled: true,
            fuel_enabled: true,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SingleCylinderStepOutput {
    pub elapsed_time_seconds: f64,
    pub crank_angle_rad: f64,
    pub crank_speed_rad_per_s: f64,
    pub rpm: f64,
    pub cylinder_volume_m3: f64,
    pub volume_rate_m3_per_s: f64,
    pub cylinder_pressure_pa: f64,
    pub cylinder_temperature_k: f64,
    pub chamber_mass_kg: f64,
    pub intake_effective_area_m2: f64,
    pub exhaust_effective_area_m2: f64,
    pub intake_mass_flow_kg_per_s: f64,
    pub exhaust_mass_flow_kg_per_s: f64,
    pub gas_force_n: f64,
    pub indicated_torque_nm: f64,
    pub load_torque_nm: f64,
    pub combustion_heat_added_j: f64,
    pub indicated_work_j: f64,
    pub mean_indicated_torque_nm: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FixedSpeedRunSummary {
    pub rpm: f64,
    pub cycles: usize,
    pub steps: usize,
    pub integrated_angle_rad: f64,
    pub elapsed_time_seconds: f64,
    pub indicated_work_j: f64,
    pub mean_indicated_torque_nm: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TorqueSweepPoint {
    pub rpm: f64,
    pub mean_indicated_torque_nm: f64,
    pub indicated_power_kw: f64,
}

impl SingleCylinderEngine {
    pub fn from_definition(definition: EngineDefinition) -> Self {
        let profile = profile_from_definition(&definition);
        Self::from_definition_with_profile(definition, profile)
    }

    pub fn from_definition_with_profile(
        definition: EngineDefinition,
        profile: SimulationProfile,
    ) -> Self {
        profile.validate();
        let crank_angle_rad = definition.crank.initial_crank_angle_deg.to_radians();
        let crank_speed_rad_per_s = rpm_to_rad_per_s(definition.crank.initial_speed_rpm);
        let properties = chamber_properties(&definition);
        let volume_m3 = cylinder_volume_m3(&definition, crank_angle_rad);
        let chamber_state = chamber_state_at_pressure(
            definition.boundaries.intake_pressure_pa,
            definition.boundaries.intake_temperature_k,
            volume_m3,
            properties,
        );

        Self {
            definition,
            profile,
            chamber_state,
            crank_angle_rad: normalize_cycle_angle_rad(crank_angle_rad),
            crank_speed_rad_per_s,
            active_combustion: None,
            accumulated_indicated_work_j: 0.0,
            accumulated_crank_angle_rad: 0.0,
            elapsed_time_seconds: 0.0,
        }
    }

    pub fn definition(&self) -> &EngineDefinition {
        &self.definition
    }

    pub fn profile(&self) -> SimulationProfile {
        self.profile
    }

    pub fn chamber_state(&self) -> ChamberState {
        self.chamber_state
    }

    pub fn crank_angle_rad(&self) -> f64 {
        self.crank_angle_rad
    }

    pub fn crank_speed_rad_per_s(&self) -> f64 {
        self.crank_speed_rad_per_s
    }

    pub fn accumulated_indicated_work_j(&self) -> f64 {
        self.accumulated_indicated_work_j
    }

    pub fn mean_indicated_torque_nm(&self) -> f64 {
        if self.accumulated_crank_angle_rad <= 0.0 {
            return 0.0;
        }

        self.accumulated_indicated_work_j / self.accumulated_crank_angle_rad
    }

    pub fn reset_accumulators(&mut self) {
        self.accumulated_indicated_work_j = 0.0;
        self.accumulated_crank_angle_rad = 0.0;
    }

    pub fn step(&mut self, inputs: SingleCylinderStepInputs) -> SingleCylinderStepOutput {
        self.step_with_timestep(self.profile.timestep_seconds, inputs)
    }

    fn step_with_timestep(
        &mut self,
        timestep_seconds: f64,
        inputs: SingleCylinderStepInputs,
    ) -> SingleCylinderStepOutput {
        assert!(timestep_seconds > 0.0, "timestep must be positive");

        let crank_speed_rad_per_s = inputs
            .fixed_crank_speed_rad_per_s
            .unwrap_or(self.crank_speed_rad_per_s);
        assert!(
            crank_speed_rad_per_s >= 0.0,
            "single-cylinder vertical slice expects non-negative crank speed"
        );
        assert!(
            inputs.added_inertia_kg_m2 >= 0.0,
            "added inertia must be non-negative"
        );

        let start_angle_rad = self.crank_angle_rad;
        let delta_angle_rad = crank_speed_rad_per_s * timestep_seconds;
        let end_angle_rad = normalize_cycle_angle_rad(start_angle_rad + delta_angle_rad);
        let start_volume_m3 = cylinder_volume_m3(&self.definition, start_angle_rad);
        let volume_rate_m3_per_s =
            cylinder_volume_rate_m3_per_s(&self.definition, start_angle_rad, crank_speed_rad_per_s);
        let start_pressure_pa = chamber_pressure_pa(
            self.chamber_state,
            ChamberBoundary {
                volume_m3: start_volume_m3,
                volume_rate_m3_per_s,
                heat_rate_w: 0.0,
            },
            chamber_properties(&self.definition),
        );

        let intake_valve = ValveEvent::from_definition(self.definition.valves.intake);
        let exhaust_valve = ValveEvent::from_definition(self.definition.valves.exhaust);
        let intake_area_m2 = intake_valve.effective_area_m2(start_angle_rad);
        let exhaust_area_m2 = exhaust_valve.effective_area_m2(start_angle_rad);
        let inlet = inlet_boundary(
            self.definition.valves.intake,
            intake_area_m2,
            &self.definition,
        );
        let outlet = outlet_boundary(
            self.definition.valves.exhaust,
            exhaust_area_m2,
            &self.definition,
        );
        let combustion_heat_added_j = self.combustion_heat_added_j(
            start_angle_rad,
            end_angle_rad,
            crank_speed_rad_per_s,
            inputs.spark_enabled && inputs.fuel_enabled,
        );
        let boundary = ChamberBoundary {
            volume_m3: start_volume_m3,
            volume_rate_m3_per_s,
            heat_rate_w: combustion_heat_added_j / timestep_seconds,
        };
        let properties = chamber_properties(&self.definition);
        let derivatives =
            chamber_derivatives(self.chamber_state, boundary, inlet, outlet, properties);

        self.chamber_state = step_euler(
            self.chamber_state,
            boundary,
            inlet,
            outlet,
            properties,
            timestep_seconds,
        );

        let end_volume_m3 = cylinder_volume_m3(&self.definition, end_angle_rad);
        let end_boundary = ChamberBoundary {
            volume_m3: end_volume_m3,
            volume_rate_m3_per_s: 0.0,
            heat_rate_w: 0.0,
        };
        let end_pressure_pa = chamber_pressure_pa(self.chamber_state, end_boundary, properties);
        let gas_force_n = gas_force_n(&self.definition, start_pressure_pa);
        let indicated_torque_nm =
            gas_force_n * piston_dx_dtheta_m_per_rad(&self.definition, start_angle_rad);
        let indicated_work_j = indicated_torque_nm * delta_angle_rad;

        self.accumulated_indicated_work_j += indicated_work_j;
        self.accumulated_crank_angle_rad += delta_angle_rad.abs();
        if inputs.fixed_crank_speed_rad_per_s.is_none() {
            let starter_torque_nm = if inputs.starter_enabled
                && rad_per_s_to_rpm(self.crank_speed_rad_per_s) < inputs.starter_speed_limit_rpm
            {
                inputs.starter_torque_nm.max(0.0)
            } else {
                0.0
            };
            let effective_inertia_kg_m2 =
                self.definition.crank.moment_of_inertia_kg_m2 + inputs.added_inertia_kg_m2;
            let acceleration_rad_per_s2 = (indicated_torque_nm + starter_torque_nm
                - inputs.external_load_torque_nm)
                / effective_inertia_kg_m2;
            self.crank_speed_rad_per_s =
                (self.crank_speed_rad_per_s + acceleration_rad_per_s2 * timestep_seconds).max(0.0);
        } else {
            self.crank_speed_rad_per_s = crank_speed_rad_per_s;
        }
        self.crank_angle_rad = end_angle_rad;
        self.elapsed_time_seconds += timestep_seconds;

        SingleCylinderStepOutput {
            elapsed_time_seconds: self.elapsed_time_seconds,
            crank_angle_rad: self.crank_angle_rad,
            crank_speed_rad_per_s: self.crank_speed_rad_per_s,
            rpm: rad_per_s_to_rpm(self.crank_speed_rad_per_s),
            cylinder_volume_m3: end_volume_m3,
            volume_rate_m3_per_s,
            cylinder_pressure_pa: end_pressure_pa,
            cylinder_temperature_k: self.chamber_state.temperature_k,
            chamber_mass_kg: self.chamber_state.mass_kg,
            intake_effective_area_m2: intake_area_m2,
            exhaust_effective_area_m2: exhaust_area_m2,
            intake_mass_flow_kg_per_s: derivatives.inlet_mass_rate_kg_per_s,
            exhaust_mass_flow_kg_per_s: derivatives.outlet_mass_rate_kg_per_s,
            gas_force_n,
            indicated_torque_nm,
            load_torque_nm: inputs.external_load_torque_nm,
            combustion_heat_added_j,
            indicated_work_j,
            mean_indicated_torque_nm: self.mean_indicated_torque_nm(),
        }
    }

    fn combustion_heat_added_j(
        &mut self,
        start_angle_rad: f64,
        end_angle_rad: f64,
        crank_speed_rad_per_s: f64,
        combustion_enabled: bool,
    ) -> f64 {
        if !self.definition.combustion.enabled || !combustion_enabled {
            self.active_combustion = None;
            return 0.0;
        }

        let spark_angle_deg = spark_angle_deg_for_rpm(
            self.definition.combustion.spark_timing,
            rad_per_s_to_rpm(crank_speed_rad_per_s),
        );
        let start_of_combustion_rad = normalize_cycle_angle_rad(
            spark_angle_deg.to_radians()
                + self.definition.combustion.ignition_delay_deg.to_radians(),
        );
        if self.active_combustion.is_none()
            && crossed_cycle_angle_rad(start_angle_rad, end_angle_rad, start_of_combustion_rad)
        {
            self.active_combustion = Some(ActiveCombustion {
                start_angle_rad: start_of_combustion_rad,
                burned_fraction: 0.0,
            });
        }

        let Some(mut event) = self.active_combustion else {
            return 0.0;
        };

        let elapsed_angle_rad = positive_cycle_delta_rad(event.start_angle_rad, end_angle_rad);
        let cumulative_fraction =
            cumulative_wiebe_burned_fraction(elapsed_angle_rad, self.definition.combustion.wiebe);
        let burned_fraction_delta = (cumulative_fraction - event.burned_fraction).max(0.0);
        event.burned_fraction = cumulative_fraction.max(event.burned_fraction);

        if elapsed_angle_rad >= self.definition.combustion.wiebe.combustion_duration_rad
            || event.burned_fraction >= 1.0
        {
            self.active_combustion = None;
        } else {
            self.active_combustion = Some(event);
        }

        burned_fraction_delta
            * self.definition.combustion.fuel_mass_per_cycle_kg
            * self.definition.combustion.fuel_lower_heating_value_j_per_kg
            * self.definition.combustion.combustion_efficiency
            * (1.0 - self.definition.combustion.heat_loss_fraction).clamp(0.0, 1.0)
    }
}

impl StepModel for SingleCylinderEngine {
    type Inputs = SingleCylinderStepInputs;
    type Outputs = SingleCylinderStepOutput;

    fn step(&mut self, context: StepContext, inputs: Self::Inputs) -> Self::Outputs {
        self.step_with_timestep(context.timestep_seconds, inputs)
    }
}

pub fn rpm_to_rad_per_s(rpm: f64) -> f64 {
    rpm * std::f64::consts::TAU / 60.0
}

pub fn rad_per_s_to_rpm(rad_per_s: f64) -> f64 {
    rad_per_s * 60.0 / std::f64::consts::TAU
}

pub fn spark_angle_deg_for_rpm(timing: SparkTimingDefinition, rpm: f64) -> f64 {
    let speed_span = timing.high_speed_rpm - timing.low_speed_rpm;
    let advance_deg_btdc = if speed_span <= 0.0 {
        timing.high_speed_advance_deg_btdc
    } else {
        let blend = ((rpm - timing.low_speed_rpm) / speed_span).clamp(0.0, 1.0);
        timing.low_speed_advance_deg_btdc
            + blend * (timing.high_speed_advance_deg_btdc - timing.low_speed_advance_deg_btdc)
    };

    normalize_cycle_angle_rad((720.0 - advance_deg_btdc).to_radians()).to_degrees()
}

pub fn run_fixed_speed_cycles(
    definition: &EngineDefinition,
    rpm: f64,
    cycles: usize,
) -> FixedSpeedRunSummary {
    run_fixed_speed_cycles_with_profile(
        definition,
        profile_from_definition(definition),
        rpm,
        cycles,
    )
}

pub fn run_fixed_speed_cycles_with_profile(
    definition: &EngineDefinition,
    profile: SimulationProfile,
    rpm: f64,
    cycles: usize,
) -> FixedSpeedRunSummary {
    assert!(rpm > 0.0, "rpm must be positive");
    assert!(cycles > 0, "cycle count must be positive");
    profile.validate();

    let mut engine =
        SingleCylinderEngine::from_definition_with_profile(definition.clone(), profile);
    engine.reset_accumulators();
    let crank_speed_rad_per_s = rpm_to_rad_per_s(rpm);
    let timestep_seconds = profile.timestep_seconds;
    let total_angle_rad = ENGINE_CYCLE_RADIANS * cycles as f64;
    let mut remaining_angle_rad = total_angle_rad;
    let mut steps = 0;

    while remaining_angle_rad > f64::EPSILON {
        let step_angle_rad = (crank_speed_rad_per_s * timestep_seconds).min(remaining_angle_rad);
        let step_timestep_seconds = step_angle_rad / crank_speed_rad_per_s;
        engine.step_with_timestep(
            step_timestep_seconds,
            SingleCylinderStepInputs {
                fixed_crank_speed_rad_per_s: Some(crank_speed_rad_per_s),
                ..SingleCylinderStepInputs::default()
            },
        );
        remaining_angle_rad -= step_angle_rad;
        steps += 1;
    }

    let elapsed_time_seconds = total_angle_rad / crank_speed_rad_per_s;
    FixedSpeedRunSummary {
        rpm,
        cycles,
        steps,
        integrated_angle_rad: total_angle_rad,
        elapsed_time_seconds,
        indicated_work_j: engine.accumulated_indicated_work_j(),
        mean_indicated_torque_nm: engine.mean_indicated_torque_nm(),
    }
}

pub fn run_fixed_speed_torque_sweep(
    definition: &EngineDefinition,
    rpm_points: &[f64],
    cycles_per_point: usize,
) -> Vec<TorqueSweepPoint> {
    run_fixed_speed_torque_sweep_with_profile(
        definition,
        profile_from_definition(definition),
        rpm_points,
        cycles_per_point,
    )
}

pub fn run_fixed_speed_torque_sweep_with_profile(
    definition: &EngineDefinition,
    profile: SimulationProfile,
    rpm_points: &[f64],
    cycles_per_point: usize,
) -> Vec<TorqueSweepPoint> {
    profile.validate();
    rpm_points
        .iter()
        .copied()
        .map(|rpm| {
            let summary =
                run_fixed_speed_cycles_with_profile(definition, profile, rpm, cycles_per_point);
            let angular_speed_rad_per_s = rpm_to_rad_per_s(rpm);
            TorqueSweepPoint {
                rpm,
                mean_indicated_torque_nm: summary.mean_indicated_torque_nm,
                indicated_power_kw: summary.mean_indicated_torque_nm * angular_speed_rad_per_s
                    / 1000.0,
            }
        })
        .collect()
}

fn profile_from_definition(definition: &EngineDefinition) -> SimulationProfile {
    SimulationProfile {
        timestep_seconds: definition.simulation.timestep_seconds,
        ..SimulationProfile::real_time()
    }
}

fn chamber_properties(definition: &EngineDefinition) -> ChamberProperties {
    ChamberProperties {
        gas_constant_j_per_kg_k: definition.gas.gas_constant_j_per_kg_k,
        specific_heat_ratio: definition.gas.specific_heat_ratio,
        minimum_mass_kg: definition.gas.minimum_mass_kg,
        minimum_temperature_k: definition.gas.minimum_temperature_k,
    }
}

fn chamber_state_at_pressure(
    pressure_pa: f64,
    temperature_k: f64,
    volume_m3: f64,
    properties: ChamberProperties,
) -> ChamberState {
    ChamberState {
        mass_kg: (pressure_pa * volume_m3 / (properties.gas_constant_j_per_kg_k * temperature_k))
            .max(properties.minimum_mass_kg),
        temperature_k: temperature_k.max(properties.minimum_temperature_k),
    }
}

fn inlet_boundary(
    valve: ValveDefinition,
    area_m2: f64,
    definition: &EngineDefinition,
) -> FlowBoundary {
    FlowBoundary {
        upstream_pressure_pa: definition.boundaries.intake_pressure_pa,
        upstream_temperature_k: definition.boundaries.intake_temperature_k,
        downstream_pressure_pa: 0.0,
        discharge_coefficient: valve.discharge_coefficient,
        area_m2,
    }
}

fn outlet_boundary(
    valve: ValveDefinition,
    area_m2: f64,
    definition: &EngineDefinition,
) -> FlowBoundary {
    FlowBoundary {
        upstream_pressure_pa: definition.boundaries.exhaust_pressure_pa,
        upstream_temperature_k: definition.boundaries.exhaust_temperature_k,
        downstream_pressure_pa: definition.boundaries.exhaust_pressure_pa,
        discharge_coefficient: valve.discharge_coefficient,
        area_m2,
    }
}

fn cylinder_volume_m3(definition: &EngineDefinition, crank_angle_rad: f64) -> f64 {
    slider_crank_cylinder_volume_m3(
        crank_angle_rad,
        clearance_volume_m3(
            definition.geometry.bore_m,
            definition.geometry.stroke_m,
            definition.geometry.compression_ratio,
        ),
        bore_area_m2(definition.geometry.bore_m),
        crank_radius_m(definition.geometry.stroke_m),
        definition.geometry.connecting_rod_length_m,
    )
}

fn cylinder_volume_rate_m3_per_s(
    definition: &EngineDefinition,
    crank_angle_rad: f64,
    crank_speed_rad_per_s: f64,
) -> f64 {
    slider_crank_volume_rate_m3_per_s(
        crank_angle_rad,
        crank_speed_rad_per_s,
        bore_area_m2(definition.geometry.bore_m),
        crank_radius_m(definition.geometry.stroke_m),
        definition.geometry.connecting_rod_length_m,
    )
}

fn piston_dx_dtheta_m_per_rad(definition: &EngineDefinition, crank_angle_rad: f64) -> f64 {
    slider_crank_dx_dtheta_m_per_rad(
        crank_angle_rad,
        crank_radius_m(definition.geometry.stroke_m),
        definition.geometry.connecting_rod_length_m,
    )
}

fn gas_force_n(definition: &EngineDefinition, cylinder_pressure_pa: f64) -> f64 {
    (cylinder_pressure_pa - definition.boundaries.crankcase_pressure_pa)
        * bore_area_m2(definition.geometry.bore_m)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chamber::ChamberDerivatives;

    const EPSILON: f64 = 1.0e-9;

    fn definition() -> EngineDefinition {
        EngineDefinition::from_json_str(include_str!("../data/engines/gn250.json"))
            .expect("GN250 JSON should parse")
    }

    fn assert_approx_eq(actual: f64, expected: f64, tolerance: f64) {
        let difference = (actual - expected).abs();
        assert!(
            difference <= tolerance,
            "expected {expected}, got {actual}; difference {difference} exceeded {tolerance}",
        );
    }

    #[test]
    fn loads_gn250_engine_definition() {
        let definition = definition();

        assert_eq!(definition.metadata.name, "Suzuki GN250 approximation");
        assert_eq!(definition.geometry.bore_m, 0.072);
        assert_eq!(definition.geometry.stroke_m, 0.0612);
    }

    #[test]
    fn rpm_conversions_round_trip() {
        assert_approx_eq(rad_per_s_to_rpm(rpm_to_rad_per_s(3000.0)), 3000.0, EPSILON);
    }

    #[test]
    fn spark_curve_converts_btdc_advance_to_cycle_angle() {
        let timing = definition().combustion.spark_timing;

        assert_approx_eq(spark_angle_deg_for_rpm(timing, 1600.0), 710.0, EPSILON);
        assert_approx_eq(spark_angle_deg_for_rpm(timing, 3000.0), 685.0, EPSILON);
        assert_approx_eq(spark_angle_deg_for_rpm(timing, 2350.0), 697.5, EPSILON);
    }

    #[test]
    fn motored_cycle_repeats_deterministically() {
        let mut first_definition = definition();
        first_definition.combustion.enabled = false;
        let second_definition = first_definition.clone();
        let first = run_fixed_speed_cycles(&first_definition, 3000.0, 1);
        let second = run_fixed_speed_cycles(&second_definition, 3000.0, 1);

        assert_eq!(first.steps, second.steps);
        assert_approx_eq(
            first.indicated_work_j,
            second.indicated_work_j,
            first.indicated_work_j.abs().max(1.0) * 1.0e-12,
        );
        assert_approx_eq(
            first.mean_indicated_torque_nm,
            second.mean_indicated_torque_nm,
            first.mean_indicated_torque_nm.abs().max(1.0) * 1.0e-12,
        );
    }

    #[test]
    fn fired_cycle_produces_more_work_than_motored_cycle() {
        let fired_definition = definition();
        let mut motored_definition = fired_definition.clone();
        motored_definition.combustion.enabled = false;

        let fired = run_fixed_speed_cycles(&fired_definition, 3000.0, 2);
        let motored = run_fixed_speed_cycles(&motored_definition, 3000.0, 2);

        assert!(fired.indicated_work_j > motored.indicated_work_j);
        assert!(fired.mean_indicated_torque_nm > motored.mean_indicated_torque_nm);
    }

    #[test]
    fn valve_overlap_allows_intake_flow_reversal() {
        let mut definition = definition();
        definition.crank.initial_crank_angle_deg = 390.0;
        let mut engine = SingleCylinderEngine::from_definition(definition.clone());
        let volume_m3 = cylinder_volume_m3(&definition, engine.crank_angle_rad());
        engine.chamber_state = chamber_state_at_pressure(
            definition.boundaries.intake_pressure_pa * 1.2,
            definition.boundaries.intake_temperature_k,
            volume_m3,
            chamber_properties(&definition),
        );

        let output = engine.step(SingleCylinderStepInputs {
            fixed_crank_speed_rad_per_s: Some(rpm_to_rad_per_s(3000.0)),
            ..SingleCylinderStepInputs::default()
        });

        assert!(output.intake_effective_area_m2 > 0.0);
        assert!(output.intake_mass_flow_kg_per_s < 0.0);
    }

    #[test]
    fn fixed_speed_sweep_reports_power_from_mean_torque() {
        let points = run_fixed_speed_torque_sweep(&definition(), &[3000.0, 4000.0], 1);

        assert_eq!(points.len(), 2);
        for point in points {
            assert_approx_eq(
                point.indicated_power_kw,
                point.mean_indicated_torque_nm * rpm_to_rad_per_s(point.rpm) / 1000.0,
                point.indicated_power_kw.abs().max(1.0) * 1.0e-12,
            );
        }
    }

    #[test]
    fn cycle_work_matches_reported_mean_torque() {
        let summary = run_fixed_speed_cycles(&definition(), 3000.0, 1);
        let expected_work_j = summary.mean_indicated_torque_nm * summary.integrated_angle_rad;

        assert_approx_eq(
            summary.indicated_work_j,
            expected_work_j,
            summary.indicated_work_j.abs().max(1.0) * 1.0e-12,
        );
    }

    #[test]
    fn step_model_uses_supplied_context_timestep() {
        let mut engine = SingleCylinderEngine::from_definition(definition());
        let context = StepContext::new(0.0001);
        let output = StepModel::step(
            &mut engine,
            context,
            SingleCylinderStepInputs {
                fixed_crank_speed_rad_per_s: Some(rpm_to_rad_per_s(3000.0)),
                ..SingleCylinderStepInputs::default()
            },
        );

        assert_approx_eq(output.elapsed_time_seconds, 0.0001, EPSILON);
    }

    #[test]
    fn chamber_derivatives_are_available_for_boundary_diagnostics() {
        let definition = definition();
        let engine = SingleCylinderEngine::from_definition(definition.clone());
        let boundary = ChamberBoundary {
            volume_m3: cylinder_volume_m3(&definition, engine.crank_angle_rad()),
            volume_rate_m3_per_s: 0.0,
            heat_rate_w: 0.0,
        };
        let derivatives = chamber_derivatives(
            engine.chamber_state(),
            boundary,
            FlowBoundary::closed(),
            FlowBoundary::closed(),
            chamber_properties(&definition),
        );

        assert_eq!(derivatives, ChamberDerivatives::default());
    }
}
