use crate::combustion::{
    BurnDurationInputs, IgnitionDelayInputs, WiebeParameters, air_fuel_ratio, burn_duration_rad,
    burn_species, cumulative_wiebe_burned_fraction, equivalence_ratio_from_air_fuel_ratio,
    ignition_delay_rad, mixture_combustion_efficiency, stoichiometric_oxygen_fuel_ratio,
};
use crate::engine_config::{EngineDefinition, SparkTimingDefinition};
use crate::engine_geometry::{
    bore_area_m2, clearance_volume_m3, crank_radius_m, slider_crank_cylinder_volume_m3,
    slider_crank_dx_dtheta_m_per_rad, slider_crank_volume_rate_m3_per_s,
};
use crate::engine_handling::EngineHandlingDefinition;
use crate::physics::chamber::{
    ChamberBoundary, ChamberProperties, ChamberSpeciesMasses, ChamberState,
    DRY_AIR_OXYGEN_MASS_FRACTION, chamber_pressure_pa, integrate_internal_energy_rk4,
    mixture_chamber_properties, species_internal_energy_j,
};
use crate::physics::gas::cv;
use crate::physics::pipe::{PipeBoundaryFlux, PlenumState, ThrottlePlenumInput};
use crate::profiles::SimulationProfile;
use crate::simulation::{StepContext, StepModel};
use crate::throttle;
use crate::valve::{
    ENGINE_CYCLE_RADIANS, ValveEvent, crossed_cycle_angle_rad, normalize_cycle_angle_rad,
    positive_cycle_delta_rad,
};
use onedpipes::{
    DuctConfig, DuctEnd, ExternalBoundaryControl, ExternalBoundaryId, GasProperties, Model,
    ModelBoundary, PipeEnd, PipeId, SpeciesFractions, SpeciesMass, State as PipeState,
    TemperatureDependentAir, ValveOrifice,
};

const LOW_SPEED_FUELING_FALLBACK_RPM: f64 = 500.0;
const PLENUM_RUNNER_COUPLING_CELL_FRACTION: f64 = 0.5;
const VALVE_COUPLING_CELL_FRACTION: f64 = 0.5;
const INTAKE_PLENUM_EXTERNAL_ID: usize = 0;
const INTAKE_VALVE_EXTERNAL_ID: usize = 1;
const EXHAUST_VALVE_EXTERNAL_ID: usize = 2;

#[derive(Debug, Clone)]
struct OneDPipeNetwork {
    gas: TemperatureDependentAir,
    model: Model<TemperatureDependentAir>,
    intake_runner: PipeId,
    exhaust_runner: PipeId,
    exhaust_collector: Option<PipeId>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct EnginePipeFlow {
    mass_kg_per_s: f64,
    energy_w: f64,
}

impl EnginePipeFlow {
    fn zero() -> Self {
        Self {
            mass_kg_per_s: 0.0,
            energy_w: 0.0,
        }
    }
}

/// One external boundary of the pipe network for a macro step. The external
/// (chamber/plenum) state is frozen for the step; the orifice flow against it
/// is re-evaluated from the live pipe state every solver substep so a blowdown
/// pulse decays naturally as the boundary cell fills instead of a stale
/// start-of-step rate being forced in for the whole step.
#[derive(Debug, Clone, Copy, PartialEq)]
struct EnginePipeBoundaryRequest {
    pipe_id: PipeId,
    end: DuctEnd,
    external_state: PipeState,
    flow_area_m2: f64,
    inflow_species: SpeciesFractions,
    /// Fraction of the boundary cell's current mass that may transfer per substep.
    cell_fraction: f64,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct AcceptedPipeTransfer {
    mass_kg: f64,
    energy_j: f64,
    species_kg: SpeciesMass,
}

impl AcceptedPipeTransfer {
    fn absorb(&mut self, other: Self) {
        self.mass_kg += other.mass_kg;
        self.energy_j += other.energy_j;
        self.species_kg = self.species_kg.add_scaled(other.species_kg, 1.0);
    }

    fn into_flux(self, timestep_seconds: f64) -> PipeBoundaryFlux {
        if timestep_seconds <= 0.0 {
            return PipeBoundaryFlux::zero();
        }

        PipeBoundaryFlux {
            mass_kg_per_s: self.mass_kg / timestep_seconds,
            momentum_n: 0.0,
            energy_w: self.energy_j / timestep_seconds,
            species_kg_per_s: species_mass_to_chamber_species(self.species_kg, timestep_seconds),
        }
    }
}

#[derive(Debug, Clone)]
pub struct SingleCylinderEngine {
    definition: EngineDefinition,
    profile: SimulationProfile,
    chamber_state: ChamberState,
    chamber_species: ChamberSpeciesMasses,
    intake_plenum: PlenumState,
    pipes: OneDPipeNetwork,
    crank_angle_rad: f64,
    crank_speed_rad_per_s: f64,
    pending_combustion: Option<PendingCombustion>,
    active_combustion: Option<ActiveCombustion>,
    current_cycle_intake_air_kg: f64,
    last_completed_cycle_intake_air_kg: Option<f64>,
    species_budget: EngineSpeciesBudget,
    accumulated_indicated_work_j: f64,
    accumulated_crank_angle_rad: f64,
    elapsed_time_seconds: f64,
    over_temperature_warning_emitted: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct PendingCombustion {
    start_angle_rad: f64,
    burn_duration_rad: f64,
    burnable_fuel_snapshot_kg: f64,
    combustion_efficiency: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct ActiveCombustion {
    start_angle_rad: f64,
    burn_duration_rad: f64,
    burnable_fuel_snapshot_kg: f64,
    combustion_efficiency: f64,
    previous_wiebe_fraction: f64,
    consumed_fuel_kg: f64,
    consumed_oxygen_kg: f64,
    generated_products_kg: f64,
    released_heat_j: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct CombustionStepResult {
    heat_added_j: f64,
    internal_energy_before_heat_j: f64,
    fuel_injected_kg: f64,
    fuel_burned_kg: f64,
    oxygen_consumed_kg: f64,
    products_generated_kg: f64,
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
    pub lambda_target: f64,
    pub throttle_position: f64,
    pub idle_throttle_fraction: f64,
    pub throttle_effective_area_fraction: Option<f64>,
    /// When set, the intake manifold behaves as an ideal pressure reservoir at
    /// this absolute pressure for the whole step, bypassing throttle/plenum
    /// dynamics. This is the primary forced-MAP diagnostic mode.
    pub forced_map_pressure_pa: Option<f64>,
    /// Legacy finite-plenum MAP forcing: when set, the plenum is reset to this
    /// pressure before pipe coupling, then runner/plenum exchange is allowed to
    /// move it during the step.
    pub repinned_plenum_pressure_pa: Option<f64>,
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
            lambda_target: 1.0,
            throttle_position: 1.0,
            idle_throttle_fraction: 0.0,
            throttle_effective_area_fraction: None,
            forced_map_pressure_pa: None,
            repinned_plenum_pressure_pa: None,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct EngineSpeciesBudget {
    pub injected_fuel_kg: f64,
    pub burned_fuel_kg: f64,
    pub fuel_remaining_in_chamber_kg: f64,
    pub fuel_exported_through_exhaust_kg: f64,
    pub oxygen_consumed_kg: f64,
    pub products_generated_kg: f64,
    pub products_exported_through_exhaust_kg: f64,
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
    pub chamber_oxygen_mass_kg: f64,
    pub chamber_fuel_mass_kg: f64,
    pub chamber_inert_mass_kg: f64,
    pub chamber_products_mass_kg: f64,
    pub chamber_lambda: Option<f64>,
    pub intake_plenum_pressure_pa: f64,
    pub intake_runner_pressure_pa: f64,
    pub exhaust_runner_pressure_pa: f64,
    /// Pressure in the collector tailpipe at the junction end, or `None` when
    /// the engine has no exhaust collector.
    pub exhaust_collector_pressure_pa: Option<f64>,
    pub exhaust_exit_pressure_pa: f64,
    pub intake_effective_area_m2: f64,
    pub exhaust_effective_area_m2: f64,
    pub intake_mass_flow_kg_per_s: f64,
    pub exhaust_mass_flow_kg_per_s: f64,
    pub intake_species_flow_kg_per_s: ChamberSpeciesMasses,
    pub exhaust_species_flow_kg_per_s: ChamberSpeciesMasses,
    pub fuel_injected_kg: f64,
    pub fuel_burned_kg: f64,
    pub oxygen_consumed_kg: f64,
    pub products_generated_kg: f64,
    pub species_budget: EngineSpeciesBudget,
    pub exhaust_lambda: Option<f64>,
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

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThrottleLoadSweepPoint {
    pub rpm: f64,
    pub throttle_effective_area_fraction: f64,
    pub external_load_torque_nm: f64,
    pub mean_indicated_torque_nm: f64,
    pub indicated_power_kw: f64,
}

impl SingleCylinderEngine {
    pub fn from_definition(definition: EngineDefinition) -> Self {
        Self::from_definition_with_profile(definition, default_runner_profile())
    }

    /// Build the engine using the timestep/profile derived from a handling
    /// config. This is the path the GUI and on-disk loading use now that
    /// timestep no longer lives in the engine definition.
    pub fn from_definition_and_handling(
        definition: EngineDefinition,
        handling: &EngineHandlingDefinition,
    ) -> Self {
        Self::from_definition_with_profile(definition, handling.to_profile())
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
        let chamber_species = ChamberSpeciesMasses::from_dry_air_mass(chamber_state.mass_kg);
        let intake_plenum = default_intake_plenum(&definition);
        let pipes = default_pipe_network(&definition);

        Self {
            definition,
            profile,
            chamber_state,
            chamber_species,
            intake_plenum,
            pipes,
            crank_angle_rad: normalize_cycle_angle_rad(crank_angle_rad),
            crank_speed_rad_per_s,
            pending_combustion: None,
            active_combustion: None,
            current_cycle_intake_air_kg: 0.0,
            last_completed_cycle_intake_air_kg: None,
            species_budget: EngineSpeciesBudget {
                fuel_remaining_in_chamber_kg: chamber_species.fuel_kg,
                ..EngineSpeciesBudget::default()
            },
            accumulated_indicated_work_j: 0.0,
            accumulated_crank_angle_rad: 0.0,
            elapsed_time_seconds: 0.0,
            over_temperature_warning_emitted: false,
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

    pub fn chamber_species(&self) -> ChamberSpeciesMasses {
        self.chamber_species
    }

    pub fn species_budget(&self) -> EngineSpeciesBudget {
        EngineSpeciesBudget {
            fuel_remaining_in_chamber_kg: self.chamber_species.fuel_kg,
            ..self.species_budget
        }
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
        let substeps = self.profile.chamber_substeps.max(1);
        if substeps > 1 {
            let substep_seconds = timestep_seconds / substeps as f64;
            let mut last_output = None;
            let mut heat_added_j = 0.0;
            let mut indicated_work_j = 0.0;
            for _ in 0..substeps {
                let output = self.step_interval(substep_seconds, inputs);
                heat_added_j += output.combustion_heat_added_j;
                indicated_work_j += output.indicated_work_j;
                last_output = Some(output);
            }
            let mut output = last_output.expect("substep count is positive");
            output.combustion_heat_added_j = heat_added_j;
            output.indicated_work_j = indicated_work_j;
            output.mean_indicated_torque_nm = self.mean_indicated_torque_nm();
            return output;
        }

        self.step_interval(timestep_seconds, inputs)
    }

    fn step_interval(
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
        assert!(inputs.lambda_target > 0.0, "lambda target must be positive");

        let start_angle_rad = self.crank_angle_rad;
        let delta_angle_rad = crank_speed_rad_per_s * timestep_seconds;
        let end_angle_rad = normalize_cycle_angle_rad(start_angle_rad + delta_angle_rad);
        let start_volume_m3 = cylinder_volume_m3(&self.definition, start_angle_rad);
        let volume_rate_m3_per_s =
            cylinder_volume_rate_m3_per_s(&self.definition, start_angle_rad, crank_speed_rad_per_s);
        self.sync_chamber_mass_from_species();
        let start_properties = self.current_chamber_properties();
        let start_pressure_pa = chamber_pressure_pa(
            self.chamber_state,
            ChamberBoundary {
                volume_m3: start_volume_m3,
                volume_rate_m3_per_s,
                heat_rate_w: 0.0,
            },
            start_properties,
        );

        let intake_valve = ValveEvent::from_definition(self.definition.valves.intake);
        let exhaust_valve = ValveEvent::from_definition(self.definition.valves.exhaust);
        let intake_area_m2 = intake_valve.effective_area_m2(start_angle_rad);
        let exhaust_area_m2 = exhaust_valve.effective_area_m2(start_angle_rad);
        let plenum_fallback_properties = chamber_properties(&self.definition);
        let throttle_maximum_area_m2 = self
            .definition
            .intake_exhaust
            .throttle_maximum_area_m2
            .max(1.0e-6);
        let idle_throttle_maximum_area_m2 = self
            .definition
            .intake_exhaust
            .effective_idle_throttle_area_m2();
        let throttle_effective_area_m2 =
            if let Some(area_fraction) = inputs.throttle_effective_area_fraction {
                (throttle_maximum_area_m2 + idle_throttle_maximum_area_m2)
                    * area_fraction.clamp(0.0, 1.0)
            } else {
                throttle::effective_area_m2(
                    0.0,
                    throttle_maximum_area_m2,
                    inputs.throttle_position,
                    1.0,
                ) + throttle::effective_area_m2(
                    0.0,
                    idle_throttle_maximum_area_m2,
                    inputs.idle_throttle_fraction,
                    1.0,
                )
            };
        self.intake_plenum.step_throttle(
            ThrottlePlenumInput {
                throttle_position: 0.0,
                minimum_area_m2: throttle_effective_area_m2,
                maximum_added_area_m2: 0.0,
                maximum_throttle_position: 1.0,
                discharge_coefficient: 1.0,
                upstream_pressure_pa: self.definition.boundaries.intake_pressure_pa,
                upstream_temperature_k: self.definition.boundaries.intake_temperature_k,
            },
            plenum_fallback_properties,
            timestep_seconds,
        );
        let ideal_map_pressure_pa = inputs
            .forced_map_pressure_pa
            .map(|pressure| pressure.max(1.0));
        let repinned_plenum_pressure_pa = inputs
            .repinned_plenum_pressure_pa
            .map(|pressure| pressure.max(1.0));
        if let Some(repinned_plenum_pressure_pa) = repinned_plenum_pressure_pa {
            // Legacy finite-plenum MAP forcing: reset the plenum before pipe
            // coupling, then allow pipe/plenum exchange to move it during the
            // step.
            self.intake_plenum = PlenumState::from_pressure_temperature(
                repinned_plenum_pressure_pa,
                self.definition.boundaries.intake_temperature_k,
                self.intake_plenum.volume_m3,
                plenum_fallback_properties,
            );
        }
        if let Some(ideal_map_pressure_pa) = ideal_map_pressure_pa {
            // Ideal forced-MAP diagnostic: the runner sees the requested
            // pressure source for this step, and the stored/reporting plenum is
            // restored to that source after pipe coupling below.
            self.intake_plenum = PlenumState::from_pressure_temperature(
                ideal_map_pressure_pa,
                self.definition.boundaries.intake_temperature_k,
                self.intake_plenum.volume_m3,
                plenum_fallback_properties,
            );
        }
        let combustion_step = self.combustion_step_result(
            start_angle_rad,
            end_angle_rad,
            crank_speed_rad_per_s,
            inputs.spark_enabled && inputs.fuel_enabled,
            inputs.lambda_target,
        );
        self.chamber_state = chamber_state_from_internal_energy(
            self.chamber_species,
            self.chamber_state,
            combustion_step.internal_energy_before_heat_j,
            chamber_properties(&self.definition),
        );
        let combustion_heat_added_j = combustion_step.heat_added_j;
        self.sync_chamber_mass_from_species();
        let plenum_pressure_pa = self.intake_plenum.pressure_pa(plenum_fallback_properties);
        let plenum_state = pipe_state_from_pressure_temperature(
            plenum_pressure_pa,
            self.intake_plenum.chamber_state.temperature_k,
            self.pipes.gas,
        );
        let chamber_pipe_state = pipe_state_from_pressure_temperature(
            start_pressure_pa,
            self.chamber_state.temperature_k,
            self.pipes.gas,
        );
        let chamber_props = chamber_properties(&self.definition);
        // Donor budgets: how much mass each external volume may supply into
        // the pipes over this step without dropping below its minimum. The
        // chamber budget is shared by the intake and exhaust valve boundaries.
        let plenum_donor_budget_kg = if ideal_map_pressure_pa.is_some() {
            f64::INFINITY
        } else {
            (self.intake_plenum.species.total_mass_kg()
                - plenum_fallback_properties.minimum_mass_kg)
                .max(0.0)
        };
        let chamber_donor_budget_kg =
            (self.chamber_species.total_mass_kg() - chamber_props.minimum_mass_kg).max(0.0);

        let accepted_transfers = self.pipes.step_stable(
            timestep_seconds,
            self.pipes.boundary_request(
                self.pipes.intake_runner,
                DuctEnd::Left,
                plenum_state,
                self.pipes.pipe_area_m2(self.pipes.intake_runner),
                chamber_species_to_pipe_fractions(self.intake_plenum.species),
                PLENUM_RUNNER_COUPLING_CELL_FRACTION,
            ),
            self.pipes.boundary_request(
                self.pipes.intake_runner,
                DuctEnd::Right,
                chamber_pipe_state,
                intake_area_m2 * intake_valve.discharge_coefficient.clamp(0.0, 1.0),
                chamber_species_to_pipe_fractions(self.chamber_species),
                VALVE_COUPLING_CELL_FRACTION,
            ),
            self.pipes.boundary_request(
                self.pipes.exhaust_runner,
                DuctEnd::Left,
                chamber_pipe_state,
                exhaust_area_m2 * exhaust_valve.discharge_coefficient.clamp(0.0, 1.0),
                chamber_species_to_pipe_fractions(self.chamber_species),
                VALVE_COUPLING_CELL_FRACTION,
            ),
            self.intake_plenum.volume_m3,
            plenum_donor_budget_kg,
            chamber_donor_budget_kg,
        );
        let [plenum_transfer, intake_transfer, exhaust_transfer] = accepted_transfers;
        let plenum_to_runner_flux = plenum_transfer.into_flux(timestep_seconds);
        let intake_flux = intake_transfer.into_flux(timestep_seconds);
        let exhaust_flux = exhaust_transfer.into_flux(timestep_seconds);

        let plenum_volume_m3 = self.intake_plenum.volume_m3;
        self.intake_plenum.chamber_state = step_chamber_with_pipe_fluxes(
            &mut self.intake_plenum.species,
            self.intake_plenum.chamber_state,
            plenum_fallback_properties,
            [plenum_to_runner_flux, PipeBoundaryFlux::zero()],
            0.0,
            timestep_seconds,
            // Fixed-volume plenum: no piston work, so RK4 reduces to exact
            // linear integration of the constant flux/heat rates.
            |_| (plenum_volume_m3, 0.0),
        );
        if let Some(ideal_map_pressure_pa) = ideal_map_pressure_pa {
            self.intake_plenum = PlenumState::from_pressure_temperature(
                ideal_map_pressure_pa,
                self.definition.boundaries.intake_temperature_k,
                self.intake_plenum.volume_m3,
                plenum_fallback_properties,
            );
        }
        let intake_species_flow_kg_per_s = intake_flux.species_kg_per_s;
        let exhaust_species_flow_kg_per_s =
            scaled_species_flow(exhaust_flux.species_kg_per_s, -1.0);
        self.current_cycle_intake_air_kg +=
            net_plenum_supplied_air_delta_kg(plenum_to_runner_flux, timestep_seconds);
        self.species_budget.fuel_exported_through_exhaust_kg +=
            (exhaust_species_flow_kg_per_s.fuel_kg * timestep_seconds).max(0.0);
        self.species_budget.products_exported_through_exhaust_kg +=
            (exhaust_species_flow_kg_per_s.products_kg * timestep_seconds).max(0.0);

        self.chamber_state = step_chamber_with_pipe_fluxes(
            &mut self.chamber_species,
            self.chamber_state,
            chamber_properties(&self.definition),
            [intake_flux, exhaust_flux],
            combustion_heat_added_j,
            timestep_seconds,
            // Sample the moving cylinder volume at each RK4 stage by advancing
            // the crank angle across the step at the fixed step speed.
            |fraction| {
                let angle_rad =
                    normalize_cycle_angle_rad(start_angle_rad + delta_angle_rad * fraction);
                (
                    cylinder_volume_m3(&self.definition, angle_rad),
                    cylinder_volume_rate_m3_per_s(
                        &self.definition,
                        angle_rad,
                        crank_speed_rad_per_s,
                    ),
                )
            },
        );
        self.sync_chamber_mass_from_species();

        let end_volume_m3 = cylinder_volume_m3(&self.definition, end_angle_rad);
        let end_boundary = ChamberBoundary {
            volume_m3: end_volume_m3,
            volume_rate_m3_per_s: 0.0,
            heat_rate_w: 0.0,
        };
        let end_pressure_pa = chamber_pressure_pa(
            self.chamber_state,
            end_boundary,
            self.current_chamber_properties(),
        );
        self.warn_once_if_chamber_temperature_extreme();
        let gas_force_n = gas_force_n(&self.definition, end_pressure_pa);
        let indicated_torque_nm =
            gas_force_n * piston_dx_dtheta_m_per_rad(&self.definition, end_angle_rad);
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
        if crossed_cycle_angle_rad(start_angle_rad, end_angle_rad, 0.0) {
            self.last_completed_cycle_intake_air_kg =
                Some(self.current_cycle_intake_air_kg.max(0.0));
            self.current_cycle_intake_air_kg = 0.0;
        }
        self.crank_angle_rad = end_angle_rad;
        self.elapsed_time_seconds += timestep_seconds;
        let species_budget = self.species_budget();

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
            chamber_oxygen_mass_kg: self.chamber_species.oxygen_kg,
            chamber_fuel_mass_kg: self.chamber_species.fuel_kg,
            chamber_inert_mass_kg: self.chamber_species.inert_kg,
            chamber_products_mass_kg: self.chamber_species.products_kg,
            chamber_lambda: self.chamber_species.lambda(
                self.definition
                    .combustion
                    .mixture_limits
                    .stoichiometric_air_fuel_ratio,
            ),
            intake_plenum_pressure_pa: self.intake_plenum.pressure_pa(plenum_fallback_properties),
            intake_runner_pressure_pa: self
                .pipes
                .pipe_end_primitive(self.pipes.intake_runner, DuctEnd::Right)
                .p,
            exhaust_runner_pressure_pa: self
                .pipes
                .pipe_end_primitive(self.pipes.exhaust_runner, DuctEnd::Left)
                .p,
            exhaust_collector_pressure_pa: self
                .pipes
                .exhaust_collector
                .map(|collector| self.pipes.pipe_end_primitive(collector, DuctEnd::Left).p),
            exhaust_exit_pressure_pa: self.pipes.exhaust_exit_pressure_pa(),
            intake_effective_area_m2: intake_area_m2,
            exhaust_effective_area_m2: exhaust_area_m2,
            intake_mass_flow_kg_per_s: intake_flux.mass_kg_per_s,
            exhaust_mass_flow_kg_per_s: -exhaust_flux.mass_kg_per_s,
            intake_species_flow_kg_per_s,
            exhaust_species_flow_kg_per_s,
            fuel_injected_kg: combustion_step.fuel_injected_kg,
            fuel_burned_kg: combustion_step.fuel_burned_kg,
            oxygen_consumed_kg: combustion_step.oxygen_consumed_kg,
            products_generated_kg: combustion_step.products_generated_kg,
            species_budget,
            // Exhaust gas is almost entirely combustion products, so lambda
            // must use the combustion-invariant reconstruction (split products
            // back into the air and fuel that formed them) rather than reading
            // the near-zero leftover fuel fraction directly.
            exhaust_lambda: pipe_fractions_to_chamber_species(
                self.pipes
                    .pipe_end_species(self.pipes.exhaust_runner, DuctEnd::Left),
            )
            .lambda(
                self.definition
                    .combustion
                    .mixture_limits
                    .stoichiometric_air_fuel_ratio,
            ),
            gas_force_n,
            indicated_torque_nm,
            load_torque_nm: inputs.external_load_torque_nm,
            combustion_heat_added_j,
            indicated_work_j,
            mean_indicated_torque_nm: self.mean_indicated_torque_nm(),
        }
    }

    fn combustion_step_result(
        &mut self,
        start_angle_rad: f64,
        end_angle_rad: f64,
        crank_speed_rad_per_s: f64,
        combustion_enabled: bool,
        lambda_target: f64,
    ) -> CombustionStepResult {
        let mut internal_energy_before_heat_j = chamber_internal_energy_j(
            self.chamber_state,
            self.chamber_species,
            chamber_properties(&self.definition),
        );
        if !self.definition.combustion.enabled || !combustion_enabled {
            self.pending_combustion = None;
            self.active_combustion = None;
            return CombustionStepResult {
                heat_added_j: 0.0,
                internal_energy_before_heat_j,
                fuel_injected_kg: 0.0,
                fuel_burned_kg: 0.0,
                oxygen_consumed_kg: 0.0,
                products_generated_kg: 0.0,
            };
        }

        let mut fuel_injected_kg = 0.0;
        let spark_angle_deg = spark_angle_deg_for_rpm(
            self.definition.combustion.spark_timing,
            rad_per_s_to_rpm(crank_speed_rad_per_s),
        );
        if self.active_combustion.is_none()
            && self.pending_combustion.is_none()
            && crossed_cycle_angle_rad(start_angle_rad, end_angle_rad, spark_angle_deg.to_radians())
        {
            let fuel_mass_kg = fuel_mass_for_lambda_at_spark(
                &self.definition,
                self.fueling_air_mass_kg(crank_speed_rad_per_s),
                lambda_target,
            );
            self.chamber_species.fuel_kg += fuel_mass_kg;
            fuel_injected_kg += fuel_mass_kg;
            self.species_budget.injected_fuel_kg += fuel_mass_kg;
            internal_energy_before_heat_j += fuel_sensible_internal_energy_j(
                fuel_mass_kg,
                self.definition.boundaries.intake_temperature_k,
                chamber_properties(&self.definition),
            );
            self.pending_combustion = self
                .pending_combustion_at_spark(spark_angle_deg.to_radians(), crank_speed_rad_per_s);
        }

        if let Some(pending) = self.pending_combustion
            && self.active_combustion.is_none()
            && crossed_cycle_angle_rad(start_angle_rad, end_angle_rad, pending.start_angle_rad)
        {
            self.pending_combustion = None;
            self.active_combustion = Some(ActiveCombustion {
                start_angle_rad: pending.start_angle_rad,
                burn_duration_rad: pending.burn_duration_rad,
                burnable_fuel_snapshot_kg: pending.burnable_fuel_snapshot_kg,
                combustion_efficiency: pending.combustion_efficiency,
                previous_wiebe_fraction: 0.0,
                consumed_fuel_kg: 0.0,
                consumed_oxygen_kg: 0.0,
                generated_products_kg: 0.0,
                released_heat_j: 0.0,
            });
        }

        let Some(mut event) = self.active_combustion else {
            return CombustionStepResult {
                heat_added_j: 0.0,
                internal_energy_before_heat_j,
                fuel_injected_kg,
                fuel_burned_kg: 0.0,
                oxygen_consumed_kg: 0.0,
                products_generated_kg: 0.0,
            };
        };

        if event.burnable_fuel_snapshot_kg <= 0.0 || event.combustion_efficiency <= 0.0 {
            self.active_combustion = None;
            return CombustionStepResult {
                heat_added_j: 0.0,
                internal_energy_before_heat_j,
                fuel_injected_kg,
                fuel_burned_kg: 0.0,
                oxygen_consumed_kg: 0.0,
                products_generated_kg: 0.0,
            };
        }

        let elapsed_angle_rad = positive_cycle_delta_rad(event.start_angle_rad, end_angle_rad);
        let wiebe_parameters = WiebeParameters {
            combustion_duration_rad: event.burn_duration_rad,
            ..self.definition.combustion.wiebe
        };
        let cumulative_fraction =
            cumulative_wiebe_burned_fraction(elapsed_angle_rad, wiebe_parameters);
        let burned_fraction_delta = (cumulative_fraction - event.previous_wiebe_fraction).max(0.0);
        event.previous_wiebe_fraction = cumulative_fraction.max(event.previous_wiebe_fraction);
        let requested_fuel_kg = event.burnable_fuel_snapshot_kg * burned_fraction_delta;
        let conversion = burn_species(
            &mut self.chamber_species,
            requested_fuel_kg,
            self.definition
                .combustion
                .mixture_limits
                .stoichiometric_air_fuel_ratio,
            self.definition.combustion.fuel_lower_heating_value_j_per_kg,
            event.combustion_efficiency,
            self.definition.combustion.heat_loss_fraction,
        );
        event.consumed_fuel_kg += conversion.consumed_fuel_kg;
        event.consumed_oxygen_kg += conversion.consumed_oxygen_kg;
        event.generated_products_kg += conversion.generated_products_kg;
        event.released_heat_j += conversion.released_heat_j;
        self.species_budget.burned_fuel_kg += conversion.consumed_fuel_kg;
        self.species_budget.oxygen_consumed_kg += conversion.consumed_oxygen_kg;
        self.species_budget.products_generated_kg += conversion.generated_products_kg;

        if elapsed_angle_rad >= event.burn_duration_rad
            || (requested_fuel_kg > 0.0 && conversion.consumed_fuel_kg <= f64::EPSILON)
        {
            self.active_combustion = None;
        } else {
            self.active_combustion = Some(ActiveCombustion { ..event });
        }

        CombustionStepResult {
            heat_added_j: conversion.released_heat_j,
            internal_energy_before_heat_j,
            fuel_injected_kg,
            fuel_burned_kg: conversion.consumed_fuel_kg,
            oxygen_consumed_kg: conversion.consumed_oxygen_kg,
            products_generated_kg: conversion.generated_products_kg,
        }
    }

    fn pending_combustion_at_spark(
        &self,
        spark_angle_rad: f64,
        crank_speed_rad_per_s: f64,
    ) -> Option<PendingCombustion> {
        let limits = self.definition.combustion.mixture_limits;
        let afr = air_fuel_ratio(
            self.chamber_oxygen_equivalent_air_kg(),
            self.chamber_species.fuel_kg,
        );
        let equivalence_ratio =
            equivalence_ratio_from_air_fuel_ratio(afr, limits.stoichiometric_air_fuel_ratio);
        let mixture_efficiency = mixture_combustion_efficiency(afr, limits);
        if mixture_efficiency <= 0.0 {
            return None;
        }

        let oxygen_fuel_ratio =
            stoichiometric_oxygen_fuel_ratio(limits.stoichiometric_air_fuel_ratio)
                .max(f64::MIN_POSITIVE);
        let burnable_fuel_snapshot_kg = self
            .chamber_species
            .fuel_kg
            .min(self.chamber_species.oxygen_kg / oxygen_fuel_ratio)
            .max(0.0);
        if burnable_fuel_snapshot_kg <= 0.0 {
            return None;
        }

        let residual_fraction = residual_fraction(self.chamber_species);
        let pressure_at_spark_pa = chamber_pressure_pa(
            self.chamber_state,
            ChamberBoundary {
                volume_m3: cylinder_volume_m3(&self.definition, spark_angle_rad),
                volume_rate_m3_per_s: 0.0,
                heat_rate_w: 0.0,
            },
            self.current_chamber_properties(),
        );
        let ignition_delay_rad = ignition_delay_rad(
            IgnitionDelayInputs {
                reference_delay_rad: self.definition.combustion.ignition_delay_deg.to_radians(),
                temperature_k: self.chamber_state.temperature_k,
                pressure_pa: pressure_at_spark_pa,
                equivalence_ratio,
                residual_fraction,
                spark_energy_j: 0.04,
                turbulence_intensity: 0.0,
            },
            limits,
        );
        let burn_duration_rad = burn_duration_rad(
            BurnDurationInputs {
                reference_duration_rad: self.definition.combustion.wiebe.combustion_duration_rad,
                temperature_k: self.chamber_state.temperature_k,
                pressure_pa: pressure_at_spark_pa,
                equivalence_ratio,
                residual_fraction,
                crank_speed_rad_per_s,
                turbulence_intensity: 0.0,
                chamber_characteristic_length_m: self.definition.geometry.bore_m,
            },
            limits,
        );

        let combustion_efficiency = self.definition.combustion.combustion_efficiency
            * mixture_efficiency
            * residual_combustion_efficiency_multiplier(residual_fraction);
        if !ignition_delay_rad.is_finite()
            || !burn_duration_rad.is_finite()
            || combustion_efficiency <= 0.0
        {
            return None;
        }

        Some(PendingCombustion {
            start_angle_rad: normalize_cycle_angle_rad(spark_angle_rad + ignition_delay_rad),
            burn_duration_rad,
            burnable_fuel_snapshot_kg,
            combustion_efficiency,
        })
    }

    fn current_chamber_properties(&self) -> ChamberProperties {
        mixture_chamber_properties(self.chamber_species, chamber_properties(&self.definition))
    }

    fn fueling_air_mass_kg(&self, crank_speed_rad_per_s: f64) -> f64 {
        let minimum_air_kg = self.definition.gas.minimum_mass_kg.max(0.0);
        let current_cycle_air_kg = self.current_cycle_intake_air_kg.max(0.0);
        if current_cycle_air_kg > minimum_air_kg {
            return current_cycle_air_kg;
        }

        let speed_density_air_kg = self.speed_density_intake_air_estimate_kg();
        let rpm = rad_per_s_to_rpm(crank_speed_rad_per_s).abs();
        if rpm < LOW_SPEED_FUELING_FALLBACK_RPM {
            return speed_density_air_kg
                .max(self.chamber_oxygen_equivalent_air_kg())
                .max(0.0);
        }

        // A genuinely tiny-but-measured last-cycle reading (e.g. a
        // near-closed throttle correctly starving induction) must still be
        // trusted over the speed-density fallback: gating on
        // `minimum_air_kg` here would silently re-inflate fueling to a full
        // atmospheric estimate once a real, heavily throttled reading drops
        // below that (near-zero) numerical floor. Only fall back when we
        // have no completed-cycle measurement at all.
        self.last_completed_cycle_intake_air_kg
            .filter(|air_kg| *air_kg > 0.0)
            .unwrap_or_else(|| {
                speed_density_air_kg
                    .max(self.chamber_oxygen_equivalent_air_kg())
                    .max(0.0)
            })
    }

    fn speed_density_intake_air_estimate_kg(&self) -> f64 {
        let intake_valve = ValveEvent::from_definition(self.definition.valves.intake);
        let intake_close_angle_rad = intake_valve.close_angle_rad;
        let volume_m3 = cylinder_volume_m3(&self.definition, intake_close_angle_rad);
        let plenum_properties = chamber_properties(&self.definition);
        let pressure_pa = self.intake_plenum.pressure_pa(plenum_properties).max(1.0);
        let temperature_k = self
            .intake_plenum
            .chamber_state
            .temperature_k
            .max(plenum_properties.minimum_temperature_k);
        let gas_constant = plenum_properties
            .gas_constant_j_per_kg_k
            .max(f64::MIN_POSITIVE);

        pressure_pa * volume_m3 / (gas_constant * temperature_k)
    }

    fn chamber_oxygen_equivalent_air_kg(&self) -> f64 {
        if DRY_AIR_OXYGEN_MASS_FRACTION <= 0.0 {
            return 0.0;
        }

        self.chamber_species.oxygen_kg.max(0.0) / DRY_AIR_OXYGEN_MASS_FRACTION
    }

    fn sync_chamber_mass_from_species(&mut self) {
        self.chamber_species = self.chamber_species.normalized();
        self.chamber_state.mass_kg = self
            .chamber_species
            .total_mass_kg()
            .max(self.definition.gas.minimum_mass_kg);
    }

    fn warn_once_if_chamber_temperature_extreme(&mut self) {
        const WARNING_THRESHOLD_K: f64 = 10_000.0 + 273.15;
        if !self.over_temperature_warning_emitted
            && self.chamber_state.temperature_k > WARNING_THRESHOLD_K
        {
            eprintln!(
                "warning: combustion chamber temperature exceeded 10000 deg C: {:.1} deg C at {:.0} rpm",
                self.chamber_state.temperature_k - 273.15,
                rad_per_s_to_rpm(self.crank_speed_rad_per_s)
            );
            self.over_temperature_warning_emitted = true;
        }
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
    run_fixed_speed_cycles_with_profile(definition, default_runner_profile(), rpm, cycles)
}

pub fn run_fixed_speed_cycles_with_profile(
    definition: &EngineDefinition,
    profile: SimulationProfile,
    rpm: f64,
    cycles: usize,
) -> FixedSpeedRunSummary {
    run_fixed_speed_cycles_with_profile_and_inputs(
        definition,
        profile,
        rpm,
        cycles,
        SingleCylinderStepInputs::default(),
    )
}

pub fn run_fixed_speed_cycles_with_profile_and_inputs(
    definition: &EngineDefinition,
    profile: SimulationProfile,
    rpm: f64,
    cycles: usize,
    inputs: SingleCylinderStepInputs,
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
                ..inputs
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
        default_runner_profile(),
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

pub fn run_fixed_speed_throttle_load_sweep(
    definition: &EngineDefinition,
    profile: SimulationProfile,
    rpm_points: &[f64],
    throttle_effective_area_fractions: &[f64],
    load_torque_points_nm: &[f64],
    cycles_per_point: usize,
) -> Vec<ThrottleLoadSweepPoint> {
    profile.validate();
    let mut points = Vec::new();
    for rpm in rpm_points.iter().copied() {
        for throttle_effective_area_fraction in throttle_effective_area_fractions.iter().copied() {
            for external_load_torque_nm in load_torque_points_nm.iter().copied() {
                let summary = run_fixed_speed_cycles_with_profile_and_inputs(
                    definition,
                    profile,
                    rpm,
                    cycles_per_point,
                    SingleCylinderStepInputs {
                        external_load_torque_nm: external_load_torque_nm.max(0.0),
                        throttle_effective_area_fraction: Some(
                            throttle_effective_area_fraction.clamp(0.0, 1.0),
                        ),
                        ..SingleCylinderStepInputs::default()
                    },
                );
                let angular_speed_rad_per_s = rpm_to_rad_per_s(rpm);
                points.push(ThrottleLoadSweepPoint {
                    rpm,
                    throttle_effective_area_fraction: throttle_effective_area_fraction
                        .clamp(0.0, 1.0),
                    external_load_torque_nm: external_load_torque_nm.max(0.0),
                    mean_indicated_torque_nm: summary.mean_indicated_torque_nm,
                    indicated_power_kw: summary.mean_indicated_torque_nm * angular_speed_rad_per_s
                        / 1000.0,
                });
            }
        }
    }
    points
}

/// Default real-time profile for the convenience runners that are not given an
/// explicit profile or handling config. Uses the default handling timestep,
/// which equals the value the removed `EngineDefinition.simulation` carried.
fn default_runner_profile() -> SimulationProfile {
    EngineHandlingDefinition::default().to_profile()
}

fn default_intake_plenum(definition: &EngineDefinition) -> PlenumState {
    PlenumState::from_pressure_temperature(
        definition.boundaries.intake_pressure_pa,
        definition.boundaries.intake_temperature_k,
        definition
            .intake_exhaust
            .intake_plenum_volume_m3
            .max(1.0e-6),
        chamber_properties(definition),
    )
}

fn default_pipe_network(definition: &EngineDefinition) -> OneDPipeNetwork {
    let gas = TemperatureDependentAir::new();
    let mut model = Model::new(0.5);
    let intake = definition.intake_exhaust.intake_runner;
    let exhaust = definition.intake_exhaust.exhaust_runner;
    let intake_initial = pipe_state_from_pressure_temperature(
        definition.boundaries.intake_pressure_pa,
        definition.boundaries.intake_temperature_k,
        gas,
    );
    let exhaust_initial = pipe_state_from_pressure_temperature(
        definition.boundaries.exhaust_pressure_pa,
        definition.boundaries.exhaust_temperature_k,
        gas,
    );
    let intake_runner = model.add_uniform_duct_with_species(
        gas,
        duct_config(intake),
        intake_initial,
        SpeciesFractions::AIR,
        ModelBoundary::external(INTAKE_PLENUM_EXTERNAL_ID),
        ModelBoundary::external(INTAKE_VALVE_EXTERNAL_ID),
    );
    let exhaust_right_boundary = if definition.intake_exhaust.exhaust_collector.is_some() {
        ModelBoundary::junction(0)
    } else {
        ModelBoundary::open(definition.boundaries.exhaust_pressure_pa)
    };
    let exhaust_runner = model.add_uniform_duct_with_species(
        gas,
        duct_config(exhaust),
        exhaust_initial,
        SpeciesFractions::EXHAUST,
        ModelBoundary::external(EXHAUST_VALVE_EXTERNAL_ID),
        exhaust_right_boundary,
    );
    let exhaust_collector = definition
        .intake_exhaust
        .exhaust_collector
        .map(|collector| {
            model.add_uniform_duct_with_species(
                gas,
                duct_config(collector),
                exhaust_initial,
                SpeciesFractions::EXHAUST,
                ModelBoundary::junction(0),
                ModelBoundary::open(definition.boundaries.exhaust_pressure_pa),
            )
        });

    OneDPipeNetwork {
        gas,
        model,
        intake_runner,
        exhaust_runner,
        exhaust_collector,
    }
}

/// onedpipes' finite-volume reconstruction needs at least this many cells
/// (`DuctConfig::new` asserts `cells >= 4`), so coarser configurations are
/// promoted rather than allowed to panic.
const MIN_DUCT_CELLS: usize = 4;
/// Per-cell length floor, carried over from the old `Pipe1D` construction.
const MIN_DUCT_CELL_LENGTH_M: f64 = 1.0e-4;

fn duct_config(pipe: crate::engine_config::PipeDefinition) -> DuctConfig {
    let cells = pipe.number_of_cells.max(MIN_DUCT_CELLS);
    DuctConfig::new(
        pipe.total_length_m
            .max(cells as f64 * MIN_DUCT_CELL_LENGTH_M),
        cells,
        pipe.area_m2.max(1.0e-6),
    )
}

impl OneDPipeNetwork {
    fn pipe_area_m2(&self, pipe_id: PipeId) -> f64 {
        self.model.pipe(pipe_id).config().area
    }

    fn pipe_end_cell_mass_kg(&self, pipe_id: PipeId, end: DuctEnd) -> f64 {
        let duct = self.model.pipe(pipe_id);
        let state = self.pipe_end_state(pipe_id, end);
        state.rho.max(0.0) * duct.config().area * duct.config().dx()
    }

    fn boundary_request(
        &self,
        pipe_id: PipeId,
        end: DuctEnd,
        external_state: PipeState,
        flow_area_m2: f64,
        inflow_species: SpeciesFractions,
        cell_fraction: f64,
    ) -> EnginePipeBoundaryRequest {
        EnginePipeBoundaryRequest {
            pipe_id,
            end,
            external_state,
            flow_area_m2,
            inflow_species,
            cell_fraction,
        }
    }

    fn pipe_end_state(&self, pipe_id: PipeId, end: DuctEnd) -> PipeState {
        self.model.pipe_end_state(PipeEnd { pipe_id, end })
    }

    fn pipe_end_species(&self, pipe_id: PipeId, end: DuctEnd) -> SpeciesFractions {
        self.model.pipe_end_species(PipeEnd { pipe_id, end })
    }

    fn pipe_end_primitive(&self, pipe_id: PipeId, end: DuctEnd) -> onedpipes::Primitive {
        self.pipe_end_state(pipe_id, end)
            .primitive_clamped(self.gas)
    }

    fn exhaust_exit_pressure_pa(&self) -> f64 {
        let exit_pipe = self.exhaust_collector.unwrap_or(self.exhaust_runner);
        self.pipe_end_primitive(exit_pipe, DuctEnd::Right).p
    }

    fn step_stable(
        &mut self,
        timestep_seconds: f64,
        intake_plenum_request: EnginePipeBoundaryRequest,
        intake_valve_request: EnginePipeBoundaryRequest,
        exhaust_valve_request: EnginePipeBoundaryRequest,
        plenum_volume_m3: f64,
        plenum_donor_budget_kg: f64,
        chamber_donor_budget_kg: f64,
    ) -> [AcceptedPipeTransfer; 3] {
        if timestep_seconds <= 0.0 {
            return [AcceptedPipeTransfer::default(); 3];
        }

        let mut requests = [
            (INTAKE_PLENUM_EXTERNAL_ID, intake_plenum_request),
            (INTAKE_VALVE_EXTERNAL_ID, intake_valve_request),
            (EXHAUST_VALVE_EXTERNAL_ID, exhaust_valve_request),
        ];
        let plenum_primitive = requests[0].1.external_state.primitive_clamped(self.gas);
        let mut plenum_external_mass_kg =
            (plenum_primitive.rho * plenum_volume_m3.max(0.0)).max(0.0);
        let plenum_external_temperature_k = plenum_primitive.temperature.max(1.0);
        let finite_plenum_feedback = plenum_volume_m3 > 0.0 && plenum_donor_budget_kg.is_finite();
        // Remaining mass each donor volume may still push into the pipes this
        // step; request 0 draws on the plenum, requests 1 and 2 share the
        // chamber.
        let mut plenum_budget_kg = plenum_donor_budget_kg.max(0.0);
        let mut chamber_budget_kg = chamber_donor_budget_kg.max(0.0);
        let mut cell_transfer_budgets_kg = requests.map(|(_, request)| {
            request.cell_fraction.max(0.0)
                * self
                    .pipe_end_cell_mass_kg(request.pipe_id, request.end)
                    .max(1.0e-9)
        });
        let mut accepted = [AcceptedPipeTransfer::default(); 3];
        let mut remaining_seconds = timestep_seconds;
        while remaining_seconds > f64::EPSILON {
            let stable_dt = self.model.stable_timestep();
            let substep_seconds = if stable_dt.is_finite() && stable_dt > 0.0 {
                remaining_seconds.min(stable_dt)
            } else {
                remaining_seconds
            };
            self.model.clear_external_boundary_controls();
            for (index, (external_id, request)) in requests.iter().copied().enumerate() {
                // Re-evaluate the orifice flow from the live pipe end state so
                // the request itself decays as the boundary cell equalizes
                // with the (frozen) external state, instead of a stale
                // start-of-step rate being forced in all step long.
                let mut flow = pipe_to_external_orifice_flow(
                    self.pipe_end_state(request.pipe_id, request.end),
                    request.external_state,
                    request.flow_area_m2,
                    self.gas,
                );
                // Negative flow draws mass from the donor volume into the
                // pipe; never draw more than the donor has left to give.
                let donor_budget_kg = if index == 0 {
                    plenum_budget_kg
                } else {
                    chamber_budget_kg
                };
                let inflow_mass_kg = -flow.mass_kg_per_s * substep_seconds;
                if inflow_mass_kg > donor_budget_kg {
                    let scale = (donor_budget_kg / inflow_mass_kg).clamp(0.0, 1.0);
                    flow.mass_kg_per_s *= scale;
                    flow.energy_w *= scale;
                }
                // Cap each macro-step transfer at a fraction of the boundary
                // cell mass. Reusing the full cap for every solver substep
                // lets external boundaries pump more than the intended
                // boundary-cell inventory during one engine step.
                let max_mass_transfer_kg = cell_transfer_budgets_kg[index].max(0.0);
                self.set_external_flow(external_id, request, flow, max_mass_transfer_kg);
            }
            let report = self.model.step_with_dt(substep_seconds);
            for diagnostic in report.external_boundary_diagnostics {
                let Some(index) = external_transfer_index(diagnostic.external_id) else {
                    continue;
                };
                accepted[index].absorb(AcceptedPipeTransfer {
                    mass_kg: diagnostic.mass_transferred_out,
                    energy_j: diagnostic.energy_transferred_out,
                    species_kg: diagnostic.species_transferred_out,
                });
                cell_transfer_budgets_kg[index] = (cell_transfer_budgets_kg[index]
                    - diagnostic.mass_transferred_out.abs())
                .max(0.0);
                // Positive transfer is pipe -> external; negative drew mass
                // from the donor volume.
                let drawn_kg = (-diagnostic.mass_transferred_out).max(0.0);
                if index == 0 {
                    plenum_budget_kg = (plenum_budget_kg - drawn_kg).max(0.0);
                    if finite_plenum_feedback {
                        plenum_external_mass_kg = (plenum_external_mass_kg
                            + diagnostic.mass_transferred_out)
                            .max(1.0e-12);
                        let pressure_pa = (plenum_external_mass_kg
                            * self.gas.r()
                            * plenum_external_temperature_k
                            / plenum_volume_m3)
                            .max(1.0);
                        requests[0].1.external_state = pipe_state_from_pressure_temperature(
                            pressure_pa,
                            plenum_external_temperature_k,
                            self.gas,
                        );
                    }
                } else {
                    chamber_budget_kg = (chamber_budget_kg - drawn_kg).max(0.0);
                }
            }
            remaining_seconds -= substep_seconds;
        }
        accepted
    }

    fn set_external_flow(
        &mut self,
        external_id: usize,
        request: EnginePipeBoundaryRequest,
        flow: EnginePipeFlow,
        max_mass_transfer: f64,
    ) {
        let max_energy_transfer = if flow.mass_kg_per_s.abs() > 0.0 {
            max_mass_transfer * (flow.energy_w / flow.mass_kg_per_s).abs()
        } else {
            0.0
        };
        self.model.set_external_boundary_control(
            ExternalBoundaryId(external_id),
            ExternalBoundaryControl::BoundedFlow {
                mass_flow_out: flow.mass_kg_per_s,
                energy_flow_out: flow.energy_w,
                max_mass_transfer,
                max_energy_transfer,
                inflow_species: request.inflow_species,
            },
        );
    }
}

fn pipe_state_from_pressure_temperature(
    pressure_pa: f64,
    temperature_k: f64,
    gas: TemperatureDependentAir,
) -> PipeState {
    PipeState::from_primitive(
        pressure_pa.max(1.0) / (gas.r() * temperature_k.max(1.0)),
        0.0,
        pressure_pa.max(1.0),
        gas,
    )
}

fn pipe_to_external_orifice_flow(
    pipe_state: PipeState,
    external_state: PipeState,
    flow_area_m2: f64,
    gas: TemperatureDependentAir,
) -> EnginePipeFlow {
    if flow_area_m2 <= 0.0 {
        return EnginePipeFlow::zero();
    }

    let flow = ValveOrifice::new(1.0, flow_area_m2).mass_flow(
        physical_pipe_state(pipe_state, gas),
        physical_pipe_state(external_state, gas),
        gas,
    );
    EnginePipeFlow {
        mass_kg_per_s: flow.mass_flow,
        energy_w: flow.energy_flow,
    }
}

fn physical_pipe_state(state: PipeState, gas: TemperatureDependentAir) -> PipeState {
    if state.try_primitive(gas).is_ok() {
        return state;
    }

    let primitive = state.primitive_clamped(gas);
    let rho = if primitive.rho.is_finite() {
        primitive.rho.max(1.0e-8)
    } else {
        1.0e-8
    };
    let velocity = if primitive.u.is_finite() {
        primitive.u
    } else {
        0.0
    };
    let pressure = if primitive.p.is_finite() {
        primitive.p.max(1.0)
    } else {
        1.0
    };
    PipeState::from_primitive(rho, velocity, pressure, gas)
}

fn chamber_species_to_pipe_fractions(species: ChamberSpeciesMasses) -> SpeciesFractions {
    // SpeciesFractions::new normalizes by the sum and falls back to AIR when
    // the total is non-positive, so raw masses can be passed straight through.
    SpeciesFractions::new(
        species.oxygen_kg,
        species.fuel_kg,
        species.inert_kg,
        species.products_kg,
    )
}

/// Inverse of [`chamber_species_to_pipe_fractions`]: mass fractions reinterpreted
/// as masses, which is exact for ratio-based consumers like `lambda`.
fn pipe_fractions_to_chamber_species(fractions: SpeciesFractions) -> ChamberSpeciesMasses {
    ChamberSpeciesMasses {
        oxygen_kg: fractions.oxygen,
        fuel_kg: fractions.fuel_vapor,
        inert_kg: fractions.inert,
        products_kg: fractions.products,
    }
}

fn species_mass_to_chamber_species(
    species: SpeciesMass,
    timestep_seconds: f64,
) -> ChamberSpeciesMasses {
    if timestep_seconds <= 0.0 {
        return ChamberSpeciesMasses::default();
    }

    ChamberSpeciesMasses {
        oxygen_kg: species.oxygen / timestep_seconds,
        fuel_kg: species.fuel_vapor / timestep_seconds,
        inert_kg: species.inert / timestep_seconds,
        products_kg: species.products / timestep_seconds,
    }
}

fn external_transfer_index(external_id: usize) -> Option<usize> {
    match external_id {
        INTAKE_PLENUM_EXTERNAL_ID => Some(0),
        INTAKE_VALVE_EXTERNAL_ID => Some(1),
        EXHAUST_VALVE_EXTERNAL_ID => Some(2),
        _ => None,
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

fn step_chamber_with_pipe_fluxes(
    species: &mut ChamberSpeciesMasses,
    state: ChamberState,
    fallback_properties: ChamberProperties,
    mut fluxes: [PipeBoundaryFlux; 2],
    heat_added_j: f64,
    timestep_seconds: f64,
    volume_at: impl Fn(f64) -> (f64, f64),
) -> ChamberState {
    limit_chamber_outflow_fluxes(
        species.total_mass_kg(),
        fallback_properties,
        &mut fluxes,
        timestep_seconds,
    );
    let properties = mixture_chamber_properties(*species, fallback_properties);
    let cv_j_per_kg_k = cv(
        properties.gas_constant_j_per_kg_k,
        properties.specific_heat_ratio,
    );
    let internal_energy_j = state.mass_kg * cv_j_per_kg_k * state.temperature_k;
    let energy_flux_rate_w = fluxes.iter().map(|flux| flux.energy_w).sum::<f64>();
    let heat_rate_w = if timestep_seconds > 0.0 {
        heat_added_j / timestep_seconds
    } else {
        0.0
    };
    // RK4-integrate the internal energy over the moving cylinder volume instead
    // of the previous explicit-Euler piston-work term (P at the start angle).
    let updated_internal_energy_j = integrate_internal_energy_rk4(
        internal_energy_j,
        energy_flux_rate_w,
        heat_rate_w,
        properties.specific_heat_ratio,
        timestep_seconds,
        volume_at,
    );

    for flux in fluxes {
        species.oxygen_kg += flux.species_kg_per_s.oxygen_kg * timestep_seconds;
        species.fuel_kg += flux.species_kg_per_s.fuel_kg * timestep_seconds;
        species.inert_kg += flux.species_kg_per_s.inert_kg * timestep_seconds;
        species.products_kg += flux.species_kg_per_s.products_kg * timestep_seconds;
    }
    *species = species.normalized();

    let updated_properties = mixture_chamber_properties(*species, fallback_properties);
    let updated_cv_j_per_kg_k = cv(
        updated_properties.gas_constant_j_per_kg_k,
        updated_properties.specific_heat_ratio,
    );
    let updated_mass_kg = species
        .total_mass_kg()
        .max(fallback_properties.minimum_mass_kg);
    let updated_temperature_k = if updated_mass_kg > 0.0 && updated_cv_j_per_kg_k > 0.0 {
        (updated_internal_energy_j / (updated_mass_kg * updated_cv_j_per_kg_k))
            .max(fallback_properties.minimum_temperature_k)
    } else {
        fallback_properties.minimum_temperature_k
    };

    ChamberState {
        mass_kg: updated_mass_kg,
        temperature_k: updated_temperature_k,
    }
}

fn chamber_internal_energy_j(
    state: ChamberState,
    species: ChamberSpeciesMasses,
    fallback_properties: ChamberProperties,
) -> f64 {
    let properties = mixture_chamber_properties(species, fallback_properties);
    let cv_j_per_kg_k = cv(
        properties.gas_constant_j_per_kg_k,
        properties.specific_heat_ratio,
    );
    species
        .total_mass_kg()
        .max(fallback_properties.minimum_mass_kg)
        * cv_j_per_kg_k
        * state.temperature_k
}

fn chamber_state_from_internal_energy(
    species: ChamberSpeciesMasses,
    previous_state: ChamberState,
    internal_energy_j: f64,
    fallback_properties: ChamberProperties,
) -> ChamberState {
    let species = species.normalized();
    let mass_kg = species
        .total_mass_kg()
        .max(fallback_properties.minimum_mass_kg);
    let properties = mixture_chamber_properties(species, fallback_properties);
    let cv_j_per_kg_k = cv(
        properties.gas_constant_j_per_kg_k,
        properties.specific_heat_ratio,
    );
    let temperature_k = if mass_kg > 0.0 && cv_j_per_kg_k > 0.0 {
        (internal_energy_j.max(0.0) / (mass_kg * cv_j_per_kg_k))
            .max(fallback_properties.minimum_temperature_k)
    } else {
        previous_state
            .temperature_k
            .max(fallback_properties.minimum_temperature_k)
    };

    ChamberState {
        mass_kg,
        temperature_k,
    }
}

fn limit_chamber_outflow_fluxes(
    available_mass_kg: f64,
    fallback_properties: ChamberProperties,
    fluxes: &mut [PipeBoundaryFlux; 2],
    timestep_seconds: f64,
) {
    if timestep_seconds <= 0.0 {
        return;
    }

    let incoming_mass_kg = fluxes
        .iter()
        .filter(|flux| flux.mass_kg_per_s > 0.0)
        .map(|flux| flux.mass_kg_per_s * timestep_seconds)
        .sum::<f64>();
    let requested_outgoing_mass_kg = fluxes
        .iter()
        .filter(|flux| flux.mass_kg_per_s < 0.0)
        .map(|flux| -flux.mass_kg_per_s * timestep_seconds)
        .sum::<f64>();
    let removable_mass_kg =
        (available_mass_kg + incoming_mass_kg - fallback_properties.minimum_mass_kg).max(0.0);
    if requested_outgoing_mass_kg <= removable_mass_kg || requested_outgoing_mass_kg <= 0.0 {
        return;
    }

    let scale = removable_mass_kg / requested_outgoing_mass_kg;
    for flux in fluxes.iter_mut().filter(|flux| flux.mass_kg_per_s < 0.0) {
        *flux = flux.scaled(scale);
    }
}

fn scaled_species_flow(species: ChamberSpeciesMasses, scale: f64) -> ChamberSpeciesMasses {
    ChamberSpeciesMasses {
        oxygen_kg: species.oxygen_kg * scale,
        fuel_kg: species.fuel_kg * scale,
        inert_kg: species.inert_kg * scale,
        products_kg: species.products_kg * scale,
    }
}

fn net_plenum_supplied_air_delta_kg(plenum_flux: PipeBoundaryFlux, timestep_seconds: f64) -> f64 {
    if timestep_seconds <= 0.0 || DRY_AIR_OXYGEN_MASS_FRACTION <= 0.0 {
        return 0.0;
    }

    -plenum_flux.species_kg_per_s.oxygen_kg * timestep_seconds / DRY_AIR_OXYGEN_MASS_FRACTION
}

fn fuel_mass_for_lambda_at_spark(
    definition: &EngineDefinition,
    chamber_air_mass_kg: f64,
    lambda_target: f64,
) -> f64 {
    let stoich_air_fuel_ratio = definition
        .combustion
        .mixture_limits
        .stoichiometric_air_fuel_ratio;
    if chamber_air_mass_kg <= 0.0 || stoich_air_fuel_ratio <= 0.0 {
        return 0.0;
    }

    chamber_air_mass_kg / (stoich_air_fuel_ratio * lambda_target.clamp(0.1, 2.0))
}

fn fuel_sensible_internal_energy_j(
    fuel_mass_kg: f64,
    temperature_k: f64,
    fallback_properties: ChamberProperties,
) -> f64 {
    if fuel_mass_kg <= 0.0 {
        return 0.0;
    }

    species_internal_energy_j(
        ChamberSpeciesMasses {
            oxygen_kg: 0.0,
            fuel_kg: fuel_mass_kg,
            inert_kg: 0.0,
            products_kg: 0.0,
        },
        temperature_k,
        fallback_properties,
    )
}

fn residual_fraction(species: ChamberSpeciesMasses) -> f64 {
    let total_mass_kg = species.total_mass_kg();
    if total_mass_kg <= 0.0 {
        return 0.0;
    }

    (species.products_kg / total_mass_kg).clamp(0.0, 1.0)
}

fn residual_combustion_efficiency_multiplier(residual_fraction: f64) -> f64 {
    (1.0 - 0.6 * residual_fraction.clamp(0.0, 1.0)).clamp(0.0, 1.0)
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
    use crate::engine_config::{ValveLiftProfileDefinition, ValveLiftProfileModel};
    use crate::physics::chamber::{
        ChamberDerivatives, ChamberSpeciesMasses, FlowBoundary, chamber_derivatives,
    };
    use crate::physics::gas::cp;

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

    fn run_controlled_cycles(
        definition: &EngineDefinition,
        rpm: f64,
        cycles: usize,
        mut inputs: SingleCylinderStepInputs,
    ) -> (f64, f64, f64) {
        let mut engine = SingleCylinderEngine::from_definition(definition.clone());
        engine.reset_accumulators();
        let crank_speed_rad_per_s = rpm_to_rad_per_s(rpm);
        inputs.fixed_crank_speed_rad_per_s = Some(crank_speed_rad_per_s);

        let mut max_temperature_k = 0.0_f64;
        let mut heat_added_j = 0.0_f64;
        let mut remaining_angle_rad = ENGINE_CYCLE_RADIANS * cycles as f64;
        while remaining_angle_rad > f64::EPSILON {
            let step_angle_rad = (crank_speed_rad_per_s * engine.profile().timestep_seconds)
                .min(remaining_angle_rad);
            let output = engine.step_with_timestep(step_angle_rad / crank_speed_rad_per_s, inputs);
            max_temperature_k = max_temperature_k.max(output.cylinder_temperature_k);
            heat_added_j += output.combustion_heat_added_j;
            remaining_angle_rad -= step_angle_rad;
        }

        (
            max_temperature_k,
            heat_added_j,
            engine.mean_indicated_torque_nm(),
        )
    }

    #[test]
    fn loads_gn250_engine_definition() {
        let definition = definition();

        assert_eq!(definition.metadata.name, "Suzuki GN250 approximation");
        assert_eq!(definition.geometry.bore_m, 0.072);
        assert_eq!(definition.geometry.stroke_m, 0.0612);
    }

    #[test]
    fn pipe_orifice_flow_sign_matches_engine_boundary_convention() {
        let gas = TemperatureDependentAir::new();
        let high_pressure = pipe_state_from_pressure_temperature(120_000.0, 300.0, gas);
        let low_pressure = pipe_state_from_pressure_temperature(100_000.0, 300.0, gas);

        let pipe_to_external =
            pipe_to_external_orifice_flow(high_pressure, low_pressure, 1.0e-4, gas);
        assert!(pipe_to_external.mass_kg_per_s > 0.0);
        assert!(pipe_to_external.energy_w > 0.0);

        let external_to_pipe =
            pipe_to_external_orifice_flow(low_pressure, high_pressure, 1.0e-4, gas);
        assert!(external_to_pipe.mass_kg_per_s < 0.0);
        assert!(external_to_pipe.energy_w < 0.0);

        assert_eq!(
            pipe_to_external_orifice_flow(high_pressure, low_pressure, 0.0, gas),
            EnginePipeFlow::zero()
        );
    }

    #[test]
    fn chamber_pipe_outflow_removes_mass_and_does_not_heat_the_charge() {
        let definition = definition();
        let properties = chamber_properties(&definition);
        let mut species = ChamberSpeciesMasses::from_dry_air_mass(1.0e-3);
        let state = ChamberState {
            mass_kg: species.total_mass_kg(),
            temperature_k: 600.0,
        };
        let mass_rate = -1.0e-4;
        let timestep_seconds = 0.01;
        let flux = PipeBoundaryFlux {
            mass_kg_per_s: mass_rate,
            momentum_n: 0.0,
            energy_w: mass_rate
                * cp(
                    properties.gas_constant_j_per_kg_k,
                    properties.specific_heat_ratio,
                )
                * state.temperature_k,
            species_kg_per_s: ChamberSpeciesMasses {
                oxygen_kg: mass_rate.abs() * -DRY_AIR_OXYGEN_MASS_FRACTION,
                fuel_kg: 0.0,
                inert_kg: mass_rate.abs() * -(1.0 - DRY_AIR_OXYGEN_MASS_FRACTION),
                products_kg: 0.0,
            },
        };

        let updated = step_chamber_with_pipe_fluxes(
            &mut species,
            state,
            properties,
            [flux, PipeBoundaryFlux::zero()],
            0.0,
            timestep_seconds,
            |_| (1.0e-4, 0.0),
        );

        assert_approx_eq(updated.mass_kg, 9.99e-4, 1.0e-12);
        assert!(updated.temperature_k.is_finite());
        assert!(
            updated.temperature_k < state.temperature_k,
            "enthalpy outflow should not heat a fixed-volume charge: {} K -> {} K",
            state.temperature_k,
            updated.temperature_k
        );
    }

    #[test]
    fn chamber_pipe_cool_inflow_adds_mass_and_cools_the_charge() {
        let definition = definition();
        let properties = chamber_properties(&definition);
        let mut species = ChamberSpeciesMasses::from_dry_air_mass(1.0e-3);
        let state = ChamberState {
            mass_kg: species.total_mass_kg(),
            temperature_k: 600.0,
        };
        let mass_rate = 1.0e-4;
        let timestep_seconds = 0.01;
        let source_temperature_k = 300.0;
        let flux = PipeBoundaryFlux {
            mass_kg_per_s: mass_rate,
            momentum_n: 0.0,
            energy_w: mass_rate
                * cp(
                    properties.gas_constant_j_per_kg_k,
                    properties.specific_heat_ratio,
                )
                * source_temperature_k,
            species_kg_per_s: ChamberSpeciesMasses::from_dry_air_mass(mass_rate),
        };

        let updated = step_chamber_with_pipe_fluxes(
            &mut species,
            state,
            properties,
            [flux, PipeBoundaryFlux::zero()],
            0.0,
            timestep_seconds,
            |_| (1.0e-4, 0.0),
        );

        assert_approx_eq(updated.mass_kg, 1.001e-3, 1.0e-12);
        assert!(updated.temperature_k.is_finite());
        assert!(
            updated.temperature_k < state.temperature_k,
            "cool inflow should reduce fixed-volume chamber temperature: {} K -> {} K",
            state.temperature_k,
            updated.temperature_k
        );
    }

    #[test]
    fn rpm_conversions_round_trip() {
        assert_approx_eq(rad_per_s_to_rpm(rpm_to_rad_per_s(3000.0)), 3000.0, EPSILON);
    }

    #[test]
    fn spark_curve_converts_btdc_advance_to_cycle_angle() {
        let timing = definition().combustion.spark_timing;

        assert_approx_eq(spark_angle_deg_for_rpm(timing, 1600.0), 710.0, EPSILON);
        assert_approx_eq(spark_angle_deg_for_rpm(timing, 3000.0), 690.0, EPSILON);
        assert_approx_eq(spark_angle_deg_for_rpm(timing, 2350.0), 700.0, EPSILON);
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
    fn lambda_target_scales_spark_fueling_from_trapped_air() {
        let definition = definition();
        let air_mass_kg = 0.00147;

        assert!(
            fuel_mass_for_lambda_at_spark(&definition, air_mass_kg, 1.0)
                > fuel_mass_for_lambda_at_spark(&definition, air_mass_kg, 2.0)
        );
        assert_approx_eq(
            fuel_mass_for_lambda_at_spark(&definition, air_mass_kg, 0.1),
            air_mass_kg
                / (definition
                    .combustion
                    .mixture_limits
                    .stoichiometric_air_fuel_ratio
                    * 0.1),
            EPSILON,
        );
    }

    #[test]
    fn spark_fueling_uses_current_cycle_intake_air_before_residual_air() {
        let mut engine = SingleCylinderEngine::from_definition(definition());
        engine.current_cycle_intake_air_kg = 0.00020;
        engine.last_completed_cycle_intake_air_kg = Some(0.00010);
        engine.chamber_species = ChamberSpeciesMasses {
            oxygen_kg: 0.00050,
            inert_kg: 0.00200,
            fuel_kg: 0.0,
            products_kg: 0.00100,
        };

        assert_approx_eq(
            engine.fueling_air_mass_kg(rpm_to_rad_per_s(3000.0)),
            0.00020,
            EPSILON,
        );
    }

    #[test]
    fn normal_running_fueling_can_seed_from_last_cycle_when_current_cycle_is_unknown() {
        let mut engine = SingleCylinderEngine::from_definition(definition());
        engine.current_cycle_intake_air_kg = 0.0;
        engine.last_completed_cycle_intake_air_kg = Some(0.00018);
        engine.chamber_species = ChamberSpeciesMasses {
            oxygen_kg: 0.00050,
            inert_kg: 0.00200,
            fuel_kg: 0.0,
            products_kg: 0.00100,
        };

        assert_approx_eq(
            engine.fueling_air_mass_kg(rpm_to_rad_per_s(3000.0)),
            0.00018,
            EPSILON,
        );
    }

    #[test]
    fn low_speed_fueling_ignores_stale_last_cycle_air_estimate() {
        let mut engine = SingleCylinderEngine::from_definition(definition());
        engine.current_cycle_intake_air_kg = 0.0;
        engine.last_completed_cycle_intake_air_kg = Some(0.010);
        engine.chamber_species = ChamberSpeciesMasses::default();
        let speed_density_air_kg = engine.speed_density_intake_air_estimate_kg();

        assert_approx_eq(
            engine.fueling_air_mass_kg(rpm_to_rad_per_s(50.0)),
            speed_density_air_kg,
            speed_density_air_kg.abs().max(1.0) * 1.0e-12,
        );
    }

    #[test]
    fn intake_air_accumulator_uses_net_plenum_supplied_oxygen_equivalent_air_only() {
        let species_flow = ChamberSpeciesMasses {
            oxygen_kg: DRY_AIR_OXYGEN_MASS_FRACTION * 0.020,
            inert_kg: 10.0,
            fuel_kg: 0.0,
            products_kg: 5.0,
        };
        let plenum_to_runner_flux = PipeBoundaryFlux {
            mass_kg_per_s: -0.020,
            momentum_n: 0.0,
            energy_w: -1.0,
            species_kg_per_s: ChamberSpeciesMasses {
                oxygen_kg: -DRY_AIR_OXYGEN_MASS_FRACTION * 0.020,
                ..species_flow
            },
        };

        assert_approx_eq(
            net_plenum_supplied_air_delta_kg(plenum_to_runner_flux, 0.5),
            0.010,
            EPSILON,
        );
        assert_approx_eq(
            net_plenum_supplied_air_delta_kg(
                PipeBoundaryFlux {
                    species_kg_per_s: ChamberSpeciesMasses {
                        oxygen_kg: DRY_AIR_OXYGEN_MASS_FRACTION * 0.020,
                        ..species_flow
                    },
                    ..plenum_to_runner_flux
                },
                0.5,
            ),
            -0.010,
            EPSILON,
        );
    }

    #[test]
    fn rich_misfire_does_not_drive_unbounded_temperature() {
        let definition = definition();
        let inputs = SingleCylinderStepInputs {
            lambda_target: 0.1,
            throttle_position: 1.0,
            ..SingleCylinderStepInputs::default()
        };

        let (max_temperature_k, heat_added_j, _) =
            run_controlled_cycles(&definition, 3000.0, 120, inputs);

        assert_approx_eq(heat_added_j, 0.0, 1.0e-9);
        assert!(
            max_temperature_k < 2_500.0,
            "rich misfire reached implausible {max_temperature_k} K"
        );
    }

    #[test]
    fn throttled_rich_misfire_stays_cooler_than_combustion_temperatures() {
        let definition = definition();
        let inputs = SingleCylinderStepInputs {
            lambda_target: 0.1,
            throttle_position: 0.15,
            ..SingleCylinderStepInputs::default()
        };

        let (max_temperature_k, heat_added_j, _) =
            run_controlled_cycles(&definition, 3000.0, 120, inputs);

        assert_approx_eq(heat_added_j, 0.0, 1.0e-9);
        assert!(
            max_temperature_k < 2_500.0,
            "throttled rich misfire reached implausible {max_temperature_k} K"
        );
    }

    #[test]
    fn stoichiometric_operation_has_bounded_temperature() {
        let definition = definition();
        let inputs = SingleCylinderStepInputs {
            lambda_target: 1.0,
            throttle_position: 1.0,
            ..SingleCylinderStepInputs::default()
        };

        let (max_temperature_k, _, mean_torque_nm) =
            run_controlled_cycles(&definition, 3000.0, 120, inputs);

        assert!(
            max_temperature_k < 6_000.0,
            "stoichiometric operation reached implausible {max_temperature_k} K"
        );
        assert!(mean_torque_nm.is_finite());
    }

    #[test]
    fn very_rich_high_rpm_case_stays_bounded_and_does_not_panic() {
        let definition = definition();
        let inputs = SingleCylinderStepInputs {
            lambda_target: 0.1,
            throttle_position: 1.0,
            ..SingleCylinderStepInputs::default()
        };

        let (max_temperature_k, heat_added_j, mean_torque_nm) =
            run_controlled_cycles(&definition, 7100.0, 120, inputs);

        assert_approx_eq(heat_added_j, 0.0, 1.0e-9);
        assert!(mean_torque_nm.is_finite());
        assert!(
            max_temperature_k < 2_500.0,
            "7100 rpm rich misfire reached implausible {max_temperature_k} K"
        );
    }

    #[test]
    fn high_rpm_rich_case_does_not_run_away_temperature() {
        let definition = definition();
        let inputs = SingleCylinderStepInputs {
            lambda_target: 0.35,
            throttle_position: 1.0,
            ..SingleCylinderStepInputs::default()
        };

        let (max_temperature_k, heat_added_j, mean_torque_nm) =
            run_controlled_cycles(&definition, 6000.0, 120, inputs);

        assert!(heat_added_j.is_finite());
        assert!(mean_torque_nm.is_finite());
        assert!(
            max_temperature_k < 6_000.0,
            "6000 rpm lambda 0.35 reached implausible {max_temperature_k} K"
        );
    }

    #[test]
    fn forced_map_pins_intake_plenum_pressure() {
        let definition = definition();
        let mut engine = SingleCylinderEngine::from_definition(definition);
        let target_pa = 60_000.0;
        let crank_speed_rad_per_s = rpm_to_rad_per_s(3000.0);

        // Step a few cycles with the ideal MAP override pinned well below ambient.
        let mut last_map_pa = 0.0;
        for _ in 0..4_000 {
            let output = engine.step(SingleCylinderStepInputs {
                fixed_crank_speed_rad_per_s: Some(crank_speed_rad_per_s),
                forced_map_pressure_pa: Some(target_pa),
                throttle_position: 1.0,
                ..SingleCylinderStepInputs::default()
            });
            last_map_pa = output.intake_plenum_pressure_pa;
        }

        // The plenum is re-pinned every step, so reported MAP tracks the target
        // closely despite WOT and the runner drawing charge from it.
        assert!(
            (last_map_pa - target_pa).abs() < 3_000.0,
            "forced MAP should hold near {target_pa} Pa, got {last_map_pa} Pa"
        );
    }

    #[test]
    fn repinned_plenum_pressure_preserves_finite_plenum_drift() {
        let definition = definition();
        let mut engine = SingleCylinderEngine::from_definition(definition);
        let target_pa = 60_000.0;
        let crank_speed_rad_per_s = rpm_to_rad_per_s(3000.0);

        let mut last_map_pa = 0.0;
        for _ in 0..4_000 {
            let output = engine.step(SingleCylinderStepInputs {
                fixed_crank_speed_rad_per_s: Some(crank_speed_rad_per_s),
                repinned_plenum_pressure_pa: Some(target_pa),
                throttle_position: 1.0,
                ..SingleCylinderStepInputs::default()
            });
            last_map_pa = output.intake_plenum_pressure_pa;
        }

        assert!(
            last_map_pa > target_pa + 3_000.0,
            "repinned finite plenum should be able to drift after pipe exchange; target {target_pa} Pa, got {last_map_pa} Pa"
        );
    }

    #[test]
    fn spark_time_combustion_decision_rejects_phi_outside_design_limits() {
        let mut engine = SingleCylinderEngine::from_definition(definition());
        engine.chamber_species = ChamberSpeciesMasses {
            oxygen_kg: 0.00232,
            inert_kg: 0.00768,
            fuel_kg: 0.010 / 7.0,
            products_kg: 0.0,
        };
        engine.sync_chamber_mass_from_species();

        let pending =
            engine.pending_combustion_at_spark(700.0_f64.to_radians(), rpm_to_rad_per_s(3000.0));

        assert!(pending.is_none());
    }

    #[test]
    fn spark_time_combustion_decision_uses_oxygen_not_residual_inert_for_afr() {
        let mut engine = SingleCylinderEngine::from_definition(definition());
        let limits = engine.definition.combustion.mixture_limits;
        let oxygen_kg = 0.001 * DRY_AIR_OXYGEN_MASS_FRACTION;
        engine.chamber_species = ChamberSpeciesMasses {
            oxygen_kg,
            inert_kg: 0.020,
            fuel_kg: 0.001 / limits.stoichiometric_air_fuel_ratio,
            products_kg: 0.0,
        };
        engine.sync_chamber_mass_from_species();

        let pending =
            engine.pending_combustion_at_spark(700.0_f64.to_radians(), rpm_to_rad_per_s(3000.0));

        assert!(pending.is_some());
    }

    #[test]
    fn spark_time_combustion_decision_slows_lean_burn_without_changing_wiebe_shape() {
        let mut stoich_engine = SingleCylinderEngine::from_definition(definition());
        stoich_engine.chamber_species = ChamberSpeciesMasses {
            oxygen_kg: 0.00232,
            inert_kg: 0.00768,
            fuel_kg: 0.010 / 14.7,
            products_kg: 0.0,
        };
        stoich_engine.sync_chamber_mass_from_species();

        let mut lean_engine = SingleCylinderEngine::from_definition(definition());
        lean_engine.chamber_species = ChamberSpeciesMasses {
            oxygen_kg: 0.00232,
            inert_kg: 0.00768,
            fuel_kg: 0.010 / 20.0,
            products_kg: 0.0,
        };
        lean_engine.sync_chamber_mass_from_species();

        let stoich = stoich_engine
            .pending_combustion_at_spark(700.0_f64.to_radians(), rpm_to_rad_per_s(3000.0))
            .expect("stoich mixture should combust");
        let lean = lean_engine
            .pending_combustion_at_spark(700.0_f64.to_radians(), rpm_to_rad_per_s(3000.0))
            .expect("lean combustible mixture should combust");

        assert!(lean.burn_duration_rad > stoich.burn_duration_rad);
        assert!(lean.combustion_efficiency < stoich.combustion_efficiency);
    }

    #[test]
    fn valve_overlap_allows_intake_flow_reversal() {
        let mut definition = definition();
        // The current GN250 fixture opens the intake at 405 deg, so sit a little
        // way into the intake-open window where the valve has lift.
        definition.crank.initial_crank_angle_deg = 447.0;
        let mut engine = SingleCylinderEngine::from_definition(definition.clone());
        let volume_m3 = cylinder_volume_m3(&definition, engine.crank_angle_rad());
        engine.chamber_state = chamber_state_at_pressure(
            definition.boundaries.intake_pressure_pa * 1.2,
            definition.boundaries.intake_temperature_k,
            volume_m3,
            chamber_properties(&definition),
        );
        // Keep the species inventory consistent with the pressurized charge:
        // step() re-derives the chamber mass from species, so without this the
        // 1.2x overpressure would be silently discarded before the valves see it.
        engine.chamber_species =
            ChamberSpeciesMasses::from_dry_air_mass(engine.chamber_state.mass_kg);

        let output = engine.step(SingleCylinderStepInputs {
            fixed_crank_speed_rad_per_s: Some(rpm_to_rad_per_s(3000.0)),
            ..SingleCylinderStepInputs::default()
        });

        assert!(output.intake_effective_area_m2 > 0.0);
        assert!(
            output.intake_mass_flow_kg_per_s < 0.0,
            "intake flow = {} (runner {} Pa, plenum {} Pa, cyl {} Pa, area {} m2)",
            output.intake_mass_flow_kg_per_s,
            output.intake_runner_pressure_pa,
            output.intake_plenum_pressure_pa,
            output.cylinder_pressure_pa,
            output.intake_effective_area_m2,
        );
    }

    #[test]
    fn step_uses_segmented_cubic_valve_profile_for_effective_area() {
        let mut definition = definition();
        definition.combustion.enabled = false;
        definition.crank.initial_crank_angle_deg = 420.0;
        definition.valves.intake.open_angle_deg = 360.0;
        definition.valves.intake.close_angle_deg = 600.0;
        definition.valves.intake.opening_ramp_fraction = 0.5;
        definition.valves.intake.plateau_fraction = 0.0;

        let mut segmented_definition = definition.clone();
        segmented_definition.valves.intake.lift_profile = ValveLiftProfileDefinition {
            model: ValveLiftProfileModel::SegmentedCubic,
            ramp_lift_fraction: 0.25,
            ramp_duration_fraction: 20.0,
            main_lift_duration_fraction: 80.0,
            dwell_duration_fraction: 40.0,
        };

        let mut legacy_engine = SingleCylinderEngine::from_definition(definition);
        let mut segmented_engine =
            SingleCylinderEngine::from_definition(segmented_definition.clone());
        let inputs = SingleCylinderStepInputs {
            fixed_crank_speed_rad_per_s: Some(0.0),
            ..SingleCylinderStepInputs::default()
        };

        let legacy_output = legacy_engine.step(inputs);
        let segmented_output = segmented_engine.step(inputs);
        let expected_segmented_area =
            ValveEvent::from_definition(segmented_definition.valves.intake)
                .effective_area_m2(420.0_f64.to_radians());

        assert_approx_eq(
            segmented_output.intake_effective_area_m2,
            expected_segmented_area,
            1.0e-15,
        );
        assert!(
            segmented_output.intake_effective_area_m2 > legacy_output.intake_effective_area_m2,
            "segmented area {} should exceed legacy area {} at 25% event progress",
            segmented_output.intake_effective_area_m2,
            legacy_output.intake_effective_area_m2,
        );
    }

    #[test]
    fn exhaust_collector_is_optional_and_affects_only_the_exit_path() {
        // Without a collector the exit pressure is reported and no collector
        // pressure exists; the single-pipe path is unchanged.
        let mut no_collector = definition();
        no_collector.intake_exhaust.exhaust_collector = None;
        let mut engine = SingleCylinderEngine::from_definition(no_collector);
        let output = engine.step(SingleCylinderStepInputs {
            fixed_crank_speed_rad_per_s: Some(rpm_to_rad_per_s(3000.0)),
            ..SingleCylinderStepInputs::default()
        });
        assert!(output.exhaust_collector_pressure_pa.is_none());
        assert!(output.exhaust_exit_pressure_pa > 0.0);

        // With a collector the engine stays stable and reports a finite,
        // positive collector pressure distinct from the open boundary.
        let mut with_collector = definition();
        with_collector.intake_exhaust.exhaust_collector =
            Some(crate::engine_config::PipeDefinition {
                number_of_cells: 6,
                total_length_m: 0.6,
                area_m2: 0.00096,
            });
        let mut engine = SingleCylinderEngine::from_definition(with_collector);
        let mut last = engine.step(SingleCylinderStepInputs {
            fixed_crank_speed_rad_per_s: Some(rpm_to_rad_per_s(3000.0)),
            ..SingleCylinderStepInputs::default()
        });
        for _ in 0..2000 {
            last = engine.step(SingleCylinderStepInputs {
                fixed_crank_speed_rad_per_s: Some(rpm_to_rad_per_s(3000.0)),
                ..SingleCylinderStepInputs::default()
            });
        }
        let collector_pressure = last
            .exhaust_collector_pressure_pa
            .expect("collector pressure should be reported");
        assert!(collector_pressure.is_finite() && collector_pressure > 0.0);
        assert!(last.exhaust_exit_pressure_pa.is_finite() && last.exhaust_exit_pressure_pa > 0.0);
    }

    #[test]
    fn exhaust_valve_discharge_coefficient_scales_exhaust_mass_flow() {
        let mut high_cd_definition = definition();
        high_cd_definition.combustion.enabled = false;
        high_cd_definition.crank.initial_crank_angle_deg = 180.0;
        high_cd_definition.valves.exhaust.discharge_coefficient = 1.0;
        let mut low_cd_definition = high_cd_definition.clone();
        low_cd_definition.valves.exhaust.discharge_coefficient = 0.25;

        let mut high_cd_engine = SingleCylinderEngine::from_definition(high_cd_definition.clone());
        let mut low_cd_engine = SingleCylinderEngine::from_definition(low_cd_definition.clone());
        for (engine, definition) in [
            (&mut high_cd_engine, &high_cd_definition),
            (&mut low_cd_engine, &low_cd_definition),
        ] {
            let volume_m3 = cylinder_volume_m3(definition, engine.crank_angle_rad());
            engine.chamber_state = chamber_state_at_pressure(
                definition.boundaries.exhaust_pressure_pa * 3.0,
                definition.boundaries.intake_temperature_k,
                volume_m3,
                chamber_properties(definition),
            );
            engine.chamber_species =
                ChamberSpeciesMasses::from_dry_air_mass(engine.chamber_state.mass_kg);
        }

        let inputs = SingleCylinderStepInputs {
            fixed_crank_speed_rad_per_s: Some(rpm_to_rad_per_s(3000.0)),
            throttle_position: 0.0,
            idle_throttle_fraction: 0.0,
            ..SingleCylinderStepInputs::default()
        };
        let high_cd_output = high_cd_engine.step(inputs);
        let low_cd_output = low_cd_engine.step(inputs);

        assert!(high_cd_output.exhaust_mass_flow_kg_per_s > 0.0);
        assert!(
            low_cd_output.exhaust_mass_flow_kg_per_s
                < high_cd_output.exhaust_mass_flow_kg_per_s * 0.5,
            "exhaust Cd should materially reduce valve mass flow: high Cd {}, low Cd {}",
            high_cd_output.exhaust_mass_flow_kg_per_s,
            low_cd_output.exhaust_mass_flow_kg_per_s
        );
    }

    #[test]
    fn reported_pressure_and_torque_use_same_step_instant() {
        let mut definition = definition();
        definition.combustion.enabled = false;
        definition.crank.initial_crank_angle_deg = 90.0;
        let mut engine = SingleCylinderEngine::from_definition(definition.clone());
        let volume_m3 = cylinder_volume_m3(&definition, engine.crank_angle_rad());
        engine.chamber_state = chamber_state_at_pressure(
            definition.boundaries.intake_pressure_pa * 2.0,
            definition.boundaries.intake_temperature_k,
            volume_m3,
            chamber_properties(&definition),
        );
        engine.chamber_species =
            ChamberSpeciesMasses::from_dry_air_mass(engine.chamber_state.mass_kg);

        let output = engine.step_with_timestep(
            0.001,
            SingleCylinderStepInputs {
                fixed_crank_speed_rad_per_s: Some(rpm_to_rad_per_s(3000.0)),
                throttle_position: 0.0,
                idle_throttle_fraction: 0.0,
                ..SingleCylinderStepInputs::default()
            },
        );
        let expected_snapshot_torque_nm = gas_force_n(&definition, output.cylinder_pressure_pa)
            * piston_dx_dtheta_m_per_rad(&definition, output.crank_angle_rad);

        assert_approx_eq(
            output.indicated_torque_nm,
            expected_snapshot_torque_nm,
            expected_snapshot_torque_nm.abs().max(1.0) * 1.0e-9,
        );
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
