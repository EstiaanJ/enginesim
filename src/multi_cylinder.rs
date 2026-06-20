use crate::engine_config::EngineDefinition;
use crate::profiles::SimulationProfile;
use crate::single_cylinder::{
    SingleCylinderEngine, SingleCylinderStepInputs, SingleCylinderStepOutput, rad_per_s_to_rpm,
    rpm_to_rad_per_s,
};
use crate::valve::{ENGINE_CYCLE_RADIANS, normalize_cycle_angle_rad};

#[derive(Debug, Clone)]
pub struct MultiCylinderEngine {
    definition: EngineDefinition,
    profile: SimulationProfile,
    firing_phase_offsets_deg: Vec<f64>,
    cylinders: Vec<SingleCylinderEngine>,
    crank_angle_rad: f64,
    crank_speed_rad_per_s: f64,
    accumulated_indicated_work_j: f64,
    accumulated_crank_angle_rad: f64,
    elapsed_time_seconds: f64,
    current_cycle_min_rpm: f64,
    current_cycle_max_rpm: f64,
    last_completed_cycle_rpm_delta: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CylinderStepOutput {
    pub cylinder_index: usize,
    pub firing_phase_offset_deg: f64,
    pub output: SingleCylinderStepOutput,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MultiCylinderStepOutput {
    pub elapsed_time_seconds: f64,
    pub crank_angle_rad: f64,
    pub crank_speed_rad_per_s: f64,
    pub rpm: f64,
    pub cylinder_outputs: Vec<CylinderStepOutput>,
    pub total_indicated_torque_nm: f64,
    pub indicated_work_j: f64,
    pub mean_indicated_torque_nm: f64,
    pub rpm_delta: f64,
}

impl MultiCylinderEngine {
    pub fn from_definition(definition: EngineDefinition) -> Self {
        Self::from_definition_with_profile(definition, SimulationProfile::real_time())
    }

    pub fn from_definition_with_profile(
        definition: EngineDefinition,
        profile: SimulationProfile,
    ) -> Self {
        profile.validate();
        let firing_phase_offsets_deg = definition.layout.firing_phase_offsets_deg();
        let global_initial_angle_deg = definition.crank.initial_crank_angle_deg;
        let initial_rpm = definition.crank.initial_speed_rpm;
        let cylinders = firing_phase_offsets_deg
            .iter()
            .map(|phase_deg| {
                let mut cylinder_definition = definition.clone();
                cylinder_definition.layout = Default::default();
                cylinder_definition.crank.initial_crank_angle_deg =
                    (global_initial_angle_deg - phase_deg).rem_euclid(720.0);
                SingleCylinderEngine::from_definition_with_profile(cylinder_definition, profile)
            })
            .collect();

        Self {
            crank_angle_rad: global_initial_angle_deg.to_radians(),
            crank_speed_rad_per_s: rpm_to_rad_per_s(definition.crank.initial_speed_rpm),
            definition,
            profile,
            firing_phase_offsets_deg,
            cylinders,
            accumulated_indicated_work_j: 0.0,
            accumulated_crank_angle_rad: 0.0,
            elapsed_time_seconds: 0.0,
            current_cycle_min_rpm: initial_rpm,
            current_cycle_max_rpm: initial_rpm,
            last_completed_cycle_rpm_delta: 0.0,
        }
    }

    pub fn firing_phase_offsets_deg(&self) -> &[f64] {
        &self.firing_phase_offsets_deg
    }

    pub fn crank_angle_rad(&self) -> f64 {
        self.crank_angle_rad
    }

    pub fn crank_speed_rad_per_s(&self) -> f64 {
        self.crank_speed_rad_per_s
    }

    pub fn mean_indicated_torque_nm(&self) -> f64 {
        if self.accumulated_crank_angle_rad <= 0.0 {
            return 0.0;
        }
        self.accumulated_indicated_work_j / self.accumulated_crank_angle_rad
    }

    pub fn rpm_delta(&self) -> f64 {
        self.last_completed_cycle_rpm_delta
            .max((self.current_cycle_max_rpm - self.current_cycle_min_rpm).max(0.0))
    }

    pub fn step(&mut self, inputs: SingleCylinderStepInputs) -> MultiCylinderStepOutput {
        let timestep_seconds = self.profile.timestep_seconds;
        let shared_crank_speed_rad_per_s = inputs
            .fixed_crank_speed_rad_per_s
            .unwrap_or(self.crank_speed_rad_per_s);
        assert!(
            shared_crank_speed_rad_per_s >= 0.0,
            "multi-cylinder runner expects non-negative crank speed"
        );

        let previous_crank_angle_rad = self.crank_angle_rad;
        let delta_angle_rad = shared_crank_speed_rad_per_s * timestep_seconds;
        let mut cylinder_outputs = Vec::with_capacity(self.cylinders.len());
        let mut total_indicated_torque_nm = 0.0;
        let mut indicated_work_j = 0.0;

        for (index, cylinder) in self.cylinders.iter_mut().enumerate() {
            let mut cylinder_inputs = inputs;
            cylinder_inputs.fixed_crank_speed_rad_per_s = Some(shared_crank_speed_rad_per_s);
            cylinder_inputs.external_load_torque_nm = 0.0;
            cylinder_inputs.added_inertia_kg_m2 = 0.0;
            cylinder_inputs.starter_enabled = false;
            let output = cylinder.step(cylinder_inputs);
            total_indicated_torque_nm += output.indicated_torque_nm;
            indicated_work_j += output.indicated_work_j;
            cylinder_outputs.push(CylinderStepOutput {
                cylinder_index: index,
                firing_phase_offset_deg: self.firing_phase_offsets_deg[index],
                output,
            });
        }

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
            let acceleration_rad_per_s2 = (total_indicated_torque_nm + starter_torque_nm
                - inputs.external_load_torque_nm)
                / effective_inertia_kg_m2;
            self.crank_speed_rad_per_s =
                (self.crank_speed_rad_per_s + acceleration_rad_per_s2 * timestep_seconds).max(0.0);
        } else {
            self.crank_speed_rad_per_s = shared_crank_speed_rad_per_s;
        }

        self.crank_angle_rad = normalize_cycle_angle_rad(self.crank_angle_rad + delta_angle_rad);
        self.elapsed_time_seconds += timestep_seconds;
        self.record_rpm_for_cycle(
            previous_crank_angle_rad,
            self.crank_angle_rad,
            delta_angle_rad,
            rad_per_s_to_rpm(self.crank_speed_rad_per_s),
        );

        MultiCylinderStepOutput {
            elapsed_time_seconds: self.elapsed_time_seconds,
            crank_angle_rad: self.crank_angle_rad,
            crank_speed_rad_per_s: self.crank_speed_rad_per_s,
            rpm: rad_per_s_to_rpm(self.crank_speed_rad_per_s),
            cylinder_outputs,
            total_indicated_torque_nm,
            indicated_work_j,
            mean_indicated_torque_nm: self.mean_indicated_torque_nm(),
            rpm_delta: self.rpm_delta(),
        }
    }

    fn record_rpm_for_cycle(
        &mut self,
        previous_crank_angle_rad: f64,
        crank_angle_rad: f64,
        delta_angle_rad: f64,
        rpm: f64,
    ) {
        if crank_angle_rad < previous_crank_angle_rad
            || previous_crank_angle_rad + delta_angle_rad >= ENGINE_CYCLE_RADIANS
        {
            self.last_completed_cycle_rpm_delta =
                (self.current_cycle_max_rpm - self.current_cycle_min_rpm).max(0.0);
            self.current_cycle_min_rpm = rpm;
            self.current_cycle_max_rpm = rpm;
            return;
        }

        self.current_cycle_min_rpm = self.current_cycle_min_rpm.min(rpm);
        self.current_cycle_max_rpm = self.current_cycle_max_rpm.max(rpm);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn definition() -> EngineDefinition {
        EngineDefinition::from_json_str(include_str!("../data/engines/gn250.json"))
            .expect("GN250 JSON should parse")
    }

    fn inline_four_definition() -> EngineDefinition {
        let mut definition = definition();
        definition.layout.cylinder_count = 4;
        definition.layout.firing_order = vec![1, 3, 4, 2];
        definition.combustion.enabled = false;
        definition
    }

    #[test]
    fn firing_order_maps_to_even_fire_phase_offsets() {
        let engine = MultiCylinderEngine::from_definition(inline_four_definition());

        assert_eq!(
            engine.firing_phase_offsets_deg(),
            &[0.0, 540.0, 180.0, 360.0]
        );
    }

    #[test]
    fn cylinders_are_initialized_with_phase_shifted_local_angles() {
        let mut engine = MultiCylinderEngine::from_definition(inline_four_definition());
        let output = engine.step(SingleCylinderStepInputs {
            fixed_crank_speed_rad_per_s: Some(0.0),
            ..SingleCylinderStepInputs::default()
        });
        let angles: Vec<f64> = output
            .cylinder_outputs
            .iter()
            .map(|cylinder| cylinder.output.crank_angle_rad.to_degrees().round())
            .collect();

        assert_eq!(angles, vec![0.0, 180.0, 540.0, 360.0]);
    }

    #[test]
    fn total_torque_is_sum_of_cylinder_torques() {
        let mut engine = MultiCylinderEngine::from_definition(inline_four_definition());
        let output = engine.step(SingleCylinderStepInputs {
            fixed_crank_speed_rad_per_s: Some(rpm_to_rad_per_s(3000.0)),
            ..SingleCylinderStepInputs::default()
        });
        let sum = output
            .cylinder_outputs
            .iter()
            .map(|cylinder| cylinder.output.indicated_torque_nm)
            .sum::<f64>();

        assert!((output.total_indicated_torque_nm - sum).abs() <= 1.0e-12);
    }

    #[test]
    fn repeated_multi_cylinder_runs_are_deterministic() {
        let mut first = MultiCylinderEngine::from_definition(inline_four_definition());
        let mut second = MultiCylinderEngine::from_definition(inline_four_definition());
        let inputs = SingleCylinderStepInputs {
            fixed_crank_speed_rad_per_s: Some(rpm_to_rad_per_s(3000.0)),
            ..SingleCylinderStepInputs::default()
        };

        for _ in 0..200 {
            let first_output = first.step(inputs);
            let second_output = second.step(inputs);
            assert_eq!(first_output, second_output);
        }
    }

    #[test]
    fn reports_rpm_delta_for_shared_crank_cycle() {
        let mut engine = MultiCylinderEngine::from_definition(inline_four_definition());
        let output = engine.step(SingleCylinderStepInputs {
            external_load_torque_nm: 5.0,
            ..SingleCylinderStepInputs::default()
        });

        assert!(output.rpm_delta.is_finite());
        assert!(output.rpm_delta >= 0.0);
    }

    #[test]
    fn changing_firing_order_changes_cylinder_phase_not_mean_torque() {
        let first_definition = inline_four_definition();
        let mut second_definition = first_definition.clone();
        second_definition.layout.firing_order = vec![1, 2, 4, 3];
        let mut first = MultiCylinderEngine::from_definition(first_definition.clone());
        let mut second = MultiCylinderEngine::from_definition(second_definition.clone());
        let inputs = SingleCylinderStepInputs {
            fixed_crank_speed_rad_per_s: Some(rpm_to_rad_per_s(2500.0)),
            ..SingleCylinderStepInputs::default()
        };

        for _ in 0..400 {
            first.step(inputs);
            second.step(inputs);
        }

        assert_ne!(
            first_definition.layout.firing_phase_offsets_deg(),
            second_definition.layout.firing_phase_offsets_deg()
        );
        assert!(
            (first.mean_indicated_torque_nm() - second.mean_indicated_torque_nm()).abs() < 1.0e-9
        );
    }
}
