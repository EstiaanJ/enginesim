use crate::linear_mass_integrator::{LinearMassState, update_linear_mass};
use crate::rotational_mass_integrator::{
    RotationalMassConfig, RotationalMassState, update_rotational_mass,
};

#[derive(Debug, Clone)]
pub struct MechanicalLoads {
    /// Net force to apply to each linear body [N]
    pub linear_forces_n: Vec<f64>,

    /// Net torque to apply to each rotational body [N*m]
    pub rotational_torques_nm: Vec<f64>,
}

impl MechanicalLoads {
    pub fn new(linear_count: usize, rotational_count: usize) -> Self {
        Self {
            linear_forces_n: vec![0.0; linear_count],
            rotational_torques_nm: vec![0.0; rotational_count],
        }
    }

    pub fn add_linear_force(&mut self, linear_index: usize, force_n: f64) {
        self.linear_forces_n[linear_index] += force_n;
    }

    pub fn add_rotational_torque(&mut self, rotational_index: usize, torque_nm: f64) {
        self.rotational_torques_nm[rotational_index] += torque_nm;
    }
}

#[derive(Debug, Clone)]
pub struct MechanicalStepUpdate {
    pub linear_states: Vec<LinearMassState>,
    pub rotational_states: Vec<RotationalMassState>,
    pub loads: MechanicalLoads,
}

pub fn integrate_linked_masses(
    linear_states: &mut [LinearMassState],
    rotational_states: &mut [RotationalMassState],
    rotational_configs: &[RotationalMassConfig],
    timestep_seconds: f64,
    loads: &MechanicalLoads,
) -> MechanicalStepUpdate {
    assert_eq!(
        linear_states.len(),
        loads.linear_forces_n.len(),
        "linear state and force counts must match",
    );
    assert_eq!(
        rotational_states.len(),
        loads.rotational_torques_nm.len(),
        "rotational state and torque counts must match",
    );
    assert_eq!(
        rotational_states.len(),
        rotational_configs.len(),
        "rotational state and config counts must match",
    );

    for (state, force_n) in linear_states.iter_mut().zip(loads.linear_forces_n.iter()) {
        update_linear_mass(state, timestep_seconds, *force_n);
    }

    for ((state, config), torque_nm) in rotational_states
        .iter_mut()
        .zip(rotational_configs.iter())
        .zip(loads.rotational_torques_nm.iter())
    {
        update_rotational_mass(state, *config, timestep_seconds, *torque_nm);
    }

    MechanicalStepUpdate {
        linear_states: linear_states.to_vec(),
        rotational_states: rotational_states.to_vec(),
        loads: loads.clone(),
    }
}

#[derive(Debug, Clone, Copy)]
pub struct LinearSpringDamperLink {
    pub first_linear_index: usize,
    pub second_linear_index: usize,
    pub rest_displacement_m: f64,
    pub stiffness_n_per_m: f64,
    pub damping_n_per_m_per_s: f64,
}

pub fn apply_linear_spring_damper_link(
    loads: &mut MechanicalLoads,
    linear_states: &[LinearMassState],
    link: LinearSpringDamperLink,
) {
    let first = linear_states[link.first_linear_index];
    let second = linear_states[link.second_linear_index];
    let displacement_error_m = (second.position_m - first.position_m) - link.rest_displacement_m;
    let velocity_error_m_per_s = second.velocity_m_per_s - first.velocity_m_per_s;
    let force_on_first_n = link.stiffness_n_per_m * displacement_error_m
        + link.damping_n_per_m_per_s * velocity_error_m_per_s;

    loads.add_linear_force(link.first_linear_index, force_on_first_n);
    loads.add_linear_force(link.second_linear_index, -force_on_first_n);
}

#[derive(Debug, Clone, Copy)]
pub struct RotationalRatioSpringDamperLink {
    pub driver_rotational_index: usize,
    pub driven_rotational_index: usize,

    /// Driven angle per driver angle, e.g. 0.5 for cam angle from crank angle.
    pub driven_per_driver_ratio: f64,

    /// Driven angle when driver angle * ratio is zero [rad]
    pub phase_offset_rad: f64,

    /// Corrective torque stiffness applied to the driven side [N*m/rad]
    pub stiffness_nm_per_rad: f64,

    /// Corrective damping applied to the driven side [N*m/(rad/s)]
    pub damping_nm_per_rad_per_s: f64,
}

pub fn apply_rotational_ratio_spring_damper_link(
    loads: &mut MechanicalLoads,
    rotational_states: &[RotationalMassState],
    link: RotationalRatioSpringDamperLink,
) {
    let driver = rotational_states[link.driver_rotational_index];
    let driven = rotational_states[link.driven_rotational_index];

    let expected_driven_angle_rad =
        driver.angle_rad * link.driven_per_driver_ratio + link.phase_offset_rad;
    let angle_error_rad = driven.angle_rad - expected_driven_angle_rad;
    let angular_velocity_error_rad_per_s = driven.angular_velocity_rad_per_s
        - driver.angular_velocity_rad_per_s * link.driven_per_driver_ratio;
    let torque_on_driven_nm = -link.stiffness_nm_per_rad * angle_error_rad
        - link.damping_nm_per_rad_per_s * angular_velocity_error_rad_per_s;
    let torque_on_driver_nm = -torque_on_driven_nm * link.driven_per_driver_ratio;

    loads.add_rotational_torque(link.driven_rotational_index, torque_on_driven_nm);
    loads.add_rotational_torque(link.driver_rotational_index, torque_on_driver_nm);
}

#[derive(Debug, Clone, Copy)]
pub struct LinearRotationalSpringDamperLink {
    pub linear_index: usize,
    pub rotational_index: usize,

    /// Linear position per rotational angle [m/rad].
    /// For piston/crank use an externally calculated local dx/dtheta.
    pub meters_per_radian: f64,

    /// Linear position when angle * meters_per_radian is zero [m]
    pub position_offset_m: f64,

    /// Corrective force stiffness applied to the linear side [N/m]
    pub stiffness_n_per_m: f64,

    /// Corrective damping applied to the linear side [N/(m/s)]
    pub damping_n_per_m_per_s: f64,
}

pub fn apply_linear_rotational_spring_damper_link(
    loads: &mut MechanicalLoads,
    linear_states: &[LinearMassState],
    rotational_states: &[RotationalMassState],
    link: LinearRotationalSpringDamperLink,
) {
    let linear = linear_states[link.linear_index];
    let rotational = rotational_states[link.rotational_index];

    let expected_position_m =
        rotational.angle_rad * link.meters_per_radian + link.position_offset_m;
    let position_error_m = linear.position_m - expected_position_m;
    let velocity_error_m_per_s =
        linear.velocity_m_per_s - rotational.angular_velocity_rad_per_s * link.meters_per_radian;
    let force_on_linear_n = -link.stiffness_n_per_m * position_error_m
        - link.damping_n_per_m_per_s * velocity_error_m_per_s;
    let torque_on_rotational_nm = -force_on_linear_n * link.meters_per_radian;

    loads.add_linear_force(link.linear_index, force_on_linear_n);
    loads.add_rotational_torque(link.rotational_index, torque_on_rotational_nm);
}

pub fn torque_from_linear_force(force_n: f64, meters_per_radian: f64) -> f64 {
    force_n * meters_per_radian
}

pub fn force_from_rotational_torque(torque_nm: f64, meters_per_radian: f64) -> f64 {
    assert!(
        meters_per_radian.abs() > 0.0,
        "meters per radian must be non-zero",
    );

    torque_nm / meters_per_radian
}

pub fn explicit_spring_stability_timestep_seconds(inertia: f64, stiffness: f64) -> f64 {
    assert!(inertia > 0.0, "inertia must be positive");
    assert!(stiffness >= 0.0, "stiffness must be non-negative");
    if stiffness == 0.0 {
        return f64::INFINITY;
    }

    0.25 * 2.0 / (stiffness / inertia).sqrt()
}

pub fn explicit_spring_timestep_is_stable(
    timestep_seconds: f64,
    inertia: f64,
    stiffness: f64,
) -> bool {
    assert!(timestep_seconds >= 0.0, "timestep must be non-negative");
    timestep_seconds <= explicit_spring_stability_timestep_seconds(inertia, stiffness)
}

pub fn enforce_linear_kinematic_state(
    state: &mut LinearMassState,
    position_m: f64,
    velocity_m_per_s: f64,
    acceleration_m_per_s2: f64,
) -> LinearMassState {
    let previous_position_m = state.position_m;

    state.previous_position_m = previous_position_m;
    state.position_m = position_m;
    state.velocity_m_per_s = velocity_m_per_s;
    state.acceleration_m_per_s2 = acceleration_m_per_s2;
    state.displacement_m = position_m - previous_position_m;

    *state
}

pub fn apply_constrained_linear_force_to_rotational_load(
    loads: &mut MechanicalLoads,
    rotational_index: usize,
    linear_force_n: f64,
    dx_dtheta_m_per_rad: f64,
) {
    loads.add_rotational_torque(
        rotational_index,
        torque_from_linear_force(linear_force_n, dx_dtheta_m_per_rad),
    );
}

#[derive(Debug, Clone, Copy)]
pub struct SolidLinearRotationalConstraint {
    pub linear_index: usize,
    pub rotational_index: usize,
}

#[derive(Debug, Clone, Copy)]
pub struct SolidLinearKinematics {
    /// Linear position imposed by the rotational body [m]
    pub position_m: f64,

    /// Linear velocity imposed by the rotational body [m/s]
    pub velocity_m_per_s: f64,

    /// Linear acceleration imposed by the rotational body [m/s^2]
    pub acceleration_m_per_s2: f64,

    /// Current derivative of imposed linear position with respect to rotational angle [m/rad]
    pub dx_dtheta_m_per_rad: f64,
}

pub fn enforce_solid_linear_rotational_constraint(
    linear_states: &mut [LinearMassState],
    constraint: SolidLinearRotationalConstraint,
    kinematics: SolidLinearKinematics,
) -> LinearMassState {
    enforce_linear_kinematic_state(
        &mut linear_states[constraint.linear_index],
        kinematics.position_m,
        kinematics.velocity_m_per_s,
        kinematics.acceleration_m_per_s2,
    )
}

pub fn apply_solid_linear_rotational_force(
    loads: &mut MechanicalLoads,
    constraint: SolidLinearRotationalConstraint,
    linear_force_n: f64,
    kinematics: SolidLinearKinematics,
) {
    apply_constrained_linear_force_to_rotational_load(
        loads,
        constraint.rotational_index,
        linear_force_n,
        kinematics.dx_dtheta_m_per_rad,
    );
}

pub fn apply_solid_linear_rotational_external_and_inertial_forces(
    loads: &mut MechanicalLoads,
    linear_states: &[LinearMassState],
    constraint: SolidLinearRotationalConstraint,
    external_linear_force_n: f64,
    kinematics: SolidLinearKinematics,
) {
    let constrained_linear = linear_states[constraint.linear_index];
    let inertial_reaction_force_n = -constrained_linear.mass_kg * kinematics.acceleration_m_per_s2;
    let force_transmitted_to_rotational_n = external_linear_force_n + inertial_reaction_force_n;

    apply_solid_linear_rotational_force(
        loads,
        constraint,
        force_transmitted_to_rotational_n,
        kinematics,
    );
}

#[derive(Debug, Clone, Copy)]
pub struct SolidRotationalRatioConstraint {
    pub driver_rotational_index: usize,
    pub driven_rotational_index: usize,

    /// Driven angle per driver angle. Use 0.5 for cam angle from crank angle.
    pub driven_per_driver_ratio: f64,

    /// Driven angle when driver angle * ratio is zero [rad]
    pub phase_offset_rad: f64,
}

pub fn enforce_solid_rotational_ratio_constraint(
    rotational_states: &mut [RotationalMassState],
    driven_config: RotationalMassConfig,
    constraint: SolidRotationalRatioConstraint,
) -> RotationalMassState {
    let driver = rotational_states[constraint.driver_rotational_index];
    let driven = &mut rotational_states[constraint.driven_rotational_index];
    let previous_angle_rad = driven.angle_rad;
    let driven_angle_rad =
        driver.angle_rad * constraint.driven_per_driver_ratio + constraint.phase_offset_rad;
    let driven_velocity_rad_per_s =
        driver.angular_velocity_rad_per_s * constraint.driven_per_driver_ratio;
    let driven_acceleration_rad_per_s2 =
        driver.angular_acceleration_rad_per_s2 * constraint.driven_per_driver_ratio;
    let constrained_angle_rad = bounded_rotational_angle_rad(driven_angle_rad, driven_config);

    driven.previous_angle_rad = previous_angle_rad;
    driven.angle_rad = constrained_angle_rad;
    driven.angular_velocity_rad_per_s = driven_velocity_rad_per_s;
    driven.angular_acceleration_rad_per_s2 = driven_acceleration_rad_per_s2;
    driven.angular_displacement_rad =
        angular_displacement_rad(previous_angle_rad, driven.angle_rad, driven_config);

    *driven
}

pub fn apply_solid_rotational_ratio_driven_torque_to_driver(
    loads: &mut MechanicalLoads,
    constraint: SolidRotationalRatioConstraint,
    driven_torque_nm: f64,
) {
    loads.add_rotational_torque(
        constraint.driver_rotational_index,
        driven_torque_nm * constraint.driven_per_driver_ratio,
    );
}

fn bounded_rotational_angle_rad(angle_rad: f64, config: RotationalMassConfig) -> f64 {
    if config.wrap_angle {
        angle_rad.rem_euclid(config.max_angle_rad)
    } else {
        angle_rad.clamp(0.0, config.max_angle_rad)
    }
}

fn angular_displacement_rad(
    previous_angle_rad: f64,
    current_angle_rad: f64,
    config: RotationalMassConfig,
) -> f64 {
    let raw_displacement_rad = current_angle_rad - previous_angle_rad;
    if !config.wrap_angle {
        return raw_displacement_rad;
    }

    let half_range_rad = config.max_angle_rad * 0.5;
    (raw_displacement_rad + half_range_rad).rem_euclid(config.max_angle_rad) - half_range_rad
}

#[derive(Debug, Clone, Copy)]
pub struct IndependentRotationalBody {
    pub state: RotationalMassState,
    pub config: RotationalMassConfig,
    pub external_torque_nm: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct DependentLinearBody {
    pub state: LinearMassState,
    pub parent_rotational_index: usize,
    pub external_force_n: f64,
    pub kinematics: SolidLinearKinematics,
}

#[derive(Debug, Clone, Copy)]
pub struct DependentRotationalBody {
    pub state: RotationalMassState,
    pub config: RotationalMassConfig,
    pub parent_rotational_index: usize,
    pub external_torque_nm: f64,
    pub driven_per_parent_ratio: f64,
    pub phase_offset_rad: f64,
}

#[derive(Debug, Clone)]
pub struct MechanicalSystemStep {
    pub independent_rotational_bodies: Vec<IndependentRotationalBody>,
    pub dependent_linear_bodies: Vec<DependentLinearBody>,
    pub dependent_rotational_bodies: Vec<DependentRotationalBody>,
    pub independent_rotational_torques_nm: Vec<f64>,
}

pub fn step_mechanical_system(
    independent_rotational_bodies: &mut [IndependentRotationalBody],
    dependent_linear_bodies: &mut [DependentLinearBody],
    dependent_rotational_bodies: &mut [DependentRotationalBody],
    timestep_seconds: f64,
) -> MechanicalSystemStep {
    let mut independent_rotational_torques_nm = independent_rotational_bodies
        .iter()
        .map(|body| body.external_torque_nm)
        .collect::<Vec<_>>();
    let mut effective_parent_inertia_kg_m2 = independent_rotational_bodies
        .iter()
        .map(|body| body.state.moment_of_inertia_kg_m2)
        .collect::<Vec<_>>();

    for body in dependent_linear_bodies.iter_mut() {
        enforce_linear_kinematic_state(
            &mut body.state,
            body.kinematics.position_m,
            body.kinematics.velocity_m_per_s,
            body.kinematics.acceleration_m_per_s2,
        );

        independent_rotational_torques_nm[body.parent_rotational_index] +=
            torque_from_linear_force(body.external_force_n, body.kinematics.dx_dtheta_m_per_rad);
        effective_parent_inertia_kg_m2[body.parent_rotational_index] +=
            body.state.mass_kg * body.kinematics.dx_dtheta_m_per_rad.powi(2);
    }

    for body in dependent_rotational_bodies.iter() {
        independent_rotational_torques_nm[body.parent_rotational_index] +=
            body.external_torque_nm * body.driven_per_parent_ratio;
        effective_parent_inertia_kg_m2[body.parent_rotational_index] +=
            body.state.moment_of_inertia_kg_m2 * body.driven_per_parent_ratio.powi(2);
    }

    for ((body, torque_nm), effective_inertia_kg_m2) in independent_rotational_bodies
        .iter_mut()
        .zip(independent_rotational_torques_nm.iter())
        .zip(effective_parent_inertia_kg_m2.iter())
    {
        let body_inertia_kg_m2 = body.state.moment_of_inertia_kg_m2;
        body.state.moment_of_inertia_kg_m2 = *effective_inertia_kg_m2;
        update_rotational_mass(&mut body.state, body.config, timestep_seconds, *torque_nm);
        body.state.moment_of_inertia_kg_m2 = body_inertia_kg_m2;
    }

    for body in dependent_rotational_bodies.iter_mut() {
        let parent = independent_rotational_bodies[body.parent_rotational_index].state;
        let previous_angle_rad = body.state.angle_rad;
        let driven_angle_rad =
            parent.angle_rad * body.driven_per_parent_ratio + body.phase_offset_rad;

        body.state.previous_angle_rad = previous_angle_rad;
        body.state.angle_rad = bounded_rotational_angle_rad(driven_angle_rad, body.config);
        body.state.angular_velocity_rad_per_s =
            parent.angular_velocity_rad_per_s * body.driven_per_parent_ratio;
        body.state.angular_acceleration_rad_per_s2 =
            parent.angular_acceleration_rad_per_s2 * body.driven_per_parent_ratio;
        body.state.angular_displacement_rad =
            angular_displacement_rad(previous_angle_rad, body.state.angle_rad, body.config);
    }

    MechanicalSystemStep {
        independent_rotational_bodies: independent_rotational_bodies.to_vec(),
        dependent_linear_bodies: dependent_linear_bodies.to_vec(),
        dependent_rotational_bodies: dependent_rotational_bodies.to_vec(),
        independent_rotational_torques_nm,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1.0e-12;

    fn assert_approx_eq(actual: f64, expected: f64) {
        let difference = (actual - expected).abs();
        assert!(
            difference <= EPSILON,
            "expected {expected}, got {actual}; difference {difference} exceeded {EPSILON}",
        );
    }

    #[test]
    fn aggregates_external_loads() {
        let mut loads = MechanicalLoads::new(2, 1);

        loads.add_linear_force(0, 10.0);
        loads.add_linear_force(0, -3.0);
        loads.add_rotational_torque(0, 4.0);

        assert_approx_eq(loads.linear_forces_n[0], 7.0);
        assert_approx_eq(loads.linear_forces_n[1], 0.0);
        assert_approx_eq(loads.rotational_torques_nm[0], 4.0);
    }

    #[test]
    fn linear_spring_damper_applies_equal_and_opposite_forces() {
        let states = [
            LinearMassState::new(1.0, 0.0, 0.0),
            LinearMassState::new(1.0, 2.0, 0.0),
        ];
        let mut loads = MechanicalLoads::new(2, 0);

        apply_linear_spring_damper_link(
            &mut loads,
            &states,
            LinearSpringDamperLink {
                first_linear_index: 0,
                second_linear_index: 1,
                rest_displacement_m: 1.0,
                stiffness_n_per_m: 10.0,
                damping_n_per_m_per_s: 0.0,
            },
        );

        assert_approx_eq(loads.linear_forces_n[0], 10.0);
        assert_approx_eq(loads.linear_forces_n[1], -10.0);
    }

    #[test]
    fn rotational_ratio_link_transfers_torque_to_driver_and_driven() {
        let config = RotationalMassConfig {
            max_angle_rad: std::f64::consts::TAU,
            wrap_angle: true,
        };
        let states = [
            RotationalMassState::new(1.0, 2.0, 0.0, config),
            RotationalMassState::new(1.0, 0.5, 0.0, config),
        ];
        let mut loads = MechanicalLoads::new(0, 2);

        apply_rotational_ratio_spring_damper_link(
            &mut loads,
            &states,
            RotationalRatioSpringDamperLink {
                driver_rotational_index: 0,
                driven_rotational_index: 1,
                driven_per_driver_ratio: 0.5,
                phase_offset_rad: 0.0,
                stiffness_nm_per_rad: 20.0,
                damping_nm_per_rad_per_s: 0.0,
            },
        );

        assert_approx_eq(loads.rotational_torques_nm[1], 10.0);
        assert_approx_eq(loads.rotational_torques_nm[0], -5.0);
    }

    #[test]
    fn linear_rotational_link_transfers_force_and_reaction_torque() {
        let config = RotationalMassConfig {
            max_angle_rad: std::f64::consts::TAU,
            wrap_angle: true,
        };
        let linear_states = [LinearMassState::new(1.0, 3.0, 0.0)];
        let rotational_states = [RotationalMassState::new(1.0, 1.0, 0.0, config)];
        let mut loads = MechanicalLoads::new(1, 1);

        apply_linear_rotational_spring_damper_link(
            &mut loads,
            &linear_states,
            &rotational_states,
            LinearRotationalSpringDamperLink {
                linear_index: 0,
                rotational_index: 0,
                meters_per_radian: 2.0,
                position_offset_m: 0.0,
                stiffness_n_per_m: 10.0,
                damping_n_per_m_per_s: 0.0,
            },
        );

        assert_approx_eq(loads.linear_forces_n[0], -10.0);
        assert_approx_eq(loads.rotational_torques_nm[0], 20.0);
    }

    #[test]
    fn driver_applies_aggregated_loads_to_integrators() {
        let mut linear_states = [LinearMassState::new(2.0, 0.0, 0.0)];
        let config = RotationalMassConfig {
            max_angle_rad: std::f64::consts::TAU,
            wrap_angle: true,
        };
        let mut rotational_states = [RotationalMassState::new(2.0, 0.0, 0.0, config)];
        let mut loads = MechanicalLoads::new(1, 1);
        loads.add_linear_force(0, 8.0);
        loads.add_rotational_torque(0, 4.0);

        let update = integrate_linked_masses(
            &mut linear_states,
            &mut rotational_states,
            &[config],
            0.5,
            &loads,
        );

        assert_approx_eq(update.linear_states[0].position_m, 0.5);
        assert_approx_eq(update.linear_states[0].velocity_m_per_s, 2.0);
        assert_approx_eq(update.rotational_states[0].angle_rad, 0.25);
        assert_approx_eq(update.rotational_states[0].angular_velocity_rad_per_s, 1.0);
        assert_approx_eq(update.loads.linear_forces_n[0], 8.0);
    }

    #[test]
    fn converts_between_linear_force_and_rotational_torque() {
        assert_approx_eq(torque_from_linear_force(100.0, 0.25), 25.0);
        assert_approx_eq(force_from_rotational_torque(25.0, 0.25), 100.0);
    }

    #[test]
    fn explicit_spring_timestep_helper_flags_stiff_links() {
        let soft_limit = explicit_spring_stability_timestep_seconds(1.0, 100.0);
        let stiff_limit = explicit_spring_stability_timestep_seconds(1.0, 10_000.0);

        assert!(stiff_limit < soft_limit);
        assert!(explicit_spring_timestep_is_stable(
            stiff_limit,
            1.0,
            10_000.0
        ));
        assert!(!explicit_spring_timestep_is_stable(
            stiff_limit * 1.1,
            1.0,
            10_000.0
        ));
        assert!(explicit_spring_stability_timestep_seconds(1.0, 0.0).is_infinite());
    }

    #[test]
    fn hard_kinematic_linear_state_overrides_free_motion_state() {
        let mut state = LinearMassState::new(1.0, 1.0, 2.0);

        let update = enforce_linear_kinematic_state(&mut state, 3.0, 4.0, 5.0);

        assert_approx_eq(update.previous_position_m, 1.0);
        assert_approx_eq(update.position_m, 3.0);
        assert_approx_eq(update.displacement_m, 2.0);
        assert_approx_eq(update.velocity_m_per_s, 4.0);
        assert_approx_eq(update.acceleration_m_per_s2, 5.0);
    }

    #[test]
    fn constrained_linear_force_is_transferred_to_rotational_load() {
        let mut loads = MechanicalLoads::new(1, 1);

        apply_constrained_linear_force_to_rotational_load(&mut loads, 0, 200.0, 0.05);

        assert_approx_eq(loads.linear_forces_n[0], 0.0);
        assert_approx_eq(loads.rotational_torques_nm[0], 10.0);
    }

    #[test]
    fn solid_linear_rotational_constraint_imposes_linear_kinematics() {
        let mut linear_states = [LinearMassState::new(1.0, 0.0, 0.0)];
        let constraint = SolidLinearRotationalConstraint {
            linear_index: 0,
            rotational_index: 0,
        };

        let update = enforce_solid_linear_rotational_constraint(
            &mut linear_states,
            constraint,
            SolidLinearKinematics {
                position_m: 0.02,
                velocity_m_per_s: 1.5,
                acceleration_m_per_s2: -3.0,
                dx_dtheta_m_per_rad: 0.01,
            },
        );

        assert_approx_eq(update.position_m, 0.02);
        assert_approx_eq(update.velocity_m_per_s, 1.5);
        assert_approx_eq(update.acceleration_m_per_s2, -3.0);
    }

    #[test]
    fn solid_linear_rotational_constraint_transfers_force_to_crank_torque() {
        let mut loads = MechanicalLoads::new(1, 1);
        let constraint = SolidLinearRotationalConstraint {
            linear_index: 0,
            rotational_index: 0,
        };

        apply_solid_linear_rotational_force(
            &mut loads,
            constraint,
            1_000.0,
            SolidLinearKinematics {
                position_m: 0.02,
                velocity_m_per_s: 1.5,
                acceleration_m_per_s2: -3.0,
                dx_dtheta_m_per_rad: 0.01,
            },
        );

        assert_approx_eq(loads.linear_forces_n[0], 0.0);
        assert_approx_eq(loads.rotational_torques_nm[0], 10.0);
    }

    #[test]
    fn solid_linear_rotational_constraint_adds_piston_inertial_reaction_torque() {
        let mut loads = MechanicalLoads::new(1, 1);
        let linear_states = [LinearMassState::new(2.0, 0.0, 0.0)];
        let constraint = SolidLinearRotationalConstraint {
            linear_index: 0,
            rotational_index: 0,
        };

        apply_solid_linear_rotational_external_and_inertial_forces(
            &mut loads,
            &linear_states,
            constraint,
            1_000.0,
            SolidLinearKinematics {
                position_m: 0.02,
                velocity_m_per_s: 1.5,
                acceleration_m_per_s2: 100.0,
                dx_dtheta_m_per_rad: 0.01,
            },
        );

        assert_approx_eq(loads.rotational_torques_nm[0], 8.0);
    }

    #[test]
    fn solid_rotational_ratio_constraint_imposes_no_slip_driven_state() {
        let config = RotationalMassConfig {
            max_angle_rad: std::f64::consts::TAU,
            wrap_angle: true,
        };
        let mut rotational_states = [
            RotationalMassState::new(1.0, 2.0, 4.0, config),
            RotationalMassState::new(1.0, 0.0, 0.0, config),
        ];
        rotational_states[0].angular_acceleration_rad_per_s2 = 6.0;

        let update = enforce_solid_rotational_ratio_constraint(
            &mut rotational_states,
            config,
            SolidRotationalRatioConstraint {
                driver_rotational_index: 0,
                driven_rotational_index: 1,
                driven_per_driver_ratio: 0.5,
                phase_offset_rad: 0.25,
            },
        );

        assert_approx_eq(update.angle_rad, 1.25);
        assert_approx_eq(update.angular_velocity_rad_per_s, 2.0);
        assert_approx_eq(update.angular_acceleration_rad_per_s2, 3.0);
    }

    #[test]
    fn solid_rotational_ratio_constraint_reflects_driven_torque_to_driver() {
        let mut loads = MechanicalLoads::new(0, 2);

        apply_solid_rotational_ratio_driven_torque_to_driver(
            &mut loads,
            SolidRotationalRatioConstraint {
                driver_rotational_index: 0,
                driven_rotational_index: 1,
                driven_per_driver_ratio: 0.5,
                phase_offset_rad: 0.0,
            },
            8.0,
        );

        assert_approx_eq(loads.rotational_torques_nm[0], 4.0);
        assert_approx_eq(loads.rotational_torques_nm[1], 0.0);
    }

    #[test]
    fn mechanical_system_integrates_only_independent_rotational_bodies() {
        let config = RotationalMassConfig {
            max_angle_rad: std::f64::consts::TAU,
            wrap_angle: true,
        };
        let mut independent_bodies = [IndependentRotationalBody {
            state: RotationalMassState::new(2.0, 0.0, 0.0, config),
            config,
            external_torque_nm: 8.0,
        }];
        let mut dependent_linear_bodies = [DependentLinearBody {
            state: LinearMassState::new(1.0, 0.0, 0.0),
            parent_rotational_index: 0,
            external_force_n: 0.0,
            kinematics: SolidLinearKinematics {
                position_m: 0.02,
                velocity_m_per_s: 3.0,
                acceleration_m_per_s2: 4.0,
                dx_dtheta_m_per_rad: 0.01,
            },
        }];
        let mut dependent_rotational_bodies = [DependentRotationalBody {
            state: RotationalMassState::new(1.0, 0.0, 0.0, config),
            config,
            parent_rotational_index: 0,
            external_torque_nm: 0.0,
            driven_per_parent_ratio: 0.5,
            phase_offset_rad: 0.0,
        }];

        let update = step_mechanical_system(
            &mut independent_bodies,
            &mut dependent_linear_bodies,
            &mut dependent_rotational_bodies,
            0.5,
        );

        let expected_acceleration = 8.0 / (2.0 + 1.0 * 0.01_f64.powi(2) + 1.0 * 0.5_f64.powi(2));

        assert_approx_eq(update.independent_rotational_torques_nm[0], 8.0);
        assert_approx_eq(
            update.independent_rotational_bodies[0].state.angle_rad,
            0.5 * expected_acceleration * 0.5_f64.powi(2),
        );
        assert_approx_eq(
            update.independent_rotational_bodies[0]
                .state
                .angular_velocity_rad_per_s,
            expected_acceleration * 0.5,
        );
        assert_approx_eq(update.dependent_linear_bodies[0].state.position_m, 0.02);
        assert_approx_eq(
            update.dependent_linear_bodies[0].state.velocity_m_per_s,
            3.0,
        );
        assert_approx_eq(
            update.dependent_rotational_bodies[0].state.angle_rad,
            0.25 * expected_acceleration * 0.5_f64.powi(2),
        );
        assert_approx_eq(
            update.dependent_rotational_bodies[0]
                .state
                .angular_acceleration_rad_per_s2,
            expected_acceleration * 0.5,
        );
    }

    #[test]
    fn mechanical_system_uses_current_effective_inertia_not_stale_dependent_acceleration() {
        let config = RotationalMassConfig {
            max_angle_rad: std::f64::consts::TAU,
            wrap_angle: true,
        };
        let mut independent_bodies = [IndependentRotationalBody {
            state: RotationalMassState::new(1.0, 0.0, 0.0, config),
            config,
            external_torque_nm: 0.0,
        }];
        independent_bodies[0].state.angular_acceleration_rad_per_s2 = 1000.0;
        let mut dependent_linear_bodies = [];
        let mut dependent_rotational_bodies = [DependentRotationalBody {
            state: RotationalMassState::new(2.0, 0.0, 0.0, config),
            config,
            parent_rotational_index: 0,
            external_torque_nm: 0.0,
            driven_per_parent_ratio: 0.5,
            phase_offset_rad: 0.0,
        }];

        let update = step_mechanical_system(
            &mut independent_bodies,
            &mut dependent_linear_bodies,
            &mut dependent_rotational_bodies,
            1.0,
        );

        assert_approx_eq(update.independent_rotational_torques_nm[0], 0.0);
        assert_approx_eq(
            update.independent_rotational_bodies[0]
                .state
                .angular_acceleration_rad_per_s2,
            0.0,
        );
        assert_approx_eq(
            update.dependent_rotational_bodies[0]
                .state
                .angular_velocity_rad_per_s,
            0.0,
        );
    }

    #[test]
    fn mechanical_system_projects_dependent_linear_force_and_inertia_to_parent() {
        let config = RotationalMassConfig {
            max_angle_rad: std::f64::consts::TAU,
            wrap_angle: true,
        };
        let mut independent_bodies = [IndependentRotationalBody {
            state: RotationalMassState::new(1.0, 0.0, 0.0, config),
            config,
            external_torque_nm: 0.0,
        }];
        let mut dependent_linear_bodies = [DependentLinearBody {
            state: LinearMassState::new(2.0, 0.0, 0.0),
            parent_rotational_index: 0,
            external_force_n: 1_000.0,
            kinematics: SolidLinearKinematics {
                position_m: 0.0,
                velocity_m_per_s: 0.0,
                acceleration_m_per_s2: 100.0,
                dx_dtheta_m_per_rad: 0.01,
            },
        }];
        let mut dependent_rotational_bodies = [];

        let update = step_mechanical_system(
            &mut independent_bodies,
            &mut dependent_linear_bodies,
            &mut dependent_rotational_bodies,
            1.0,
        );

        let expected_acceleration = 10.0 / (1.0 + 2.0 * 0.01_f64.powi(2));

        assert_approx_eq(update.independent_rotational_torques_nm[0], 10.0);
        assert_approx_eq(
            update.independent_rotational_bodies[0].state.angle_rad,
            0.5 * expected_acceleration,
        );
        assert_approx_eq(
            update.independent_rotational_bodies[0]
                .state
                .angular_velocity_rad_per_s,
            expected_acceleration,
        );
    }

    #[test]
    fn mechanical_system_projects_dependent_rotational_external_and_inertial_torque_to_parent() {
        let config = RotationalMassConfig {
            max_angle_rad: std::f64::consts::TAU,
            wrap_angle: true,
        };
        let mut independent_bodies = [IndependentRotationalBody {
            state: RotationalMassState::new(1.0, 1.0, 2.0, config),
            config,
            external_torque_nm: 3.0,
        }];
        independent_bodies[0].state.angular_acceleration_rad_per_s2 = 6.0;
        let mut dependent_linear_bodies = [];
        let mut dependent_rotational_bodies = [DependentRotationalBody {
            state: RotationalMassState::new(2.0, 0.0, 0.0, config),
            config,
            parent_rotational_index: 0,
            external_torque_nm: 10.0,
            driven_per_parent_ratio: 0.5,
            phase_offset_rad: 0.25,
        }];

        let update = step_mechanical_system(
            &mut independent_bodies,
            &mut dependent_linear_bodies,
            &mut dependent_rotational_bodies,
            0.0,
        );

        assert_approx_eq(update.dependent_rotational_bodies[0].state.angle_rad, 0.75);
        assert_approx_eq(
            update.dependent_rotational_bodies[0]
                .state
                .angular_velocity_rad_per_s,
            1.0,
        );
        assert_approx_eq(
            update.dependent_rotational_bodies[0]
                .state
                .angular_acceleration_rad_per_s2,
            8.0 / (1.0 + 2.0 * 0.5_f64.powi(2)) * 0.5,
        );
        assert_approx_eq(update.independent_rotational_torques_nm[0], 8.0);
        assert_approx_eq(
            update.independent_rotational_bodies[0]
                .state
                .angular_acceleration_rad_per_s2,
            8.0 / (1.0 + 2.0 * 0.5_f64.powi(2)),
        );
    }

    #[test]
    fn mechanical_system_supports_multiple_dependents_on_one_parent() {
        let config = RotationalMassConfig {
            max_angle_rad: std::f64::consts::TAU,
            wrap_angle: true,
        };
        let mut independent_bodies = [IndependentRotationalBody {
            state: RotationalMassState::new(1.0, 0.0, 0.0, config),
            config,
            external_torque_nm: 1.0,
        }];
        let mut dependent_linear_bodies = [
            DependentLinearBody {
                state: LinearMassState::new(1.0, 0.0, 0.0),
                parent_rotational_index: 0,
                external_force_n: 100.0,
                kinematics: SolidLinearKinematics {
                    position_m: 0.0,
                    velocity_m_per_s: 0.0,
                    acceleration_m_per_s2: 0.0,
                    dx_dtheta_m_per_rad: 0.1,
                },
            },
            DependentLinearBody {
                state: LinearMassState::new(1.0, 0.0, 0.0),
                parent_rotational_index: 0,
                external_force_n: -20.0,
                kinematics: SolidLinearKinematics {
                    position_m: 0.0,
                    velocity_m_per_s: 0.0,
                    acceleration_m_per_s2: 0.0,
                    dx_dtheta_m_per_rad: 0.5,
                },
            },
        ];
        let mut dependent_rotational_bodies = [DependentRotationalBody {
            state: RotationalMassState::new(1.0, 0.0, 0.0, config),
            config,
            parent_rotational_index: 0,
            external_torque_nm: 6.0,
            driven_per_parent_ratio: 0.5,
            phase_offset_rad: 0.0,
        }];

        let update = step_mechanical_system(
            &mut independent_bodies,
            &mut dependent_linear_bodies,
            &mut dependent_rotational_bodies,
            0.0,
        );

        assert_approx_eq(update.independent_rotational_torques_nm[0], 4.0);
    }

    #[test]
    fn mechanical_system_wraps_dependent_rotational_angles_using_its_config() {
        let driver_config = RotationalMassConfig {
            max_angle_rad: std::f64::consts::TAU,
            wrap_angle: true,
        };
        let driven_config = RotationalMassConfig {
            max_angle_rad: 1.0,
            wrap_angle: true,
        };
        let mut independent_bodies = [IndependentRotationalBody {
            state: RotationalMassState::new(1.0, 3.0, 0.0, driver_config),
            config: driver_config,
            external_torque_nm: 0.0,
        }];
        let mut dependent_linear_bodies = [];
        let mut dependent_rotational_bodies = [DependentRotationalBody {
            state: RotationalMassState::new(1.0, 0.0, 0.0, driven_config),
            config: driven_config,
            parent_rotational_index: 0,
            external_torque_nm: 0.0,
            driven_per_parent_ratio: 1.0,
            phase_offset_rad: 0.25,
        }];

        let update = step_mechanical_system(
            &mut independent_bodies,
            &mut dependent_linear_bodies,
            &mut dependent_rotational_bodies,
            0.0,
        );

        assert_approx_eq(update.dependent_rotational_bodies[0].state.angle_rad, 0.25);
    }

    #[test]
    fn solid_rotational_ratio_constraint_reports_wrapped_displacement() {
        let config = RotationalMassConfig {
            max_angle_rad: std::f64::consts::TAU,
            wrap_angle: true,
        };
        let mut rotational_states = [
            RotationalMassState::new(1.0, std::f64::consts::TAU - 0.1, 0.0, config),
            RotationalMassState::new(1.0, std::f64::consts::TAU - 0.1, 0.0, config),
        ];

        let update = enforce_solid_rotational_ratio_constraint(
            &mut rotational_states,
            config,
            SolidRotationalRatioConstraint {
                driver_rotational_index: 0,
                driven_rotational_index: 1,
                driven_per_driver_ratio: 1.0,
                phase_offset_rad: 0.2,
            },
        );

        assert_approx_eq(update.angle_rad, 0.1);
        assert_approx_eq(update.angular_displacement_rad, 0.2);
    }
}
