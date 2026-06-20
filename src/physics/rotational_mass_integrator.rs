#[derive(Debug, Clone, Copy)]
pub struct RotationalMassConfig {
    /// Maximum valid angle [rad]
    pub max_angle_rad: f64,

    /// Wrap angle back into [0, max_angle_rad) when it crosses the range.
    /// Use false for bounded components such as throttle blades.
    pub wrap_angle: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct RotationalMassState {
    /// Moment of inertia [kg*m^2]
    pub moment_of_inertia_kg_m2: f64,

    /// Current angle [rad], kept in the configured angle range
    pub angle_rad: f64,

    /// Previous angle before the last integration step [rad]
    pub previous_angle_rad: f64,

    /// Angular velocity [rad/s]
    pub angular_velocity_rad_per_s: f64,

    /// Angular acceleration from the last integration step [rad/s^2]
    pub angular_acceleration_rad_per_s2: f64,

    /// Angle change from the last integration step before wrapping/clamping [rad]
    pub angular_displacement_rad: f64,
}

impl RotationalMassState {
    pub fn new(
        moment_of_inertia_kg_m2: f64,
        angle_rad: f64,
        angular_velocity_rad_per_s: f64,
        config: RotationalMassConfig,
    ) -> Self {
        assert!(
            moment_of_inertia_kg_m2 > 0.0,
            "moment of inertia must be positive"
        );
        validate_config(config);

        let angle_rad = bounded_angle_rad(angle_rad, config);

        Self {
            moment_of_inertia_kg_m2,
            angle_rad,
            previous_angle_rad: angle_rad,
            angular_velocity_rad_per_s,
            angular_acceleration_rad_per_s2: 0.0,
            angular_displacement_rad: 0.0,
        }
    }
}

pub fn update_rotational_mass(
    state: &mut RotationalMassState,
    config: RotationalMassConfig,
    timestep_seconds: f64,
    net_torque_nm: f64,
) -> RotationalMassState {
    assert!(
        state.moment_of_inertia_kg_m2 > 0.0,
        "moment of inertia must be positive"
    );
    assert!(timestep_seconds >= 0.0, "timestep must be non-negative");
    validate_config(config);

    let initial_angle_rad = state.angle_rad;
    let initial_angular_velocity_rad_per_s = state.angular_velocity_rad_per_s;
    let angular_acceleration_rad_per_s2 = net_torque_nm / state.moment_of_inertia_kg_m2;
    let angular_displacement_rad = initial_angular_velocity_rad_per_s * timestep_seconds
        + 0.5 * angular_acceleration_rad_per_s2 * timestep_seconds.powi(2);

    let unconstrained_angle_rad = initial_angle_rad + angular_displacement_rad;
    let constrained_angle_rad = bounded_angle_rad(unconstrained_angle_rad, config);
    let mut angular_velocity_rad_per_s =
        initial_angular_velocity_rad_per_s + angular_acceleration_rad_per_s2 * timestep_seconds;

    if !config.wrap_angle
        && timestep_seconds > 0.0
        && (unconstrained_angle_rad < 0.0 || unconstrained_angle_rad > config.max_angle_rad)
    {
        angular_velocity_rad_per_s = 0.0;
    }

    state.previous_angle_rad = initial_angle_rad;
    state.angle_rad = constrained_angle_rad;
    state.angular_velocity_rad_per_s = angular_velocity_rad_per_s;
    state.angular_acceleration_rad_per_s2 = angular_acceleration_rad_per_s2;
    state.angular_displacement_rad = angular_displacement_rad;

    *state
}

fn validate_config(config: RotationalMassConfig) {
    assert!(config.max_angle_rad > 0.0, "max angle must be positive");
}

fn bounded_angle_rad(angle_rad: f64, config: RotationalMassConfig) -> f64 {
    if config.wrap_angle {
        angle_rad.rem_euclid(config.max_angle_rad)
    } else {
        angle_rad.clamp(0.0, config.max_angle_rad)
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
    fn integrates_rotational_motion_from_net_torque() {
        let config = RotationalMassConfig {
            max_angle_rad: std::f64::consts::TAU,
            wrap_angle: true,
        };
        let mut state = RotationalMassState::new(2.0, 1.0, 3.0, config);

        let update = update_rotational_mass(&mut state, config, 0.5, 8.0);

        assert_approx_eq(update.angular_acceleration_rad_per_s2, 4.0);
        assert_approx_eq(update.angular_displacement_rad, 2.0);
        assert_approx_eq(update.angle_rad, 3.0);
        assert_approx_eq(update.angular_velocity_rad_per_s, 5.0);
        assert_approx_eq(update.previous_angle_rad, 1.0);
    }

    #[test]
    fn wraps_angle_when_configured_to_wrap() {
        let config = RotationalMassConfig {
            max_angle_rad: std::f64::consts::TAU,
            wrap_angle: true,
        };
        let mut state = RotationalMassState::new(1.0, std::f64::consts::TAU - 0.1, 1.0, config);

        let update = update_rotational_mass(&mut state, config, 0.2, 0.0);

        assert_approx_eq(update.angle_rad, 0.1);
        assert_approx_eq(update.angular_velocity_rad_per_s, 1.0);
    }

    #[test]
    fn clamps_angle_and_stops_velocity_when_not_wrapping() {
        let config = RotationalMassConfig {
            max_angle_rad: 1.0,
            wrap_angle: false,
        };
        let mut state = RotationalMassState::new(1.0, 0.9, 2.0, config);

        let update = update_rotational_mass(&mut state, config, 0.2, 0.0);

        assert_approx_eq(update.angle_rad, 1.0);
        assert_approx_eq(update.angular_velocity_rad_per_s, 0.0);
    }

    #[test]
    fn clamps_initial_non_wrapping_angle_into_range() {
        let config = RotationalMassConfig {
            max_angle_rad: 1.0,
            wrap_angle: false,
        };

        let state = RotationalMassState::new(1.0, -0.5, 0.0, config);

        assert_approx_eq(state.angle_rad, 0.0);
    }
}
