#[derive(Debug, Clone, Copy)]
pub struct LinearMassState {
    /// Moving mass [kg]
    pub mass_kg: f64,

    /// Position along the integration axis [m]
    pub position_m: f64,

    /// Previous position before the last integration step [m]
    pub previous_position_m: f64,

    /// Velocity along the integration axis [m/s]
    pub velocity_m_per_s: f64,

    /// Acceleration from the last integration step [m/s^2]
    pub acceleration_m_per_s2: f64,

    /// Position change from the last integration step [m]
    pub displacement_m: f64,
}

impl LinearMassState {
    pub fn new(mass_kg: f64, position_m: f64, velocity_m_per_s: f64) -> Self {
        assert!(mass_kg > 0.0, "mass must be positive");

        Self {
            mass_kg,
            position_m,
            previous_position_m: position_m,
            velocity_m_per_s,
            acceleration_m_per_s2: 0.0,
            displacement_m: 0.0,
        }
    }
}

pub fn update_linear_mass(
    state: &mut LinearMassState,
    timestep_seconds: f64,
    net_force_n: f64,
) -> LinearMassState {
    assert!(state.mass_kg > 0.0, "mass must be positive");
    assert!(timestep_seconds >= 0.0, "timestep must be non-negative");

    let initial_position_m = state.position_m;
    let initial_velocity_m_per_s = state.velocity_m_per_s;
    let acceleration_m_per_s2 = net_force_n / state.mass_kg;
    let displacement_m = initial_velocity_m_per_s * timestep_seconds
        + 0.5 * acceleration_m_per_s2 * timestep_seconds.powi(2);

    state.previous_position_m = initial_position_m;
    state.position_m = initial_position_m + displacement_m;
    state.velocity_m_per_s = initial_velocity_m_per_s + acceleration_m_per_s2 * timestep_seconds;
    state.acceleration_m_per_s2 = acceleration_m_per_s2;
    state.displacement_m = displacement_m;

    *state
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
    fn integrates_linear_motion_from_net_force() {
        let mut state = LinearMassState::new(2.0, 1.0, 3.0);

        let update = update_linear_mass(&mut state, 0.5, 8.0);

        assert_approx_eq(update.acceleration_m_per_s2, 4.0);
        assert_approx_eq(update.displacement_m, 2.0);
        assert_approx_eq(update.position_m, 3.0);
        assert_approx_eq(update.velocity_m_per_s, 5.0);
        assert_approx_eq(update.previous_position_m, 1.0);
    }

    #[test]
    fn zero_timestep_preserves_position_and_velocity() {
        let mut state = LinearMassState::new(1.5, 2.0, -4.0);

        let update = update_linear_mass(&mut state, 0.0, 12.0);

        assert_approx_eq(update.position_m, 2.0);
        assert_approx_eq(update.velocity_m_per_s, -4.0);
        assert_approx_eq(update.displacement_m, 0.0);
        assert_approx_eq(update.acceleration_m_per_s2, 8.0);
    }
}
