pub const POSITIVE_CRANK_ROTATION: &str = "increasing crank angle in radians";
pub const POSITIVE_PISTON_DISPLACEMENT: &str = "piston travel away from top dead center";
pub const POSITIVE_GAS_FORCE: &str = "force pushing piston away from top dead center";
pub const POSITIVE_GAS_TORQUE: &str = "torque that increases positive crank speed";
pub const POSITIVE_LOAD_TORQUE: &str = "external torque that increases positive crank speed";
pub const POSITIVE_FRICTION_TORQUE: &str = "friction torque has the opposite sign of crank speed";

pub fn gas_torque_from_force(force_n: f64, dx_dtheta_m_per_rad: f64) -> f64 {
    force_n * dx_dtheta_m_per_rad
}

pub fn friction_torque_nm(crank_speed_rad_per_s: f64, friction_magnitude_nm: f64) -> f64 {
    assert!(
        friction_magnitude_nm >= 0.0,
        "friction magnitude must be non-negative"
    );

    if crank_speed_rad_per_s > 0.0 {
        -friction_magnitude_nm
    } else if crank_speed_rad_per_s < 0.0 {
        friction_magnitude_nm
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gas_torque_sign_follows_force_and_geometry_derivative() {
        assert!(gas_torque_from_force(100.0, 0.01) > 0.0);
        assert!(gas_torque_from_force(100.0, -0.01) < 0.0);
        assert!(gas_torque_from_force(-100.0, 0.01) < 0.0);
    }

    #[test]
    fn friction_torque_opposes_motion() {
        assert_eq!(friction_torque_nm(10.0, 2.0), -2.0);
        assert_eq!(friction_torque_nm(-10.0, 2.0), 2.0);
        assert_eq!(friction_torque_nm(0.0, 2.0), 0.0);
    }
}
