use std::f64::consts::PI;

pub fn bore_area_m2(bore_diameter_m: f64) -> f64 {
    assert!(bore_diameter_m > 0.0, "bore diameter must be positive");

    PI * bore_diameter_m.powi(2) / 4.0
}

pub fn swept_volume_m3(bore_area_m2: f64, stroke_depth_m: f64) -> f64 {
    assert!(bore_area_m2 > 0.0, "bore area must be positive");
    assert!(stroke_depth_m > 0.0, "stroke depth must be positive");

    bore_area_m2 * stroke_depth_m
}

pub fn clearance_volume_m3(
    bore_diameter_m: f64,
    stroke_depth_m: f64,
    compression_ratio: f64,
) -> f64 {
    assert!(
        compression_ratio > 1.0,
        "compression ratio must be greater than 1"
    );

    swept_volume_m3(bore_area_m2(bore_diameter_m), stroke_depth_m) / (compression_ratio - 1.0)
}

pub fn crank_radius_m(stroke_depth_m: f64) -> f64 {
    assert!(stroke_depth_m > 0.0, "stroke depth must be positive");

    stroke_depth_m * 0.5
}

pub fn slider_crank_cylinder_volume_m3(
    crank_angle_rad: f64,
    clearance_volume_m3: f64,
    bore_area_m2: f64,
    crank_radius_m: f64,
    connecting_rod_length_m: f64,
) -> f64 {
    assert!(
        clearance_volume_m3 > 0.0,
        "clearance volume must be positive"
    );
    assert!(bore_area_m2 > 0.0, "bore area must be positive");
    assert!(crank_radius_m > 0.0, "crank radius must be positive");
    assert!(
        connecting_rod_length_m >= crank_radius_m,
        "connecting rod length must be at least crank radius"
    );

    let sin_theta = crank_angle_rad.sin();
    let cos_theta = crank_angle_rad.cos();
    let under_sqrt = connecting_rod_length_m.powi(2) - crank_radius_m.powi(2) * sin_theta.powi(2);
    let piston_travel_from_tdc =
        crank_radius_m + connecting_rod_length_m - (crank_radius_m * cos_theta + under_sqrt.sqrt());

    clearance_volume_m3 + bore_area_m2 * piston_travel_from_tdc
}

pub fn slider_crank_dx_dtheta_m_per_rad(
    crank_angle_rad: f64,
    crank_radius_m: f64,
    connecting_rod_length_m: f64,
) -> f64 {
    assert!(crank_radius_m > 0.0, "crank radius must be positive");
    assert!(
        connecting_rod_length_m >= crank_radius_m,
        "connecting rod length must be at least crank radius"
    );

    let sin_theta = crank_angle_rad.sin();
    let cos_theta = crank_angle_rad.cos();
    let under_sqrt = connecting_rod_length_m.powi(2) - crank_radius_m.powi(2) * sin_theta.powi(2);

    crank_radius_m * sin_theta + crank_radius_m.powi(2) * sin_theta * cos_theta / under_sqrt.sqrt()
}

pub fn slider_crank_dv_dtheta_m3_per_rad(
    crank_angle_rad: f64,
    bore_area_m2: f64,
    crank_radius_m: f64,
    connecting_rod_length_m: f64,
) -> f64 {
    assert!(bore_area_m2 > 0.0, "bore area must be positive");

    bore_area_m2
        * slider_crank_dx_dtheta_m_per_rad(crank_angle_rad, crank_radius_m, connecting_rod_length_m)
}

pub fn slider_crank_volume_rate_m3_per_s(
    crank_angle_rad: f64,
    crank_speed_rad_per_s: f64,
    bore_area_m2: f64,
    crank_radius_m: f64,
    connecting_rod_length_m: f64,
) -> f64 {
    slider_crank_dv_dtheta_m3_per_rad(
        crank_angle_rad,
        bore_area_m2,
        crank_radius_m,
        connecting_rod_length_m,
    ) * crank_speed_rad_per_s
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    const EPSILON: f64 = 1.0e-12;

    fn assert_approx_eq(actual: f64, expected: f64) {
        let difference = (actual - expected).abs();
        assert!(
            difference <= EPSILON,
            "expected {expected}, got {actual}; difference {difference} exceeded {EPSILON}",
        );
    }

    #[test]
    fn returns_clearance_volume_at_top_dead_center() {
        let volume = slider_crank_cylinder_volume_m3(0.0, 10.0, 2.0, 3.0, 12.0);

        assert_approx_eq(volume, 10.0);
    }

    #[test]
    fn returns_clearance_plus_swept_volume_at_bottom_dead_center() {
        let clearance: f64 = 10.0;
        let area: f64 = 2.0;
        let crank_radius: f64 = 3.0;
        let volume = slider_crank_cylinder_volume_m3(PI, clearance, area, crank_radius, 12.0);

        assert_approx_eq(volume, clearance + area * 2.0 * crank_radius);
    }

    #[test]
    fn uses_slider_crank_geometry_for_mid_stroke_angles() {
        let clearance: f64 = 10.0;
        let area: f64 = 2.0;
        let crank_radius: f64 = 3.0;
        let rod_length: f64 = 12.0;
        let expected_travel =
            crank_radius + rod_length - (rod_length.powi(2) - crank_radius.powi(2)).sqrt();
        let volume =
            slider_crank_cylinder_volume_m3(FRAC_PI_2, clearance, area, crank_radius, rod_length);

        assert_approx_eq(volume, clearance + area * expected_travel);
    }

    #[test]
    fn returns_same_volume_for_equal_angles_before_and_after_top_dead_center() {
        let positive_angle_volume = slider_crank_cylinder_volume_m3(0.75, 10.0, 2.0, 3.0, 12.0);
        let negative_angle_volume = slider_crank_cylinder_volume_m3(-0.75, 10.0, 2.0, 3.0, 12.0);

        assert_approx_eq(positive_angle_volume, negative_angle_volume);
    }

    #[test]
    #[should_panic(expected = "compression ratio must be greater than 1")]
    fn rejects_invalid_compression_ratio() {
        clearance_volume_m3(0.086, 0.086, 1.0);
    }

    #[test]
    #[should_panic(expected = "connecting rod length must be at least crank radius")]
    fn rejects_impossible_slider_crank_geometry() {
        slider_crank_cylinder_volume_m3(0.5, 0.00005, 0.005, 0.05, 0.04);
    }

    #[test]
    fn slider_crank_derivative_is_zero_at_dead_centers() {
        assert_approx_eq(slider_crank_dx_dtheta_m_per_rad(0.0, 0.043, 0.143), 0.0);
        assert_approx_eq(slider_crank_dx_dtheta_m_per_rad(PI, 0.043, 0.143), 0.0);
    }

    #[test]
    fn slider_crank_derivative_matches_central_difference() {
        let theta = 1.2;
        let clearance = 0.00005;
        let bore_area = 0.005;
        let crank_radius = 0.043;
        let rod_length = 0.143;
        let epsilon = 1.0e-6;
        let forward = slider_crank_cylinder_volume_m3(
            theta + epsilon,
            clearance,
            bore_area,
            crank_radius,
            rod_length,
        );
        let backward = slider_crank_cylinder_volume_m3(
            theta - epsilon,
            clearance,
            bore_area,
            crank_radius,
            rod_length,
        );
        let dx_dtheta = (forward - backward) / (2.0 * epsilon) / bore_area;

        let actual = slider_crank_dx_dtheta_m_per_rad(theta, crank_radius, rod_length);
        let difference = (actual - dx_dtheta).abs();
        assert!(
            difference <= 1.0e-10,
            "expected {dx_dtheta}, got {actual}; difference {difference} exceeded 1e-10",
        );
    }

    #[test]
    fn slider_crank_volume_derivative_is_area_times_position_derivative() {
        let theta = 1.2;
        let bore_area = 0.005;
        let crank_radius = 0.043;
        let rod_length = 0.143;

        assert_approx_eq(
            slider_crank_dv_dtheta_m3_per_rad(theta, bore_area, crank_radius, rod_length),
            bore_area * slider_crank_dx_dtheta_m_per_rad(theta, crank_radius, rod_length),
        );
    }

    #[test]
    fn slider_crank_volume_rate_is_volume_derivative_times_crank_speed() {
        let theta = 1.2;
        let crank_speed = 300.0;
        let bore_area = 0.005;
        let crank_radius = 0.043;
        let rod_length = 0.143;

        assert_approx_eq(
            slider_crank_volume_rate_m3_per_s(
                theta,
                crank_speed,
                bore_area,
                crank_radius,
                rod_length,
            ),
            slider_crank_dv_dtheta_m3_per_rad(theta, bore_area, crank_radius, rod_length)
                * crank_speed,
        );
    }
}
