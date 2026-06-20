use crate::flow;

pub fn effective_area_m2(
    minimum_area_m2: f64,
    maximum_added_area_m2: f64,
    throttle_position: f64,
    maximum_throttle_position: f64,
) -> f64 {
    assert!(minimum_area_m2 >= 0.0, "minimum area must be non-negative");
    assert!(
        maximum_added_area_m2 >= 0.0,
        "maximum added area must be non-negative"
    );
    assert!(
        maximum_throttle_position > 0.0,
        "maximum throttle position must be positive"
    );

    let normalized_position = (throttle_position / maximum_throttle_position).clamp(0.0, 1.0);
    minimum_area_m2 + maximum_added_area_m2 * normalized_position.powi(2)
}

pub fn mass_flow_rate_kg_per_s(
    upstream_pressure_pa: f64,
    upstream_temperature_k: f64,
    downstream_pressure_pa: f64,
    discharge_coefficient: f64,
    effective_area_m2: f64,
    specific_heat_ratio: f64,
    gas_constant_j_per_kg_k: f64,
) -> f64 {
    flow::isentropic_mass_flow(
        upstream_pressure_pa,
        upstream_temperature_k,
        downstream_pressure_pa,
        discharge_coefficient,
        effective_area_m2,
        specific_heat_ratio,
        gas_constant_j_per_kg_k,
    )
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
    fn clamps_negative_throttle_to_minimum_area() {
        assert_approx_eq(effective_area_m2(0.001, 0.01, -0.5, 1.0), 0.001);
    }

    #[test]
    fn clamps_overrange_throttle_to_full_area() {
        assert_approx_eq(effective_area_m2(0.001, 0.01, 2.0, 1.0), 0.011);
    }

    #[test]
    fn uses_quadratic_opening_inside_range() {
        assert_approx_eq(effective_area_m2(0.001, 0.01, 0.5, 1.0), 0.0035);
    }

    #[test]
    #[should_panic(expected = "maximum throttle position must be positive")]
    fn rejects_invalid_maximum_throttle_position() {
        effective_area_m2(0.001, 0.01, 0.5, 0.0);
    }
}
