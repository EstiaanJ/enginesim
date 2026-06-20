#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlowEndpoint {
    pub pressure_pa: f64,
    pub temperature_k: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlowOrifice {
    pub discharge_coefficient: f64,
    pub area_m2: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GasFlowProperties {
    pub specific_heat_ratio: f64,
    pub gas_constant_j_per_kg_k: f64,
}

pub fn critical_pressure_ratio(specific_heat_ratio: f64) -> f64 {
    if specific_heat_ratio <= 1.0 {
        return 0.0;
    }

    let gamma = specific_heat_ratio;
    (2.0 / (gamma + 1.0)).powf(gamma / (gamma - 1.0))
}

pub fn is_critical_flow(
    upstream_pressure_pa: f64,
    downstream_pressure_pa: f64,
    specific_heat_ratio: f64,
) -> bool {
    if upstream_pressure_pa <= 0.0 || downstream_pressure_pa < 0.0 || specific_heat_ratio <= 1.0 {
        return false;
    }

    downstream_pressure_pa / upstream_pressure_pa <= critical_pressure_ratio(specific_heat_ratio)
}

pub fn isentropic_mass_flow(
    upstream_pressure_pa: f64,
    upstream_temperature_k: f64,
    downstream_pressure_pa: f64,
    discharge_coefficient: f64,
    area_m2: f64,
    specific_heat_ratio: f64,
    gas_constant_j_per_kg_k: f64,
) -> f64 {
    if area_m2 <= 0.0
        || discharge_coefficient <= 0.0
        || upstream_pressure_pa <= 0.0
        || upstream_temperature_k <= 0.0
        || downstream_pressure_pa < 0.0
        || downstream_pressure_pa >= upstream_pressure_pa
        || specific_heat_ratio <= 1.0
        || gas_constant_j_per_kg_k <= 0.0
    {
        return 0.0;
    }

    if is_critical_flow(
        upstream_pressure_pa,
        downstream_pressure_pa,
        specific_heat_ratio,
    ) {
        choked_isentropic_mass_flow(
            upstream_pressure_pa,
            upstream_temperature_k,
            discharge_coefficient,
            area_m2,
            specific_heat_ratio,
            gas_constant_j_per_kg_k,
        )
    } else {
        unchoked_isentropic_mass_flow(
            upstream_pressure_pa,
            upstream_temperature_k,
            downstream_pressure_pa,
            discharge_coefficient,
            area_m2,
            specific_heat_ratio,
            gas_constant_j_per_kg_k,
        )
    }
}

pub fn bidirectional_isentropic_mass_flow(
    first: FlowEndpoint,
    second: FlowEndpoint,
    orifice: FlowOrifice,
    properties: GasFlowProperties,
) -> f64 {
    if first.pressure_pa >= second.pressure_pa {
        isentropic_mass_flow(
            first.pressure_pa,
            first.temperature_k,
            second.pressure_pa,
            orifice.discharge_coefficient,
            orifice.area_m2,
            properties.specific_heat_ratio,
            properties.gas_constant_j_per_kg_k,
        )
    } else {
        -isentropic_mass_flow(
            second.pressure_pa,
            second.temperature_k,
            first.pressure_pa,
            orifice.discharge_coefficient,
            orifice.area_m2,
            properties.specific_heat_ratio,
            properties.gas_constant_j_per_kg_k,
        )
    }
}

pub fn choked_isentropic_mass_flow(
    upstream_pressure_pa: f64,
    upstream_temperature_k: f64,
    discharge_coefficient: f64,
    area_m2: f64,
    specific_heat_ratio: f64,
    gas_constant_j_per_kg_k: f64,
) -> f64 {
    if area_m2 <= 0.0
        || discharge_coefficient <= 0.0
        || upstream_pressure_pa <= 0.0
        || upstream_temperature_k <= 0.0
        || specific_heat_ratio <= 1.0
        || gas_constant_j_per_kg_k <= 0.0
    {
        return 0.0;
    }

    let gamma = specific_heat_ratio;
    discharge_coefficient
        * area_m2
        * upstream_pressure_pa
        * (gamma / (gas_constant_j_per_kg_k * upstream_temperature_k)).sqrt()
        * (2.0 / (gamma + 1.0)).powf((gamma + 1.0) / (2.0 * (gamma - 1.0)))
}

pub fn unchoked_isentropic_mass_flow(
    upstream_pressure_pa: f64,
    upstream_temperature_k: f64,
    downstream_pressure_pa: f64,
    discharge_coefficient: f64,
    area_m2: f64,
    specific_heat_ratio: f64,
    gas_constant_j_per_kg_k: f64,
) -> f64 {
    if area_m2 <= 0.0
        || discharge_coefficient <= 0.0
        || upstream_pressure_pa <= 0.0
        || upstream_temperature_k <= 0.0
        || downstream_pressure_pa < 0.0
        || downstream_pressure_pa >= upstream_pressure_pa
        || specific_heat_ratio <= 1.0
        || gas_constant_j_per_kg_k <= 0.0
    {
        return 0.0;
    }

    let gamma = specific_heat_ratio;
    let pressure_ratio = downstream_pressure_pa / upstream_pressure_pa;
    let term = pressure_ratio.powf(2.0 / gamma) - pressure_ratio.powf((gamma + 1.0) / gamma);
    if term <= 0.0 {
        return 0.0;
    }

    discharge_coefficient
        * area_m2
        * upstream_pressure_pa
        * ((2.0 * gamma / (gas_constant_j_per_kg_k * upstream_temperature_k * (gamma - 1.0)))
            * term)
            .sqrt()
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
    fn bidirectional_flow_is_positive_when_first_pressure_is_higher() {
        let mass_flow_rate_kg_per_s = bidirectional_isentropic_mass_flow(
            FlowEndpoint {
                pressure_pa: 120_000.0,
                temperature_k: 300.0,
            },
            FlowEndpoint {
                pressure_pa: 100_000.0,
                temperature_k: 310.0,
            },
            FlowOrifice {
                discharge_coefficient: 0.7,
                area_m2: 0.0001,
            },
            GasFlowProperties {
                specific_heat_ratio: 1.4,
                gas_constant_j_per_kg_k: 287.0,
            },
        );

        assert!(mass_flow_rate_kg_per_s > 0.0);
    }

    #[test]
    fn bidirectional_flow_is_negative_when_second_pressure_is_higher() {
        let mass_flow_rate_kg_per_s = bidirectional_isentropic_mass_flow(
            FlowEndpoint {
                pressure_pa: 100_000.0,
                temperature_k: 300.0,
            },
            FlowEndpoint {
                pressure_pa: 120_000.0,
                temperature_k: 310.0,
            },
            FlowOrifice {
                discharge_coefficient: 0.7,
                area_m2: 0.0001,
            },
            GasFlowProperties {
                specific_heat_ratio: 1.4,
                gas_constant_j_per_kg_k: 287.0,
            },
        );

        assert!(mass_flow_rate_kg_per_s < 0.0);
    }

    #[test]
    fn returns_zero_when_pressures_are_equal() {
        assert_approx_eq(
            isentropic_mass_flow(100_000.0, 300.0, 100_000.0, 0.7, 0.0001, 1.4, 287.0),
            0.0,
        );
    }

    #[test]
    fn mass_flow_increases_with_area() {
        let small_area = isentropic_mass_flow(120_000.0, 300.0, 100_000.0, 0.7, 0.0001, 1.4, 287.0);
        let large_area = isentropic_mass_flow(120_000.0, 300.0, 100_000.0, 0.7, 0.0002, 1.4, 287.0);

        assert!(large_area > small_area);
    }

    #[test]
    fn choked_flow_is_independent_of_downstream_pressure_below_critical_ratio() {
        let choked_at_low_pressure =
            isentropic_mass_flow(200_000.0, 300.0, 50_000.0, 0.7, 0.0001, 1.4, 287.0);
        let choked_at_lower_pressure =
            isentropic_mass_flow(200_000.0, 300.0, 20_000.0, 0.7, 0.0001, 1.4, 287.0);

        assert_approx_eq(choked_at_low_pressure, choked_at_lower_pressure);
    }
}
