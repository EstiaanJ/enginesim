#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GasState {
    pub mass_kg: f64,
    pub temperature_k: f64,
    pub volume_m3: f64,
    pub gas_constant_j_per_kg_k: f64,
}

impl GasState {
    pub fn pressure_pa(self) -> f64 {
        pressure(
            self.mass_kg,
            self.temperature_k,
            self.volume_m3,
            self.gas_constant_j_per_kg_k,
        )
    }

    pub fn density_kg_per_m3(self) -> f64 {
        density(self.mass_kg, self.volume_m3)
    }
}

pub fn pressure(
    mass_kg: f64,
    temperature_k: f64,
    volume_m3: f64,
    gas_constant_j_per_kg_k: f64,
) -> f64 {
    if mass_kg <= 0.0 || temperature_k <= 0.0 || volume_m3 <= 0.0 || gas_constant_j_per_kg_k <= 0.0
    {
        return 0.0;
    }

    mass_kg * gas_constant_j_per_kg_k * temperature_k / volume_m3
}

pub fn density(mass_kg: f64, volume_m3: f64) -> f64 {
    if mass_kg <= 0.0 || volume_m3 <= 0.0 {
        return 0.0;
    }

    mass_kg / volume_m3
}

pub fn cv(gas_constant_j_per_kg_k: f64, specific_heat_ratio: f64) -> f64 {
    if gas_constant_j_per_kg_k <= 0.0 || specific_heat_ratio <= 1.0 {
        return 0.0;
    }

    gas_constant_j_per_kg_k / (specific_heat_ratio - 1.0)
}

pub fn cp(gas_constant_j_per_kg_k: f64, specific_heat_ratio: f64) -> f64 {
    if gas_constant_j_per_kg_k <= 0.0 || specific_heat_ratio <= 1.0 {
        return 0.0;
    }

    specific_heat_ratio * gas_constant_j_per_kg_k / (specific_heat_ratio - 1.0)
}

pub fn temperature_from_internal_energy_k(
    internal_energy_j: f64,
    mass_kg: f64,
    cv_j_per_kg_k: f64,
) -> f64 {
    if internal_energy_j <= 0.0 || mass_kg <= 0.0 || cv_j_per_kg_k <= 0.0 {
        return 0.0;
    }

    internal_energy_j / (mass_kg * cv_j_per_kg_k)
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
    fn ideal_gas_pressure_matches_mass_r_t_over_volume() {
        assert_approx_eq(pressure(0.1, 300.0, 0.01, 287.0), 861_000.0);
    }

    #[test]
    fn density_matches_mass_over_volume() {
        assert_approx_eq(density(0.1, 0.01), 10.0);
    }

    #[test]
    fn heat_capacities_match_gamma_relationships() {
        let cv = cv(287.0, 1.4);
        let cp = cp(287.0, 1.4);

        assert_approx_eq(cv, 717.5);
        assert_approx_eq(cp, 1004.5);
        assert_approx_eq(cp - cv, 287.0);
    }

    #[test]
    fn temperature_from_internal_energy_inverts_internal_energy() {
        let cv = cv(287.0, 1.4);
        let internal_energy = 0.1 * cv * 300.0;

        assert_approx_eq(
            temperature_from_internal_energy_k(internal_energy, 0.1, cv),
            300.0,
        );
    }
}
