use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct MixtureLimits {
    pub stoichiometric_air_fuel_ratio: f64,
    pub minimum_combustible_air_fuel_ratio: f64,
    pub maximum_combustible_air_fuel_ratio: f64,
    pub peak_rich_equivalence_ratio: f64,
    pub peak_lean_equivalence_ratio: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct WiebeParameters {
    pub combustion_duration_rad: f64,
    pub efficiency_coefficient: f64,
    pub shape_factor: f64,
}

pub fn air_fuel_ratio(air_mass_kg: f64, fuel_mass_kg: f64) -> f64 {
    if fuel_mass_kg <= 0.0 {
        return f64::INFINITY;
    }

    air_mass_kg / fuel_mass_kg
}

pub fn stoich_limited_fuel_mass_kg(
    fuel_mass_kg: f64,
    air_fuel_ratio: f64,
    stoichiometric_air_fuel_ratio: f64,
) -> f64 {
    if fuel_mass_kg <= 0.0 || stoichiometric_air_fuel_ratio <= 0.0 {
        return 0.0;
    }

    if air_fuel_ratio >= stoichiometric_air_fuel_ratio {
        fuel_mass_kg
    } else {
        fuel_mass_kg * air_fuel_ratio / stoichiometric_air_fuel_ratio
    }
}

pub fn mixture_combustion_efficiency(air_fuel_ratio: f64, limits: MixtureLimits) -> f64 {
    if !air_fuel_ratio.is_finite() || limits.stoichiometric_air_fuel_ratio <= 0.0 {
        return 0.0;
    }

    let equivalence_ratio = limits.stoichiometric_air_fuel_ratio / air_fuel_ratio;
    let rich_limit = limits.stoichiometric_air_fuel_ratio
        / limits
            .minimum_combustible_air_fuel_ratio
            .max(f64::MIN_POSITIVE);
    let lean_limit = limits.stoichiometric_air_fuel_ratio
        / limits
            .maximum_combustible_air_fuel_ratio
            .max(f64::MIN_POSITIVE);
    let peak_rich = limits.peak_rich_equivalence_ratio.max(1.0);
    let peak_lean = limits.peak_lean_equivalence_ratio.min(1.0);

    if equivalence_ratio >= rich_limit || equivalence_ratio <= lean_limit {
        0.0
    } else if equivalence_ratio > peak_rich {
        ((rich_limit - equivalence_ratio) / (rich_limit - peak_rich)).clamp(0.0, 1.0)
    } else if equivalence_ratio < peak_lean {
        ((equivalence_ratio - lean_limit) / (peak_lean - lean_limit)).clamp(0.0, 1.0)
    } else {
        1.0
    }
}

pub fn cumulative_wiebe_burned_fraction(
    elapsed_angle_rad: f64,
    parameters: WiebeParameters,
) -> f64 {
    if elapsed_angle_rad <= 0.0
        || parameters.combustion_duration_rad <= 0.0
        || parameters.efficiency_coefficient <= 0.0
    {
        return 0.0;
    }

    let normalized_angle = (elapsed_angle_rad / parameters.combustion_duration_rad).clamp(0.0, 1.0);
    let shape_exponent = parameters.shape_factor + 1.0;
    if shape_exponent <= 0.0 {
        return 0.0;
    }

    1.0 - (-parameters.efficiency_coefficient * normalized_angle.powf(shape_exponent)).exp()
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

    fn limits() -> MixtureLimits {
        MixtureLimits {
            stoichiometric_air_fuel_ratio: 14.7,
            minimum_combustible_air_fuel_ratio: 8.0,
            maximum_combustible_air_fuel_ratio: 25.0,
            peak_rich_equivalence_ratio: 1.10,
            peak_lean_equivalence_ratio: 0.95,
        }
    }

    #[test]
    fn limits_rich_burnable_fuel_to_available_air() {
        let fuel_mass_kg = 0.00005;
        let afr = 10.0;

        assert_approx_eq(
            stoich_limited_fuel_mass_kg(fuel_mass_kg, afr, 14.7),
            fuel_mass_kg * afr / 14.7,
        );
    }

    #[test]
    fn penalizes_extreme_mixture_ratios() {
        assert_approx_eq(mixture_combustion_efficiency(7.5, limits()), 0.0);
        assert_approx_eq(mixture_combustion_efficiency(26.0, limits()), 0.0);
        assert_approx_eq(mixture_combustion_efficiency(14.7, limits()), 1.0);
    }

    #[test]
    fn wiebe_duration_does_not_force_exactly_complete_combustion() {
        let parameters = WiebeParameters {
            combustion_duration_rad: 40.0_f64.to_radians(),
            efficiency_coefficient: 5.0,
            shape_factor: 2.0,
        };

        assert_approx_eq(
            cumulative_wiebe_burned_fraction(parameters.combustion_duration_rad, parameters),
            1.0 - (-5.0_f64).exp(),
        );
    }

    #[test]
    fn wiebe_fraction_starts_at_zero() {
        let parameters = WiebeParameters {
            combustion_duration_rad: 40.0_f64.to_radians(),
            efficiency_coefficient: 5.0,
            shape_factor: 2.0,
        };

        assert_approx_eq(cumulative_wiebe_burned_fraction(0.0, parameters), 0.0);
    }

    #[test]
    fn wiebe_fraction_is_monotonic_over_event() {
        let parameters = WiebeParameters {
            combustion_duration_rad: 40.0_f64.to_radians(),
            efficiency_coefficient: 5.0,
            shape_factor: 2.0,
        };
        let early = cumulative_wiebe_burned_fraction(5.0_f64.to_radians(), parameters);
        let middle = cumulative_wiebe_burned_fraction(20.0_f64.to_radians(), parameters);
        let late = cumulative_wiebe_burned_fraction(35.0_f64.to_radians(), parameters);

        assert!(early < middle);
        assert!(middle < late);
    }

    #[test]
    fn larger_wiebe_coefficient_burns_faster() {
        let slow = WiebeParameters {
            combustion_duration_rad: 40.0_f64.to_radians(),
            efficiency_coefficient: 2.0,
            shape_factor: 2.0,
        };
        let fast = WiebeParameters {
            efficiency_coefficient: 5.0,
            ..slow
        };

        assert!(
            cumulative_wiebe_burned_fraction(20.0_f64.to_radians(), fast)
                > cumulative_wiebe_burned_fraction(20.0_f64.to_radians(), slow)
        );
    }
}
