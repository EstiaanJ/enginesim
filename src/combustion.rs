use serde::{Deserialize, Serialize};

use crate::physics::chamber::{ChamberSpeciesMasses, DRY_AIR_OXYGEN_MASS_FRACTION};

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct MixtureLimits {
    pub stoichiometric_air_fuel_ratio: f64,
    pub minimum_combustible_air_fuel_ratio: f64,
    pub maximum_combustible_air_fuel_ratio: f64,
    pub peak_rich_equivalence_ratio: f64,
    pub peak_lean_equivalence_ratio: f64,
}

pub fn default_mixture_limits() -> MixtureLimits {
    MixtureLimits {
        stoichiometric_air_fuel_ratio: 14.7,
        // Rich flammability limit at equivalence ratio phi = 1.96 (AFR = 14.7 / 1.96).
        minimum_combustible_air_fuel_ratio: 14.7 / 1.96,
        // Lean flammability limit at equivalence ratio phi ~= 0.563
        // (AFR = 14.7 / 0.563 = 26.110...). Kept as the exact literal so the
        // default mixture limits are bit-stable across builds.
        maximum_combustible_air_fuel_ratio: 26.1101243339254,
        peak_rich_equivalence_ratio: 1.10,
        peak_lean_equivalence_ratio: 0.95,
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct WiebeParameters {
    pub combustion_duration_rad: f64,
    pub efficiency_coefficient: f64,
    pub shape_factor: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CombustionConversion {
    pub consumed_fuel_kg: f64,
    pub consumed_oxygen_kg: f64,
    pub generated_products_kg: f64,
    pub released_heat_j: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IgnitionDelayInputs {
    pub reference_delay_rad: f64,
    pub pressure_pa: f64,
    pub temperature_k: f64,
    pub equivalence_ratio: f64,
    pub residual_fraction: f64,
    pub spark_energy_j: f64,
    pub turbulence_intensity: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BurnDurationInputs {
    pub reference_duration_rad: f64,
    pub pressure_pa: f64,
    pub temperature_k: f64,
    pub equivalence_ratio: f64,
    pub residual_fraction: f64,
    pub crank_speed_rad_per_s: f64,
    pub turbulence_intensity: f64,
    pub chamber_characteristic_length_m: f64,
}

pub fn air_fuel_ratio(air_mass_kg: f64, fuel_mass_kg: f64) -> f64 {
    if fuel_mass_kg <= 0.0 {
        return f64::INFINITY;
    }

    air_mass_kg / fuel_mass_kg
}

pub fn equivalence_ratio_from_air_fuel_ratio(
    air_fuel_ratio: f64,
    stoichiometric_air_fuel_ratio: f64,
) -> f64 {
    if !air_fuel_ratio.is_finite() || air_fuel_ratio <= 0.0 || stoichiometric_air_fuel_ratio <= 0.0
    {
        return 0.0;
    }

    stoichiometric_air_fuel_ratio / air_fuel_ratio
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

pub fn stoichiometric_oxygen_fuel_ratio(stoichiometric_air_fuel_ratio: f64) -> f64 {
    if stoichiometric_air_fuel_ratio <= 0.0 {
        return 0.0;
    }

    stoichiometric_air_fuel_ratio * DRY_AIR_OXYGEN_MASS_FRACTION
}

pub fn burn_species(
    species: &mut ChamberSpeciesMasses,
    requested_fuel_kg: f64,
    stoichiometric_air_fuel_ratio: f64,
    lower_heating_value_j_per_kg: f64,
    combustion_efficiency: f64,
    heat_loss_fraction: f64,
) -> CombustionConversion {
    if requested_fuel_kg <= 0.0
        || stoichiometric_air_fuel_ratio <= 0.0
        || lower_heating_value_j_per_kg <= 0.0
        || combustion_efficiency <= 0.0
    {
        return CombustionConversion {
            consumed_fuel_kg: 0.0,
            consumed_oxygen_kg: 0.0,
            generated_products_kg: 0.0,
            released_heat_j: 0.0,
        };
    }

    let oxygen_fuel_ratio = stoichiometric_oxygen_fuel_ratio(stoichiometric_air_fuel_ratio);
    if oxygen_fuel_ratio <= 0.0 {
        return CombustionConversion {
            consumed_fuel_kg: 0.0,
            consumed_oxygen_kg: 0.0,
            generated_products_kg: 0.0,
            released_heat_j: 0.0,
        };
    }

    let target_fuel_kg =
        requested_fuel_kg.min(species.fuel_kg).max(0.0) * combustion_efficiency.clamp(0.0, 1.0);
    let oxygen_limited_fuel_kg = species.oxygen_kg.max(0.0) / oxygen_fuel_ratio;
    let consumed_fuel_kg = target_fuel_kg.min(oxygen_limited_fuel_kg).max(0.0);
    let consumed_oxygen_kg = consumed_fuel_kg * oxygen_fuel_ratio;
    let generated_products_kg = consumed_fuel_kg + consumed_oxygen_kg;
    let released_heat_j = consumed_fuel_kg
        * lower_heating_value_j_per_kg
        * (1.0 - heat_loss_fraction).clamp(0.0, 1.0);

    species.fuel_kg = (species.fuel_kg - consumed_fuel_kg).max(0.0);
    species.oxygen_kg = (species.oxygen_kg - consumed_oxygen_kg).max(0.0);
    species.products_kg += generated_products_kg;

    CombustionConversion {
        consumed_fuel_kg,
        consumed_oxygen_kg,
        generated_products_kg,
        released_heat_j,
    }
}

pub fn mixture_combustion_efficiency(air_fuel_ratio: f64, limits: MixtureLimits) -> f64 {
    if !air_fuel_ratio.is_finite() || limits.stoichiometric_air_fuel_ratio <= 0.0 {
        return 0.0;
    }

    let equivalence_ratio =
        equivalence_ratio_from_air_fuel_ratio(air_fuel_ratio, limits.stoichiometric_air_fuel_ratio);
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

pub fn equivalence_ratio_ignition_delay_multiplier(
    equivalence_ratio: f64,
    limits: MixtureLimits,
) -> f64 {
    if !equivalence_ratio.is_finite() || equivalence_ratio <= 0.0 {
        return f64::INFINITY;
    }

    let rich_limit = limits.stoichiometric_air_fuel_ratio
        / limits
            .minimum_combustible_air_fuel_ratio
            .max(f64::MIN_POSITIVE);
    let lean_limit = limits.stoichiometric_air_fuel_ratio
        / limits
            .maximum_combustible_air_fuel_ratio
            .max(f64::MIN_POSITIVE);
    if equivalence_ratio >= rich_limit || equivalence_ratio <= lean_limit {
        return f64::INFINITY;
    }

    let fastest_phi = 1.05;
    let distance = if equivalence_ratio >= fastest_phi {
        (equivalence_ratio - fastest_phi) / (rich_limit - fastest_phi).max(f64::MIN_POSITIVE)
    } else {
        (fastest_phi - equivalence_ratio) / (fastest_phi - lean_limit).max(f64::MIN_POSITIVE)
    };

    (0.9 + 1.6 * distance.clamp(0.0, 1.0)).clamp(0.9, 2.5)
}

pub fn equivalence_ratio_burn_duration_multiplier(
    equivalence_ratio: f64,
    limits: MixtureLimits,
) -> f64 {
    if !equivalence_ratio.is_finite() || equivalence_ratio <= 0.0 {
        return f64::INFINITY;
    }

    let rich_limit = limits.stoichiometric_air_fuel_ratio
        / limits
            .minimum_combustible_air_fuel_ratio
            .max(f64::MIN_POSITIVE);
    let lean_limit = limits.stoichiometric_air_fuel_ratio
        / limits
            .maximum_combustible_air_fuel_ratio
            .max(f64::MIN_POSITIVE);
    if equivalence_ratio >= rich_limit || equivalence_ratio <= lean_limit {
        return f64::INFINITY;
    }

    let fastest_phi = limits.peak_rich_equivalence_ratio.clamp(1.0, rich_limit);
    if equivalence_ratio >= fastest_phi {
        let span = (rich_limit - fastest_phi).max(f64::MIN_POSITIVE);
        let blend = ((equivalence_ratio - fastest_phi) / span).clamp(0.0, 1.0);
        0.9 + 1.1 * blend
    } else if equivalence_ratio >= 1.0 {
        let span = (fastest_phi - 1.0).max(f64::MIN_POSITIVE);
        let blend = ((fastest_phi - equivalence_ratio) / span).clamp(0.0, 1.0);
        0.9 + 0.1 * blend
    } else if equivalence_ratio >= limits.peak_lean_equivalence_ratio {
        let span = (1.0 - limits.peak_lean_equivalence_ratio).max(f64::MIN_POSITIVE);
        let blend = ((1.0 - equivalence_ratio) / span).clamp(0.0, 1.0);
        1.0 + 0.15 * blend
    } else {
        let span = (limits.peak_lean_equivalence_ratio - lean_limit).max(f64::MIN_POSITIVE);
        let blend =
            ((limits.peak_lean_equivalence_ratio - equivalence_ratio) / span).clamp(0.0, 1.0);
        1.15 + 1.05 * blend
    }
}

pub fn ignition_delay_rad(inputs: IgnitionDelayInputs, limits: MixtureLimits) -> f64 {
    if inputs.reference_delay_rad <= 0.0 {
        return 0.0;
    }

    let pressure_factor = (101_325.0 / inputs.pressure_pa.max(1.0))
        .powf(0.2)
        .clamp(0.65, 1.5);
    let temperature_factor = (700.0 / inputs.temperature_k.max(1.0))
        .sqrt()
        .clamp(0.6, 1.8);
    let mixture_factor =
        equivalence_ratio_ignition_delay_multiplier(inputs.equivalence_ratio, limits);
    let residual_factor = 1.0 + 2.0 * inputs.residual_fraction.clamp(0.0, 1.0);
    let spark_energy_factor = (0.04 / inputs.spark_energy_j.max(0.005))
        .powf(0.1)
        .clamp(0.8, 1.25);
    let turbulence_factor = (1.0 / (1.0 + inputs.turbulence_intensity.max(0.0)))
        .powf(0.15)
        .clamp(0.65, 1.0);

    inputs.reference_delay_rad
        * pressure_factor
        * temperature_factor
        * mixture_factor
        * residual_factor
        * spark_energy_factor
        * turbulence_factor
}

pub fn burn_duration_rad(inputs: BurnDurationInputs, limits: MixtureLimits) -> f64 {
    if inputs.reference_duration_rad <= 0.0 {
        return 0.0;
    }

    let pressure_factor = (101_325.0 / inputs.pressure_pa.max(1.0))
        .powf(0.1)
        .clamp(0.75, 1.35);
    let temperature_factor = (700.0 / inputs.temperature_k.max(1.0))
        .powf(0.35)
        .clamp(0.7, 1.5);
    let mixture_factor =
        equivalence_ratio_burn_duration_multiplier(inputs.equivalence_ratio, limits);
    let residual_factor = 1.0 + 1.5 * inputs.residual_fraction.clamp(0.0, 1.0);
    let rpm = inputs.crank_speed_rad_per_s * 60.0 / std::f64::consts::TAU;
    let speed_factor = (rpm.max(1.0) / 3000.0).powf(0.08).clamp(0.85, 1.2);
    let turbulence_factor = (1.0 / (1.0 + inputs.turbulence_intensity.max(0.0)))
        .powf(0.2)
        .clamp(0.65, 1.0);
    let geometry_factor = (inputs.chamber_characteristic_length_m.max(0.01) / 0.07)
        .powf(0.2)
        .clamp(0.75, 1.35);

    inputs.reference_duration_rad
        * pressure_factor
        * temperature_factor
        * mixture_factor
        * residual_factor
        * speed_factor
        * turbulence_factor
        * geometry_factor
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
    fn species_burn_conserves_total_mass_and_releases_lhv_heat() {
        let mut species = ChamberSpeciesMasses {
            oxygen_kg: 0.004,
            fuel_kg: 0.001,
            inert_kg: 0.010,
            products_kg: 0.0,
        };
        let initial_mass = species.total_mass_kg();

        let conversion = burn_species(&mut species, 0.0005, 14.7, 44.0e6, 1.0, 0.10);

        assert_approx_eq(species.total_mass_kg(), initial_mass);
        assert_approx_eq(conversion.consumed_fuel_kg, 0.0005);
        assert_approx_eq(
            conversion.consumed_oxygen_kg,
            0.0005 * stoichiometric_oxygen_fuel_ratio(14.7),
        );
        assert_approx_eq(conversion.released_heat_j, 0.0005 * 44.0e6 * 0.90);
    }

    #[test]
    fn species_burn_preserves_unburned_fuel_when_oxygen_limited() {
        let mut species = ChamberSpeciesMasses {
            oxygen_kg: 0.0001,
            fuel_kg: 0.001,
            inert_kg: 0.010,
            products_kg: 0.0,
        };

        let conversion = burn_species(&mut species, 0.001, 14.7, 44.0e6, 1.0, 0.0);

        assert_approx_eq(species.oxygen_kg, 0.0);
        assert!(species.fuel_kg > 0.0);
        assert_approx_eq(
            conversion.consumed_fuel_kg,
            0.0001 / stoichiometric_oxygen_fuel_ratio(14.7),
        );
    }

    #[test]
    fn species_burn_preserves_unused_oxygen_when_fuel_limited() {
        let mut species = ChamberSpeciesMasses {
            oxygen_kg: 0.004,
            fuel_kg: 0.0001,
            inert_kg: 0.010,
            products_kg: 0.0,
        };

        let conversion = burn_species(&mut species, 0.001, 14.7, 44.0e6, 1.0, 0.0);

        assert_approx_eq(conversion.consumed_fuel_kg, 0.0001);
        assert_approx_eq(species.fuel_kg, 0.0);
        assert!(species.oxygen_kg > 0.0);
        assert!(species.products_kg > 0.0);
    }

    #[test]
    fn partial_burn_leaves_meaningful_remaining_reactants() {
        let mut species = ChamberSpeciesMasses {
            oxygen_kg: 0.004,
            fuel_kg: 0.001,
            inert_kg: 0.010,
            products_kg: 0.0,
        };

        let conversion = burn_species(&mut species, 0.001, 14.7, 44.0e6, 0.5, 0.0);

        assert_approx_eq(conversion.consumed_fuel_kg, 0.0005);
        assert!(species.fuel_kg > 0.0);
        assert!(species.oxygen_kg > 0.0);
        assert!(species.products_kg > 0.0);
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
    fn rich_lean_and_misfire_cases_follow_mixture_limits() {
        let limits = default_mixture_limits();

        assert_eq!(
            mixture_combustion_efficiency(limits.minimum_combustible_air_fuel_ratio, limits),
            0.0
        );
        assert_eq!(
            mixture_combustion_efficiency(limits.maximum_combustible_air_fuel_ratio, limits),
            0.0
        );
        assert!(mixture_combustion_efficiency(10.0, limits) > 0.0);
        assert!(mixture_combustion_efficiency(20.0, limits) > 0.0);
    }

    #[test]
    fn fixed_volume_heat_release_raises_pressure_and_temperature() {
        let mut species = ChamberSpeciesMasses {
            oxygen_kg: 0.004,
            fuel_kg: 0.0002,
            inert_kg: 0.010,
            products_kg: 0.0,
        };
        let initial_species = species;
        let volume_m3 = 0.001;
        let temperature_k = 700.0;
        let fallback = crate::physics::chamber::ChamberProperties {
            gas_constant_j_per_kg_k: 287.0,
            specific_heat_ratio: 1.4,
            minimum_mass_kg: 1.0e-9,
            minimum_temperature_k: 1.0,
        };
        let initial_properties = crate::physics::chamber::mixture_chamber_properties(species, fallback);
        let initial_pressure = crate::physics::gas::pressure(
            species.total_mass_kg(),
            temperature_k,
            volume_m3,
            initial_properties.gas_constant_j_per_kg_k,
        );
        let initial_energy =
            crate::physics::chamber::species_internal_energy_j(species, temperature_k, fallback);

        let conversion = burn_species(&mut species, 0.0001, 14.7, 44.0e6, 1.0, 0.0);
        let updated_properties = crate::physics::chamber::mixture_chamber_properties(species, fallback);
        let updated_temperature = (initial_energy + conversion.released_heat_j)
            / (species.total_mass_kg()
                * crate::physics::gas::cv(
                    updated_properties.gas_constant_j_per_kg_k,
                    updated_properties.specific_heat_ratio,
                ));
        let updated_pressure = crate::physics::gas::pressure(
            species.total_mass_kg(),
            updated_temperature,
            volume_m3,
            updated_properties.gas_constant_j_per_kg_k,
        );

        assert_approx_eq(species.total_mass_kg(), initial_species.total_mass_kg());
        assert!(updated_temperature > temperature_k);
        assert!(updated_pressure > initial_pressure);
    }

    #[test]
    fn default_limits_match_design_phi_misfire_bounds() {
        let limits = default_mixture_limits();

        assert_approx_eq(
            equivalence_ratio_from_air_fuel_ratio(
                limits.minimum_combustible_air_fuel_ratio,
                limits.stoichiometric_air_fuel_ratio,
            ),
            1.96,
        );
        assert_approx_eq(
            equivalence_ratio_from_air_fuel_ratio(
                limits.maximum_combustible_air_fuel_ratio,
                limits.stoichiometric_air_fuel_ratio,
            ),
            0.563,
        );
        assert_approx_eq(
            mixture_combustion_efficiency(limits.minimum_combustible_air_fuel_ratio, limits),
            0.0,
        );
        assert_approx_eq(
            mixture_combustion_efficiency(limits.maximum_combustible_air_fuel_ratio, limits),
            0.0,
        );
    }

    #[test]
    fn equivalence_ratio_drives_delay_and_duration_separately() {
        let limits = default_mixture_limits();

        let fastest_duration = equivalence_ratio_burn_duration_multiplier(1.10, limits);
        let stoich_duration = equivalence_ratio_burn_duration_multiplier(1.0, limits);
        let lean_duration = equivalence_ratio_burn_duration_multiplier(0.8, limits);
        let fastest_delay = equivalence_ratio_ignition_delay_multiplier(1.05, limits);
        let lean_delay = equivalence_ratio_ignition_delay_multiplier(0.8, limits);

        assert!(fastest_duration < stoich_duration);
        assert!(lean_duration > stoich_duration);
        assert!(fastest_delay < lean_delay);
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

    #[test]
    fn ignition_delay_factors_move_in_expected_directions() {
        let limits = default_mixture_limits();
        let base = IgnitionDelayInputs {
            reference_delay_rad: 10.0_f64.to_radians(),
            pressure_pa: 101_325.0,
            temperature_k: 700.0,
            equivalence_ratio: 1.0,
            residual_fraction: 0.0,
            spark_energy_j: 0.04,
            turbulence_intensity: 0.0,
        };

        assert!(
            ignition_delay_rad(
                IgnitionDelayInputs {
                    pressure_pa: 200_000.0,
                    ..base
                },
                limits
            ) < ignition_delay_rad(base, limits)
        );
        assert!(
            ignition_delay_rad(
                IgnitionDelayInputs {
                    temperature_k: 900.0,
                    ..base
                },
                limits
            ) < ignition_delay_rad(base, limits)
        );
        assert!(
            ignition_delay_rad(
                IgnitionDelayInputs {
                    equivalence_ratio: 0.8,
                    ..base
                },
                limits
            ) > ignition_delay_rad(
                IgnitionDelayInputs {
                    equivalence_ratio: 1.05,
                    ..base
                },
                limits
            )
        );
        assert!(
            ignition_delay_rad(
                IgnitionDelayInputs {
                    residual_fraction: 0.3,
                    ..base
                },
                limits
            ) > ignition_delay_rad(base, limits)
        );
        assert!(
            ignition_delay_rad(
                IgnitionDelayInputs {
                    spark_energy_j: 0.08,
                    ..base
                },
                limits
            ) < ignition_delay_rad(base, limits)
        );
        assert!(
            ignition_delay_rad(
                IgnitionDelayInputs {
                    turbulence_intensity: 2.0,
                    ..base
                },
                limits
            ) < ignition_delay_rad(base, limits)
        );
    }

    #[test]
    fn burn_duration_factors_move_in_expected_directions() {
        let limits = default_mixture_limits();
        let base = BurnDurationInputs {
            reference_duration_rad: 40.0_f64.to_radians(),
            pressure_pa: 101_325.0,
            temperature_k: 700.0,
            equivalence_ratio: 1.0,
            residual_fraction: 0.0,
            crank_speed_rad_per_s: 3000.0 * std::f64::consts::TAU / 60.0,
            turbulence_intensity: 0.0,
            chamber_characteristic_length_m: 0.07,
        };

        assert!(
            burn_duration_rad(
                BurnDurationInputs {
                    crank_speed_rad_per_s: base.crank_speed_rad_per_s * 2.0,
                    ..base
                },
                limits
            ) > burn_duration_rad(base, limits)
        );
        assert!(
            burn_duration_rad(
                BurnDurationInputs {
                    pressure_pa: 200_000.0,
                    ..base
                },
                limits
            ) < burn_duration_rad(base, limits)
        );
        assert!(
            burn_duration_rad(
                BurnDurationInputs {
                    temperature_k: 900.0,
                    ..base
                },
                limits
            ) < burn_duration_rad(base, limits)
        );
        assert!(
            burn_duration_rad(
                BurnDurationInputs {
                    equivalence_ratio: 0.8,
                    ..base
                },
                limits
            ) > burn_duration_rad(
                BurnDurationInputs {
                    equivalence_ratio: 1.1,
                    ..base
                },
                limits
            )
        );
        assert!(
            burn_duration_rad(
                BurnDurationInputs {
                    residual_fraction: 0.3,
                    ..base
                },
                limits
            ) > burn_duration_rad(base, limits)
        );
        assert!(
            burn_duration_rad(
                BurnDurationInputs {
                    turbulence_intensity: 2.0,
                    ..base
                },
                limits
            ) < burn_duration_rad(base, limits)
        );
    }
}
