use crate::physics::flow::isentropic_mass_flow;
use crate::physics::gas::{cp, cv, pressure, temperature_from_internal_energy_k};

pub const DRY_AIR_OXYGEN_MASS_FRACTION: f64 = 0.232;
pub const DRY_AIR_INERT_MASS_FRACTION: f64 = 1.0 - DRY_AIR_OXYGEN_MASS_FRACTION;

const OXYGEN_GAS_CONSTANT_J_PER_KG_K: f64 = 259.84;
const OXYGEN_CV_J_PER_KG_K: f64 = 659.0;
const INERT_GAS_CONSTANT_J_PER_KG_K: f64 = 296.8;
const INERT_CV_J_PER_KG_K: f64 = 743.0;
const PRODUCTS_GAS_CONSTANT_J_PER_KG_K: f64 = 240.0;
const PRODUCTS_CV_J_PER_KG_K: f64 = 820.0;
const FUEL_VAPOR_GAS_CONSTANT_J_PER_KG_K: f64 = 72.0;
const FUEL_VAPOR_CV_J_PER_KG_K: f64 = 1700.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChamberState {
    pub mass_kg: f64,
    pub temperature_k: f64,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ChamberSpeciesMasses {
    pub oxygen_kg: f64,
    pub fuel_kg: f64,
    pub inert_kg: f64,
    pub products_kg: f64,
}

impl ChamberSpeciesMasses {
    pub fn from_dry_air_mass(air_mass_kg: f64) -> Self {
        let air_mass_kg = air_mass_kg.max(0.0);
        Self {
            oxygen_kg: air_mass_kg * DRY_AIR_OXYGEN_MASS_FRACTION,
            fuel_kg: 0.0,
            inert_kg: air_mass_kg * DRY_AIR_INERT_MASS_FRACTION,
            products_kg: 0.0,
        }
    }

    pub fn total_mass_kg(self) -> f64 {
        self.oxygen_kg.max(0.0)
            + self.fuel_kg.max(0.0)
            + self.inert_kg.max(0.0)
            + self.products_kg.max(0.0)
    }

    pub fn air_mass_kg(self) -> f64 {
        self.oxygen_kg.max(0.0) + self.inert_kg.max(0.0)
    }

    /// Pre-combustion ("charge") air and fuel masses in kg, reconstructed by
    /// splitting `products` back into the fuel and oxygen that formed them.
    ///
    /// Combustion drains `fuel` and `oxygen` into `products` while leaving
    /// `inert` untouched, so the raw `air_mass_kg() / fuel_kg` ratio of a
    /// partially or fully burned charge no longer reflects the mixture that
    /// ignited. Because products are formed from fuel + oxygen in
    /// stoichiometric proportion (see `burn_species`), undoing that split
    /// recovers the original air/fuel split and makes the result invariant
    /// through combustion — what a wideband O2 sensor effectively measures.
    ///
    /// Returns `(air_kg, fuel_kg)`.
    pub fn charge_air_and_fuel_kg(self, stoichiometric_air_fuel_ratio: f64) -> (f64, f64) {
        let species = self.normalized();
        let oxygen_fuel_ratio =
            stoichiometric_air_fuel_ratio.max(0.0) * DRY_AIR_OXYGEN_MASS_FRACTION;
        let fuel_in_products = species.products_kg / (1.0 + oxygen_fuel_ratio);
        let oxygen_in_products = species.products_kg - fuel_in_products;

        let air_kg = species.oxygen_kg + oxygen_in_products + species.inert_kg;
        let fuel_kg = species.fuel_kg + fuel_in_products;
        (air_kg, fuel_kg)
    }

    pub fn lambda(self, stoichiometric_air_fuel_ratio: f64) -> Option<f64> {
        if stoichiometric_air_fuel_ratio <= 0.0 {
            return None;
        }

        let (air_kg, fuel_kg) = self.charge_air_and_fuel_kg(stoichiometric_air_fuel_ratio);
        if fuel_kg <= 0.0 {
            // No fuel has ever been present in this gas: there is no air/fuel
            // ratio to report.
            return None;
        }

        Some(air_kg / fuel_kg / stoichiometric_air_fuel_ratio)
    }

    pub fn normalized(self) -> Self {
        Self {
            oxygen_kg: self.oxygen_kg.max(0.0),
            fuel_kg: self.fuel_kg.max(0.0),
            inert_kg: self.inert_kg.max(0.0),
            products_kg: self.products_kg.max(0.0),
        }
    }

    pub fn add_dry_air(&mut self, air_mass_kg: f64) {
        if air_mass_kg <= 0.0 {
            return;
        }

        self.oxygen_kg += air_mass_kg * DRY_AIR_OXYGEN_MASS_FRACTION;
        self.inert_kg += air_mass_kg * DRY_AIR_INERT_MASS_FRACTION;
    }

    pub fn add_products(&mut self, products_mass_kg: f64) {
        self.products_kg += products_mass_kg.max(0.0);
    }

    pub fn remove_proportional(&mut self, mass_kg: f64) {
        let total_mass_kg = self.total_mass_kg();
        if mass_kg <= 0.0 || total_mass_kg <= 0.0 {
            return;
        }

        let removed_fraction = (mass_kg / total_mass_kg).clamp(0.0, 1.0);
        self.oxygen_kg *= 1.0 - removed_fraction;
        self.fuel_kg *= 1.0 - removed_fraction;
        self.inert_kg *= 1.0 - removed_fraction;
        self.products_kg *= 1.0 - removed_fraction;
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChamberBoundary {
    pub volume_m3: f64,
    pub volume_rate_m3_per_s: f64,
    pub heat_rate_w: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlowBoundary {
    pub upstream_pressure_pa: f64,
    pub upstream_temperature_k: f64,
    pub downstream_pressure_pa: f64,
    pub discharge_coefficient: f64,
    pub area_m2: f64,
}

impl FlowBoundary {
    pub fn closed() -> Self {
        Self {
            upstream_pressure_pa: 0.0,
            upstream_temperature_k: 0.0,
            downstream_pressure_pa: 0.0,
            discharge_coefficient: 0.0,
            area_m2: 0.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChamberProperties {
    pub gas_constant_j_per_kg_k: f64,
    pub specific_heat_ratio: f64,
    pub minimum_mass_kg: f64,
    pub minimum_temperature_k: f64,
}

pub fn mixture_chamber_properties(
    species: ChamberSpeciesMasses,
    fallback: ChamberProperties,
) -> ChamberProperties {
    let species = species.normalized();
    let total_mass_kg = species.total_mass_kg();
    if total_mass_kg <= fallback.minimum_mass_kg.max(0.0) {
        return fallback;
    }

    // Initial Phase 3 approximation: fixed per-species ideal-gas constants and cv values,
    // mixed by mass fraction. This keeps the conserved-state API replaceable later.
    let gas_constant_j_per_kg_k = (species.oxygen_kg * OXYGEN_GAS_CONSTANT_J_PER_KG_K
        + species.fuel_kg * FUEL_VAPOR_GAS_CONSTANT_J_PER_KG_K
        + species.inert_kg * INERT_GAS_CONSTANT_J_PER_KG_K
        + species.products_kg * PRODUCTS_GAS_CONSTANT_J_PER_KG_K)
        / total_mass_kg;
    let cv_j_per_kg_k = (species.oxygen_kg * OXYGEN_CV_J_PER_KG_K
        + species.fuel_kg * FUEL_VAPOR_CV_J_PER_KG_K
        + species.inert_kg * INERT_CV_J_PER_KG_K
        + species.products_kg * PRODUCTS_CV_J_PER_KG_K)
        / total_mass_kg;
    let specific_heat_ratio = if cv_j_per_kg_k > 0.0 {
        1.0 + gas_constant_j_per_kg_k / cv_j_per_kg_k
    } else {
        fallback.specific_heat_ratio
    };

    ChamberProperties {
        gas_constant_j_per_kg_k,
        specific_heat_ratio,
        minimum_mass_kg: fallback.minimum_mass_kg,
        minimum_temperature_k: fallback.minimum_temperature_k,
    }
}

pub fn species_internal_energy_j(
    species: ChamberSpeciesMasses,
    temperature_k: f64,
    fallback: ChamberProperties,
) -> f64 {
    let species = species.normalized();
    let total_mass_kg = species.total_mass_kg();
    if total_mass_kg <= fallback.minimum_mass_kg.max(0.0) {
        return total_mass_kg
            * cv(
                fallback.gas_constant_j_per_kg_k,
                fallback.specific_heat_ratio,
            )
            * temperature_k.max(fallback.minimum_temperature_k);
    }

    let cv_mass_sum_j_per_k = species.oxygen_kg * OXYGEN_CV_J_PER_KG_K
        + species.fuel_kg * FUEL_VAPOR_CV_J_PER_KG_K
        + species.inert_kg * INERT_CV_J_PER_KG_K
        + species.products_kg * PRODUCTS_CV_J_PER_KG_K;

    cv_mass_sum_j_per_k * temperature_k.max(fallback.minimum_temperature_k)
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ChamberDerivatives {
    pub mass_rate_kg_per_s: f64,
    pub temperature_rate_k_per_s: f64,
    pub pressure_rate_pa_per_s: f64,
    pub inlet_mass_rate_kg_per_s: f64,
    pub outlet_mass_rate_kg_per_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChamberRk4Stage {
    pub elapsed_seconds: f64,
    pub state: ChamberState,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChamberRk4StageInputs {
    pub boundary: ChamberBoundary,
    pub inlet: FlowBoundary,
    pub outlet: FlowBoundary,
}

pub fn chamber_derivatives(
    state: ChamberState,
    boundary: ChamberBoundary,
    inlet: FlowBoundary,
    outlet: FlowBoundary,
    properties: ChamberProperties,
) -> ChamberDerivatives {
    if state.mass_kg <= 0.0
        || state.temperature_k <= 0.0
        || boundary.volume_m3 <= 0.0
        || properties.gas_constant_j_per_kg_k <= 0.0
        || properties.specific_heat_ratio <= 1.0
    {
        return ChamberDerivatives::default();
    }

    let chamber_pressure_pa = pressure(
        state.mass_kg,
        state.temperature_k,
        boundary.volume_m3,
        properties.gas_constant_j_per_kg_k,
    );
    let (inlet_mass_rate_kg_per_s, inlet_source_temperature_k) =
        bidirectional_inlet_mass_rate_kg_per_s(state, chamber_pressure_pa, inlet, properties);
    let (outlet_mass_rate_kg_per_s, outlet_source_temperature_k) =
        bidirectional_outlet_mass_rate_kg_per_s(state, chamber_pressure_pa, outlet, properties);
    let mass_rate_kg_per_s = inlet_mass_rate_kg_per_s - outlet_mass_rate_kg_per_s;
    let cv = cv(
        properties.gas_constant_j_per_kg_k,
        properties.specific_heat_ratio,
    );
    let cp = cp(
        properties.gas_constant_j_per_kg_k,
        properties.specific_heat_ratio,
    );
    let inlet_energy_rate_w = inlet_mass_rate_kg_per_s * cp * inlet_source_temperature_k;
    let outlet_energy_rate_w = outlet_mass_rate_kg_per_s * cp * outlet_source_temperature_k;
    let piston_work_rate_w = chamber_pressure_pa * boundary.volume_rate_m3_per_s;
    let internal_energy_rate_w =
        inlet_energy_rate_w - outlet_energy_rate_w + boundary.heat_rate_w - piston_work_rate_w;
    let temperature_rate_k_per_s = (internal_energy_rate_w
        - mass_rate_kg_per_s * cv * state.temperature_k)
        / (state.mass_kg * cv);
    let pressure_rate_pa_per_s = properties.gas_constant_j_per_kg_k / boundary.volume_m3
        * (state.temperature_k * mass_rate_kg_per_s + state.mass_kg * temperature_rate_k_per_s
            - state.mass_kg * state.temperature_k * boundary.volume_rate_m3_per_s
                / boundary.volume_m3);

    ChamberDerivatives {
        mass_rate_kg_per_s,
        temperature_rate_k_per_s,
        pressure_rate_pa_per_s,
        inlet_mass_rate_kg_per_s,
        outlet_mass_rate_kg_per_s,
    }
}

fn bidirectional_inlet_mass_rate_kg_per_s(
    state: ChamberState,
    chamber_pressure_pa: f64,
    inlet: FlowBoundary,
    properties: ChamberProperties,
) -> (f64, f64) {
    if inlet.upstream_pressure_pa >= chamber_pressure_pa {
        (
            isentropic_mass_flow(
                inlet.upstream_pressure_pa,
                inlet.upstream_temperature_k,
                chamber_pressure_pa,
                inlet.discharge_coefficient,
                inlet.area_m2,
                properties.specific_heat_ratio,
                properties.gas_constant_j_per_kg_k,
            ),
            inlet.upstream_temperature_k.max(0.0),
        )
    } else {
        (
            -isentropic_mass_flow(
                chamber_pressure_pa,
                state.temperature_k,
                inlet.upstream_pressure_pa,
                inlet.discharge_coefficient,
                inlet.area_m2,
                properties.specific_heat_ratio,
                properties.gas_constant_j_per_kg_k,
            ),
            state.temperature_k,
        )
    }
}

fn bidirectional_outlet_mass_rate_kg_per_s(
    state: ChamberState,
    chamber_pressure_pa: f64,
    outlet: FlowBoundary,
    properties: ChamberProperties,
) -> (f64, f64) {
    if chamber_pressure_pa >= outlet.downstream_pressure_pa {
        (
            isentropic_mass_flow(
                chamber_pressure_pa,
                state.temperature_k,
                outlet.downstream_pressure_pa,
                outlet.discharge_coefficient,
                outlet.area_m2,
                properties.specific_heat_ratio,
                properties.gas_constant_j_per_kg_k,
            ),
            state.temperature_k,
        )
    } else {
        (
            -isentropic_mass_flow(
                outlet.downstream_pressure_pa,
                outlet.upstream_temperature_k,
                chamber_pressure_pa,
                outlet.discharge_coefficient,
                outlet.area_m2,
                properties.specific_heat_ratio,
                properties.gas_constant_j_per_kg_k,
            ),
            outlet.upstream_temperature_k.max(0.0),
        )
    }
}

pub fn step_euler(
    state: ChamberState,
    boundary: ChamberBoundary,
    inlet: FlowBoundary,
    outlet: FlowBoundary,
    properties: ChamberProperties,
    timestep_seconds: f64,
) -> ChamberState {
    assert!(timestep_seconds >= 0.0, "timestep must be non-negative");

    let derivatives = chamber_derivatives(state, boundary, inlet, outlet, properties);
    clamp_state(
        ChamberState {
            mass_kg: state.mass_kg + derivatives.mass_rate_kg_per_s * timestep_seconds,
            temperature_k: state.temperature_k
                + derivatives.temperature_rate_k_per_s * timestep_seconds,
        },
        properties,
    )
}

pub fn step_rk4(
    state: ChamberState,
    boundary: ChamberBoundary,
    inlet: FlowBoundary,
    outlet: FlowBoundary,
    properties: ChamberProperties,
    timestep_seconds: f64,
) -> ChamberState {
    assert!(timestep_seconds >= 0.0, "timestep must be non-negative");

    step_rk4_with_stage_inputs(
        state,
        |_| ChamberRk4StageInputs {
            boundary,
            inlet,
            outlet,
        },
        properties,
        timestep_seconds,
    )
}

pub fn step_rk4_with_stage_inputs(
    state: ChamberState,
    stage_inputs: impl Fn(ChamberRk4Stage) -> ChamberRk4StageInputs,
    properties: ChamberProperties,
    timestep_seconds: f64,
) -> ChamberState {
    assert!(timestep_seconds >= 0.0, "timestep must be non-negative");

    let inputs_1 = stage_inputs(ChamberRk4Stage {
        elapsed_seconds: 0.0,
        state,
    });
    let k1 = chamber_derivatives(
        state,
        inputs_1.boundary,
        inputs_1.inlet,
        inputs_1.outlet,
        properties,
    );
    let k2_state = derivative_state(state, k1, timestep_seconds * 0.5, properties);
    let inputs_2 = stage_inputs(ChamberRk4Stage {
        elapsed_seconds: timestep_seconds * 0.5,
        state: k2_state,
    });
    let k2 = chamber_derivatives(
        k2_state,
        inputs_2.boundary,
        inputs_2.inlet,
        inputs_2.outlet,
        properties,
    );
    let k3_state = derivative_state(state, k2, timestep_seconds * 0.5, properties);
    let inputs_3 = stage_inputs(ChamberRk4Stage {
        elapsed_seconds: timestep_seconds * 0.5,
        state: k3_state,
    });
    let k3 = chamber_derivatives(
        k3_state,
        inputs_3.boundary,
        inputs_3.inlet,
        inputs_3.outlet,
        properties,
    );
    let k4_state = derivative_state(state, k3, timestep_seconds, properties);
    let inputs_4 = stage_inputs(ChamberRk4Stage {
        elapsed_seconds: timestep_seconds,
        state: k4_state,
    });
    let k4 = chamber_derivatives(
        k4_state,
        inputs_4.boundary,
        inputs_4.inlet,
        inputs_4.outlet,
        properties,
    );

    clamp_state(
        ChamberState {
            mass_kg: state.mass_kg
                + timestep_seconds
                    * (k1.mass_rate_kg_per_s
                        + 2.0 * k2.mass_rate_kg_per_s
                        + 2.0 * k3.mass_rate_kg_per_s
                        + k4.mass_rate_kg_per_s)
                    / 6.0,
            temperature_k: state.temperature_k
                + timestep_seconds
                    * (k1.temperature_rate_k_per_s
                        + 2.0 * k2.temperature_rate_k_per_s
                        + 2.0 * k3.temperature_rate_k_per_s
                        + k4.temperature_rate_k_per_s)
                    / 6.0,
        },
        properties,
    )
}

pub fn chamber_pressure_pa(
    state: ChamberState,
    boundary: ChamberBoundary,
    properties: ChamberProperties,
) -> f64 {
    pressure(
        state.mass_kg,
        state.temperature_k,
        boundary.volume_m3,
        properties.gas_constant_j_per_kg_k,
    )
}

pub fn state_from_internal_energy(
    mass_kg: f64,
    internal_energy_j: f64,
    properties: ChamberProperties,
) -> ChamberState {
    let cv = cv(
        properties.gas_constant_j_per_kg_k,
        properties.specific_heat_ratio,
    );
    clamp_state(
        ChamberState {
            mass_kg,
            temperature_k: temperature_from_internal_energy_k(internal_energy_j, mass_kg, cv),
        },
        properties,
    )
}

fn derivative_state(
    state: ChamberState,
    derivatives: ChamberDerivatives,
    timestep_seconds: f64,
    properties: ChamberProperties,
) -> ChamberState {
    clamp_state(
        ChamberState {
            mass_kg: state.mass_kg + derivatives.mass_rate_kg_per_s * timestep_seconds,
            temperature_k: state.temperature_k
                + derivatives.temperature_rate_k_per_s * timestep_seconds,
        },
        properties,
    )
}

fn clamp_state(state: ChamberState, properties: ChamberProperties) -> ChamberState {
    ChamberState {
        mass_kg: state.mass_kg.max(properties.minimum_mass_kg.max(0.0)),
        temperature_k: state
            .temperature_k
            .max(properties.minimum_temperature_k.max(0.0)),
    }
}
