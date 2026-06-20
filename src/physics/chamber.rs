use crate::flow::isentropic_mass_flow;
use crate::gas::{cp, cv, pressure, temperature_from_internal_energy_k};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChamberState {
    pub mass_kg: f64,
    pub temperature_k: f64,
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

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ChamberDerivatives {
    pub mass_rate_kg_per_s: f64,
    pub temperature_rate_k_per_s: f64,
    pub pressure_rate_pa_per_s: f64,
    pub inlet_mass_rate_kg_per_s: f64,
    pub outlet_mass_rate_kg_per_s: f64,
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

    let k1 = chamber_derivatives(state, boundary, inlet, outlet, properties);
    let k2_state = derivative_state(state, k1, timestep_seconds * 0.5, properties);
    let k2 = chamber_derivatives(k2_state, boundary, inlet, outlet, properties);
    let k3_state = derivative_state(state, k2, timestep_seconds * 0.5, properties);
    let k3 = chamber_derivatives(k3_state, boundary, inlet, outlet, properties);
    let k4_state = derivative_state(state, k3, timestep_seconds, properties);
    let k4 = chamber_derivatives(k4_state, boundary, inlet, outlet, properties);

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
