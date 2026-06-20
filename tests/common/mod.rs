#![allow(dead_code)]

use enginesim::chamber::{ChamberBoundary, ChamberProperties, ChamberState, FlowBoundary};
use enginesim::consts::{GAMMA, R};

pub const EPSILON: f64 = 1.0e-9;

pub fn assert_approx_eq(actual: f64, expected: f64, tolerance: f64) {
    let difference = (actual - expected).abs();
    assert!(
        difference <= tolerance,
        "expected {expected}, got {actual}; difference {difference} exceeded {tolerance}",
    );
}

pub fn air_properties() -> ChamberProperties {
    ChamberProperties {
        gas_constant_j_per_kg_k: R,
        specific_heat_ratio: GAMMA,
        minimum_mass_kg: 1.0e-9,
        minimum_temperature_k: 1.0,
    }
}

pub fn chamber_state_at_air_pressure(
    pressure_pa: f64,
    temperature_k: f64,
    volume_m3: f64,
) -> ChamberState {
    ChamberState {
        mass_kg: pressure_pa * volume_m3 / (R * temperature_k),
        temperature_k,
    }
}

pub fn fixed_volume_boundary(volume_m3: f64) -> ChamberBoundary {
    ChamberBoundary {
        volume_m3,
        volume_rate_m3_per_s: 0.0,
        heat_rate_w: 0.0,
    }
}

pub fn closed_flow() -> FlowBoundary {
    FlowBoundary::closed()
}
