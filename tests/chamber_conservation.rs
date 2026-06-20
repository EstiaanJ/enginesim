mod common;

use common::{
    EPSILON, air_properties, assert_approx_eq, chamber_state_at_air_pressure, closed_flow,
    fixed_volume_boundary,
};
use enginesim::chamber::{
    ChamberBoundary, ChamberRk4StageInputs, FlowBoundary, chamber_derivatives, chamber_pressure_pa,
    step_euler, step_rk4, step_rk4_with_stage_inputs,
};
use enginesim::gas::{cp, cv};

#[test]
fn fixed_volume_heat_addition_increases_temperature_by_energy_over_mass_cv() {
    let properties = air_properties();
    let state = chamber_state_at_air_pressure(100_000.0, 300.0, 0.001);
    let boundary = ChamberBoundary {
        volume_m3: 0.001,
        volume_rate_m3_per_s: 0.0,
        heat_rate_w: 100.0,
    };
    let timestep_seconds = 0.001;

    let updated = step_euler(
        state,
        boundary,
        closed_flow(),
        closed_flow(),
        properties,
        timestep_seconds,
    );
    let expected_temperature = state.temperature_k
        + boundary.heat_rate_w * timestep_seconds
            / (state.mass_kg
                * cv(
                    properties.gas_constant_j_per_kg_k,
                    properties.specific_heat_ratio,
                ));

    assert_approx_eq(updated.mass_kg, state.mass_kg, EPSILON);
    assert_approx_eq(updated.temperature_k, expected_temperature, 1.0e-9);
}

#[test]
fn closed_expanding_chamber_cools_and_drops_pressure() {
    let properties = air_properties();
    let state = chamber_state_at_air_pressure(100_000.0, 300.0, 0.001);
    let boundary = ChamberBoundary {
        volume_m3: 0.001,
        volume_rate_m3_per_s: 0.0001,
        heat_rate_w: 0.0,
    };
    let start_pressure = chamber_pressure_pa(state, boundary, properties);

    let updated = step_euler(
        state,
        boundary,
        closed_flow(),
        closed_flow(),
        properties,
        0.001,
    );
    let end_pressure = chamber_pressure_pa(
        updated,
        ChamberBoundary {
            volume_m3: boundary.volume_m3 + boundary.volume_rate_m3_per_s * 0.001,
            ..boundary
        },
        properties,
    );

    assert_approx_eq(updated.mass_kg, state.mass_kg, EPSILON);
    assert!(updated.temperature_k < state.temperature_k);
    assert!(end_pressure < start_pressure);
}

#[test]
fn closed_moving_volume_derivative_matches_pressure_work_rate() {
    let properties = air_properties();
    let state = chamber_state_at_air_pressure(100_000.0, 300.0, 0.001);
    let boundary = ChamberBoundary {
        volume_m3: 0.001,
        volume_rate_m3_per_s: 0.0001,
        heat_rate_w: 0.0,
    };
    let derivatives =
        chamber_derivatives(state, boundary, closed_flow(), closed_flow(), properties);
    let chamber_cv = cv(
        properties.gas_constant_j_per_kg_k,
        properties.specific_heat_ratio,
    );
    let internal_energy_rate_w = state.mass_kg * chamber_cv * derivatives.temperature_rate_k_per_s;
    let pressure_pa = chamber_pressure_pa(state, boundary, properties);

    assert_approx_eq(
        internal_energy_rate_w,
        -pressure_pa * boundary.volume_rate_m3_per_s,
        1.0e-9,
    );
}

#[test]
fn open_chamber_filling_uses_inlet_enthalpy_in_energy_budget() {
    let properties = air_properties();
    let state = chamber_state_at_air_pressure(90_000.0, 300.0, 0.002);
    let boundary = fixed_volume_boundary(0.002);
    let inlet = FlowBoundary {
        upstream_pressure_pa: 120_000.0,
        upstream_temperature_k: 320.0,
        downstream_pressure_pa: 0.0,
        discharge_coefficient: 0.7,
        area_m2: 0.0001,
    };

    let derivatives = chamber_derivatives(state, boundary, inlet, closed_flow(), properties);
    let chamber_cv = cv(
        properties.gas_constant_j_per_kg_k,
        properties.specific_heat_ratio,
    );
    let inlet_cp = cp(
        properties.gas_constant_j_per_kg_k,
        properties.specific_heat_ratio,
    );
    let internal_energy_rate = state.mass_kg * chamber_cv * derivatives.temperature_rate_k_per_s
        + derivatives.mass_rate_kg_per_s * chamber_cv * state.temperature_k;
    let expected_energy_rate =
        derivatives.inlet_mass_rate_kg_per_s * inlet_cp * inlet.upstream_temperature_k;

    assert!(derivatives.inlet_mass_rate_kg_per_s > 0.0);
    assert_approx_eq(internal_energy_rate, expected_energy_rate, 1.0e-6);
}

#[test]
fn open_chamber_blowdown_removes_mass() {
    let properties = air_properties();
    let state = chamber_state_at_air_pressure(150_000.0, 500.0, 0.002);
    let boundary = fixed_volume_boundary(0.002);
    let outlet = FlowBoundary {
        upstream_pressure_pa: 0.0,
        upstream_temperature_k: 300.0,
        downstream_pressure_pa: 100_000.0,
        discharge_coefficient: 0.7,
        area_m2: 0.0001,
    };

    let derivatives = chamber_derivatives(state, boundary, closed_flow(), outlet, properties);

    assert!(derivatives.outlet_mass_rate_kg_per_s > 0.0);
    assert!(derivatives.mass_rate_kg_per_s < 0.0);
}

#[test]
fn rk4_stage_inputs_evaluate_moving_boundary_at_substeps() {
    let properties = air_properties();
    let state = chamber_state_at_air_pressure(100_000.0, 300.0, 0.001);
    let timestep_seconds = 0.01;
    let volume_rate_m3_per_s = 0.0001;
    let fixed_boundary = ChamberBoundary {
        volume_m3: 0.001,
        volume_rate_m3_per_s,
        heat_rate_w: 0.0,
    };
    let moving_boundary = |elapsed_seconds: f64| ChamberBoundary {
        volume_m3: fixed_boundary.volume_m3 + volume_rate_m3_per_s * elapsed_seconds,
        volume_rate_m3_per_s,
        heat_rate_w: 0.0,
    };

    let fixed = step_rk4(
        state,
        fixed_boundary,
        closed_flow(),
        closed_flow(),
        properties,
        timestep_seconds,
    );
    let moving = step_rk4_with_stage_inputs(
        state,
        |stage| ChamberRk4StageInputs {
            boundary: moving_boundary(stage.elapsed_seconds),
            inlet: closed_flow(),
            outlet: closed_flow(),
        },
        properties,
        timestep_seconds,
    );

    assert!(moving.temperature_k > fixed.temperature_k);
    assert_approx_eq(moving.mass_kg, state.mass_kg, EPSILON);
}
