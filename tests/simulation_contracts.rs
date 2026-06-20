use enginesim::profiles::SimulationProfile;
use enginesim::sign_conventions::{
    POSITIVE_CRANK_ROTATION, POSITIVE_GAS_FORCE, POSITIVE_GAS_TORQUE, POSITIVE_PISTON_DISPLACEMENT,
    friction_torque_nm, gas_torque_from_force,
};
use enginesim::simulation::{ENGINE_STEP_ORDER, EngineStepStage, StepContext};

#[test]
fn profiles_are_inspectable_and_valid() {
    let real_time = SimulationProfile::real_time();
    let render = SimulationProfile::render();

    real_time.validate();
    render.validate();
    assert!(format!("{real_time:?}").contains("RealTime"));
    assert!(format!("{render:?}").contains("Render"));
}

#[test]
fn step_context_advances_by_fixed_dt() {
    let initial = StepContext::new(0.002);
    let next = initial.advanced();

    assert_eq!(next.step_index, 1);
    assert_eq!(next.simulation_time_seconds, 0.002);
    assert_eq!(next.timestep_seconds, initial.timestep_seconds);
}

#[test]
fn engine_step_order_is_stable_contract() {
    assert_eq!(ENGINE_STEP_ORDER[0], EngineStepStage::CrankKinematics);
    assert_eq!(
        ENGINE_STEP_ORDER[ENGINE_STEP_ORDER.len() - 1],
        EngineStepStage::Output
    );
    assert_eq!(ENGINE_STEP_ORDER.len(), 10);
}

#[test]
fn sign_convention_text_is_available_for_test_failures_and_docs() {
    assert!(POSITIVE_CRANK_ROTATION.contains("crank angle"));
    assert!(POSITIVE_PISTON_DISPLACEMENT.contains("top dead center"));
    assert!(POSITIVE_GAS_FORCE.contains("piston"));
    assert!(POSITIVE_GAS_TORQUE.contains("crank speed"));
}

#[test]
fn force_torque_and_friction_signs_are_consistent() {
    assert!(gas_torque_from_force(1_000.0, 0.01) > 0.0);
    assert!(gas_torque_from_force(1_000.0, -0.01) < 0.0);
    assert!(friction_torque_nm(100.0, 5.0) < 0.0);
    assert!(friction_torque_nm(-100.0, 5.0) > 0.0);
}
