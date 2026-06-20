#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StepContext {
    pub timestep_seconds: f64,
    pub step_index: u64,
    pub simulation_time_seconds: f64,
}

impl StepContext {
    pub fn new(timestep_seconds: f64) -> Self {
        assert!(timestep_seconds > 0.0, "timestep must be positive");

        Self {
            timestep_seconds,
            step_index: 0,
            simulation_time_seconds: 0.0,
        }
    }

    pub fn advanced(self) -> Self {
        Self {
            timestep_seconds: self.timestep_seconds,
            step_index: self.step_index + 1,
            simulation_time_seconds: self.simulation_time_seconds + self.timestep_seconds,
        }
    }
}

pub trait StepModel {
    type Inputs;
    type Outputs;

    fn step(&mut self, context: StepContext, inputs: Self::Inputs) -> Self::Outputs;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EngineStepStage {
    CrankKinematics,
    Geometry,
    ValveState,
    BoundaryFlow,
    CombustionHeatSource,
    ChamberState,
    PressureForce,
    CrankTorque,
    CrankIntegration,
    Output,
}

pub const ENGINE_STEP_ORDER: [EngineStepStage; 10] = [
    EngineStepStage::CrankKinematics,
    EngineStepStage::Geometry,
    EngineStepStage::ValveState,
    EngineStepStage::BoundaryFlow,
    EngineStepStage::CombustionHeatSource,
    EngineStepStage::ChamberState,
    EngineStepStage::PressureForce,
    EngineStepStage::CrankTorque,
    EngineStepStage::CrankIntegration,
    EngineStepStage::Output,
];

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, Default)]
    struct Accumulator {
        value: f64,
    }

    impl StepModel for Accumulator {
        type Inputs = f64;
        type Outputs = f64;

        fn step(&mut self, context: StepContext, input: Self::Inputs) -> Self::Outputs {
            self.value += input * context.timestep_seconds;
            self.value
        }
    }

    #[test]
    fn context_advances_deterministically() {
        let context = StepContext::new(0.001).advanced().advanced();

        assert_eq!(context.step_index, 2);
        assert_eq!(context.simulation_time_seconds, 0.002);
    }

    #[test]
    fn step_model_repeats_for_same_inputs() {
        let mut first = Accumulator::default();
        let mut second = Accumulator::default();
        let context = StepContext::new(0.01);

        assert_eq!(first.step(context, 2.0), second.step(context, 2.0));
        assert_eq!(
            first.step(context.advanced(), 4.0),
            second.step(context.advanced(), 4.0)
        );
    }

    #[test]
    fn engine_step_order_matches_design_order() {
        assert_eq!(
            ENGINE_STEP_ORDER,
            [
                EngineStepStage::CrankKinematics,
                EngineStepStage::Geometry,
                EngineStepStage::ValveState,
                EngineStepStage::BoundaryFlow,
                EngineStepStage::CombustionHeatSource,
                EngineStepStage::ChamberState,
                EngineStepStage::PressureForce,
                EngineStepStage::CrankTorque,
                EngineStepStage::CrankIntegration,
                EngineStepStage::Output,
            ]
        );
    }
}
