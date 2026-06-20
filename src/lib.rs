pub mod physics {
    //! Low-level physics primitives. The submodule files live in `src/physics/`,
    //! so this mirrors the on-disk layout for tooling and `use` paths.
    pub mod chamber;
    pub mod consts;
    pub mod flow;
    pub mod gas;
    pub mod linear_mass_integrator;
    pub mod mechanical_linking;
    pub mod pipe;
    pub mod rotational_mass_integrator;
}

pub mod combustion;
pub mod engine_config;
pub mod engine_geometry;
pub mod engine_handling;
pub mod engine_loader;
pub mod multi_cylinder;
pub mod profiles;
pub mod sign_conventions;
pub mod simulation;
pub mod single_cylinder;
pub mod telemetry;
pub mod test_harness;
pub mod throttle;
pub mod tuning;
pub mod validation;
pub mod valve;
