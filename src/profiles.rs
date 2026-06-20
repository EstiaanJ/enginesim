#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimulationProfileKind {
    RealTime,
    Render,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimulationProfile {
    pub kind: SimulationProfileKind,
    pub timestep_seconds: f64,
    pub chamber_substeps: u32,
    pub one_d_mesh_cells_per_meter: f64,
    pub output_sample_rate_hz: f64,
}

impl SimulationProfile {
    pub fn real_time() -> Self {
        Self {
            kind: SimulationProfileKind::RealTime,
            timestep_seconds: 1.0 / 10_000.0,
            chamber_substeps: 1,
            one_d_mesh_cells_per_meter: 20.0,
            output_sample_rate_hz: 48_000.0,
        }
    }

    pub fn render() -> Self {
        Self {
            kind: SimulationProfileKind::Render,
            timestep_seconds: 1.0 / 60_000.0,
            chamber_substeps: 4,
            one_d_mesh_cells_per_meter: 100.0,
            output_sample_rate_hz: 96_000.0,
        }
    }

    pub fn validate(self) {
        assert!(self.timestep_seconds > 0.0, "timestep must be positive");
        assert!(
            self.chamber_substeps > 0,
            "chamber substeps must be positive"
        );
        assert!(
            self.one_d_mesh_cells_per_meter > 0.0,
            "1D mesh density must be positive"
        );
        assert!(
            self.output_sample_rate_hz > 0.0,
            "output sample rate must be positive"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn real_time_and_render_profiles_share_same_settings_shape() {
        let real_time = SimulationProfile::real_time();
        let render = SimulationProfile::render();

        real_time.validate();
        render.validate();
        assert_eq!(real_time.kind, SimulationProfileKind::RealTime);
        assert_eq!(render.kind, SimulationProfileKind::Render);
    }

    #[test]
    fn render_profile_uses_finer_default_resolution_than_real_time() {
        let real_time = SimulationProfile::real_time();
        let render = SimulationProfile::render();

        assert!(render.timestep_seconds < real_time.timestep_seconds);
        assert!(render.chamber_substeps > real_time.chamber_substeps);
        assert!(render.one_d_mesh_cells_per_meter > real_time.one_d_mesh_cells_per_meter);
        assert!(render.output_sample_rate_hz >= real_time.output_sample_rate_hz);
    }
}
