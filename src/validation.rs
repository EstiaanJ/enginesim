#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tolerance {
    pub absolute: f64,
    pub relative: f64,
}

impl Tolerance {
    pub const fn absolute(absolute: f64) -> Self {
        Self {
            absolute,
            relative: 0.0,
        }
    }

    pub const fn relative(relative: f64) -> Self {
        Self {
            absolute: 0.0,
            relative,
        }
    }

    pub const fn combined(absolute: f64, relative: f64) -> Self {
        Self { absolute, relative }
    }

    pub fn allowed_difference(self, expected: f64) -> f64 {
        self.absolute.max(expected.abs() * self.relative)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ScalarBaseline {
    pub name: String,
    pub expected: f64,
    pub tolerance: Tolerance,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BaselineMetadata {
    pub simulator_id: String,
    pub profile: String,
    pub engine_configuration: String,
    pub environment: String,
    pub timestep_seconds: f64,
    pub substeps: u32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct RegressionBaseline {
    pub metadata: BaselineMetadata,
    pub scalars: Vec<ScalarBaseline>,
}

impl RegressionBaseline {
    pub fn new(metadata: BaselineMetadata, scalars: Vec<ScalarBaseline>) -> Self {
        Self { metadata, scalars }
    }

    pub fn check_scalar(&self, name: &str, actual: f64) -> Option<BaselineCheck> {
        self.scalars
            .iter()
            .find(|baseline| baseline.name == name)
            .map(|baseline| baseline.check(actual))
    }

    pub fn accept_scalar(&mut self, name: impl Into<String>, actual: f64, tolerance: Tolerance) {
        let name = name.into();
        if let Some(existing) = self
            .scalars
            .iter_mut()
            .find(|baseline| baseline.name == name)
        {
            existing.expected = actual;
            existing.tolerance = tolerance;
        } else {
            self.scalars
                .push(ScalarBaseline::new(name, actual, tolerance));
        }
    }
}

impl ScalarBaseline {
    pub fn new(name: impl Into<String>, expected: f64, tolerance: Tolerance) -> Self {
        Self {
            name: name.into(),
            expected,
            tolerance,
        }
    }

    pub fn check(&self, actual: f64) -> BaselineCheck {
        let difference = (actual - self.expected).abs();
        let allowed_difference = self.tolerance.allowed_difference(self.expected);

        BaselineCheck {
            name: self.name.clone(),
            expected: self.expected,
            actual,
            difference,
            allowed_difference,
            passed: difference <= allowed_difference,
        }
    }

    pub fn to_line(&self) -> String {
        format!(
            "{}|{}|{}|{}",
            self.name, self.expected, self.tolerance.absolute, self.tolerance.relative
        )
    }

    pub fn from_line(line: &str) -> Option<Self> {
        let mut parts = line.split('|');
        let name = parts.next()?.to_string();
        let expected = parts.next()?.parse().ok()?;
        let absolute = parts.next()?.parse().ok()?;
        let relative = parts.next()?.parse().ok()?;
        if parts.next().is_some() {
            return None;
        }

        Some(Self::new(name, expected, Tolerance { absolute, relative }))
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct BaselineCheck {
    pub name: String,
    pub expected: f64,
    pub actual: f64,
    pub difference: f64,
    pub allowed_difference: f64,
    pub passed: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConvergenceCheck {
    pub coarse_value: f64,
    pub fine_value: f64,
    pub absolute_difference: f64,
    pub relative_difference: f64,
    pub tolerance: Tolerance,
    pub passed: bool,
}

pub fn compare_convergence(
    coarse_value: f64,
    fine_value: f64,
    tolerance: Tolerance,
) -> ConvergenceCheck {
    let absolute_difference = (fine_value - coarse_value).abs();
    let denominator = fine_value.abs().max(f64::MIN_POSITIVE);
    let relative_difference = absolute_difference / denominator;
    let passed = absolute_difference <= tolerance.allowed_difference(fine_value);

    ConvergenceCheck {
        coarse_value,
        fine_value,
        absolute_difference,
        relative_difference,
        tolerance,
        passed,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scalar_baseline_passes_inside_absolute_tolerance() {
        let baseline = ScalarBaseline::new("mean_torque_nm", 100.0, Tolerance::absolute(0.5));
        let check = baseline.check(100.25);

        assert!(check.passed);
        assert_eq!(check.allowed_difference, 0.5);
    }

    #[test]
    fn scalar_baseline_fails_outside_tolerance() {
        let baseline =
            ScalarBaseline::new("peak_pressure_pa", 5_000_000.0, Tolerance::relative(0.01));
        let check = baseline.check(5_100_001.0);

        assert!(!check.passed);
    }

    #[test]
    fn scalar_baseline_round_trips_through_line_format() {
        let baseline = ScalarBaseline::new("power_kw", 80.0, Tolerance::combined(0.1, 0.01));
        let parsed = ScalarBaseline::from_line(&baseline.to_line()).expect("baseline should parse");

        assert_eq!(parsed, baseline);
    }

    #[test]
    fn convergence_check_passes_when_refined_values_are_close() {
        let check = compare_convergence(100.0, 100.1, Tolerance::relative(0.01));

        assert!(check.passed);
        assert!(check.relative_difference < 0.01);
    }

    #[test]
    fn regression_baseline_records_metadata_and_checks_scalars() {
        let baseline = RegressionBaseline::new(
            BaselineMetadata {
                simulator_id: "test-build".to_string(),
                profile: "real_time".to_string(),
                engine_configuration: "single-cylinder fixture".to_string(),
                environment: "100kPa 300K".to_string(),
                timestep_seconds: 0.0001,
                substeps: 1,
            },
            vec![ScalarBaseline::new(
                "mean_torque_nm",
                10.0,
                Tolerance::absolute(0.1),
            )],
        );

        assert_eq!(baseline.metadata.profile, "real_time");
        assert!(
            baseline
                .check_scalar("mean_torque_nm", 10.05)
                .unwrap()
                .passed
        );
    }

    #[test]
    fn regression_baseline_accepts_changed_scalar_intentionally() {
        let mut baseline = RegressionBaseline::new(
            BaselineMetadata {
                simulator_id: "test-build".to_string(),
                profile: "render".to_string(),
                engine_configuration: "single-cylinder fixture".to_string(),
                environment: "100kPa 300K".to_string(),
                timestep_seconds: 0.00001,
                substeps: 4,
            },
            Vec::new(),
        );

        baseline.accept_scalar("peak_pressure_pa", 5_000_000.0, Tolerance::relative(0.01));

        assert!(
            baseline
                .check_scalar("peak_pressure_pa", 5_010_000.0)
                .unwrap()
                .passed
        );
    }
}
