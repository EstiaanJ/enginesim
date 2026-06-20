use crate::engine_config::ValveDefinition;

pub const ENGINE_CYCLE_RADIANS: f64 = std::f64::consts::TAU * 2.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ValveEvent {
    pub open_angle_rad: f64,
    pub close_angle_rad: f64,
    pub max_effective_area_m2: f64,
    pub discharge_coefficient: f64,
}

impl ValveEvent {
    pub fn from_definition(definition: ValveDefinition) -> Self {
        Self {
            open_angle_rad: definition.open_angle_deg.to_radians(),
            close_angle_rad: definition.close_angle_deg.to_radians(),
            max_effective_area_m2: definition.max_effective_area_m2,
            discharge_coefficient: definition.discharge_coefficient,
        }
    }

    pub fn lift_fraction(self, crank_angle_rad: f64) -> f64 {
        assert!(
            self.max_effective_area_m2 >= 0.0,
            "max effective area must be non-negative"
        );

        let open = normalize_cycle_angle_rad(self.open_angle_rad);
        let close = normalize_cycle_angle_rad(self.close_angle_rad);
        let angle = normalize_cycle_angle_rad(crank_angle_rad);
        let duration = positive_cycle_delta_rad(open, close);
        if duration <= 0.0 {
            return 0.0;
        }

        let elapsed = positive_cycle_delta_rad(open, angle);
        if elapsed > duration {
            return 0.0;
        }

        0.5 * (1.0 - (std::f64::consts::TAU * elapsed / duration).cos())
    }

    pub fn effective_area_m2(self, crank_angle_rad: f64) -> f64 {
        self.max_effective_area_m2 * self.lift_fraction(crank_angle_rad)
    }
}

pub fn normalize_cycle_angle_rad(angle_rad: f64) -> f64 {
    angle_rad.rem_euclid(ENGINE_CYCLE_RADIANS)
}

pub fn positive_cycle_delta_rad(start_angle_rad: f64, end_angle_rad: f64) -> f64 {
    normalize_cycle_angle_rad(end_angle_rad - start_angle_rad)
}

pub fn crossed_cycle_angle_rad(
    start_angle_rad: f64,
    end_angle_rad: f64,
    target_angle_rad: f64,
) -> bool {
    let step_delta = positive_cycle_delta_rad(start_angle_rad, end_angle_rad);
    if step_delta <= 0.0 {
        return false;
    }

    let target_delta = positive_cycle_delta_rad(start_angle_rad, target_angle_rad);
    target_delta > 0.0 && target_delta <= step_delta
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

    fn valve() -> ValveEvent {
        ValveEvent {
            open_angle_rad: 350.0_f64.to_radians(),
            close_angle_rad: 580.0_f64.to_radians(),
            max_effective_area_m2: 0.0002,
            discharge_coefficient: 0.7,
        }
    }

    #[test]
    fn valve_is_closed_before_opening() {
        assert_approx_eq(valve().effective_area_m2(300.0_f64.to_radians()), 0.0);
    }

    #[test]
    fn valve_reaches_max_area_at_mid_event() {
        let event = valve();
        let mid_angle = 465.0_f64.to_radians();

        assert_approx_eq(
            event.effective_area_m2(mid_angle),
            event.max_effective_area_m2,
        );
    }

    #[test]
    fn valve_handles_events_that_wrap_across_cycle_boundary() {
        let event = ValveEvent {
            open_angle_rad: 700.0_f64.to_radians(),
            close_angle_rad: 40.0_f64.to_radians(),
            max_effective_area_m2: 0.0002,
            discharge_coefficient: 0.7,
        };

        assert!(event.effective_area_m2(710.0_f64.to_radians()) > 0.0);
        assert!(event.effective_area_m2(20.0_f64.to_radians()) > 0.0);
        assert_approx_eq(event.effective_area_m2(200.0_f64.to_radians()), 0.0);
    }

    #[test]
    fn detects_crossed_cycle_angle() {
        assert!(crossed_cycle_angle_rad(
            700.0_f64.to_radians(),
            10.0_f64.to_radians(),
            0.0,
        ));
        assert!(!crossed_cycle_angle_rad(
            100.0_f64.to_radians(),
            110.0_f64.to_radians(),
            120.0_f64.to_radians(),
        ));
    }
}
