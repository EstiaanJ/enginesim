use crate::engine_config::{ValveDefinition, ValveLiftProfileDefinition, ValveLiftProfileModel};

pub const ENGINE_CYCLE_RADIANS: f64 = std::f64::consts::TAU * 2.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ValveEvent {
    pub open_angle_rad: f64,
    pub close_angle_rad: f64,
    pub max_lift_m: f64,
    pub valve_diameter_m: f64,
    pub valve_count: u32,
    /// Fraction of the open duration spent ramping from closed to full lift.
    pub opening_ramp_fraction: f64,
    /// Fraction of the open duration held at full lift between the ramps.
    pub plateau_fraction: f64,
    pub lift_profile: ValveLiftProfileDefinition,
    pub discharge_coefficient: f64,
}

impl ValveEvent {
    pub fn from_definition(definition: ValveDefinition) -> Self {
        Self {
            open_angle_rad: definition.open_angle_deg.to_radians(),
            close_angle_rad: definition.close_angle_deg.to_radians(),
            max_lift_m: definition.max_lift_m,
            valve_diameter_m: definition.valve_diameter_m,
            valve_count: definition.valve_count,
            opening_ramp_fraction: definition.opening_ramp_fraction,
            plateau_fraction: definition.plateau_fraction,
            lift_profile: definition.lift_profile,
            discharge_coefficient: definition.discharge_coefficient,
        }
    }

    /// Normalized lift profile in `[0, 1]`: a flat-top raised cosine with an
    /// opening ramp (`opening_ramp_fraction`), a plateau at full lift
    /// (`plateau_fraction`), and a closing ramp filling the remainder. With
    /// `opening_ramp_fraction = 0.5` and `plateau_fraction = 0.0` this reduces
    /// exactly to the symmetric `0.5 * (1 - cos(2*pi*u))` cosine lobe.
    pub fn lift_fraction(self, crank_angle_rad: f64) -> f64 {
        assert!(self.max_lift_m >= 0.0, "max lift must be non-negative");

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

        let progress = (elapsed / duration).clamp(0.0, 1.0);
        if self.lift_profile.model == ValveLiftProfileModel::SegmentedCubic {
            return segmented_cubic_lift_fraction(progress, self.lift_profile);
        }

        let opening = self.opening_ramp_fraction.clamp(1.0e-6, 1.0);
        let plateau = self.plateau_fraction.clamp(0.0, 1.0 - opening);
        let closing = (1.0 - opening - plateau).max(0.0);
        let pi = std::f64::consts::PI;

        if progress < opening {
            0.5 * (1.0 - (pi * progress / opening).cos())
        } else if progress <= opening + plateau {
            1.0
        } else if closing > 0.0 {
            0.5 * (1.0 - (pi * (1.0 - progress) / closing).cos())
        } else {
            0.0
        }
    }

    /// Peak geometric flow area at full lift: the curtain area
    /// (`pi * D * lift`) capped by the valve-head circle (`pi/4 * D^2`),
    /// summed over `valve_count` valves. The discharge coefficient is applied
    /// separately at the flow call site.
    pub fn peak_effective_area_m2(self) -> f64 {
        let curtain_area_m2 =
            std::f64::consts::PI * self.valve_diameter_m.max(0.0) * self.max_lift_m.max(0.0);
        let port_area_m2 = std::f64::consts::FRAC_PI_4 * self.valve_diameter_m.max(0.0).powi(2);
        self.valve_count.max(1) as f64 * curtain_area_m2.min(port_area_m2)
    }

    /// Instantaneous geometric flow area at the given crank angle: the curtain
    /// area at the current lift (`pi * D * lift(theta)`) capped by the
    /// valve-head circle, summed over `valve_count` valves.
    pub fn effective_area_m2(self, crank_angle_rad: f64) -> f64 {
        let lift_m = self.max_lift_m.max(0.0) * self.lift_fraction(crank_angle_rad);
        let curtain_area_m2 = std::f64::consts::PI * self.valve_diameter_m.max(0.0) * lift_m;
        let port_area_m2 = std::f64::consts::FRAC_PI_4 * self.valve_diameter_m.max(0.0).powi(2);
        self.valve_count.max(1) as f64 * curtain_area_m2.min(port_area_m2)
    }
}

pub fn segmented_cubic_lift_fraction(progress: f64, profile: ValveLiftProfileDefinition) -> f64 {
    let progress = progress.clamp(0.0, 1.0);
    let ramp_lift = profile.ramp_lift_fraction.clamp(0.0, 1.0);
    let (ramp_duration, main_duration, dwell_duration) = normalized_segment_fractions(profile);

    let ramp_end = ramp_duration;
    let main_up_end = ramp_end + main_duration;
    let dwell_end = main_up_end + dwell_duration;
    let main_down_end = dwell_end + main_duration;

    if ramp_duration > 0.0 && progress <= ramp_end {
        let theta_s = progress / ramp_duration;
        ramp_lift * ramp_specific_lift(theta_s)
    } else if main_duration > 0.0 && progress <= main_up_end {
        let theta_s = (progress - ramp_end) / main_duration;
        ramp_lift + main_specific_lift(theta_s) * (1.0 - ramp_lift)
    } else if dwell_duration > 0.0 && progress <= dwell_end {
        1.0
    } else if main_duration > 0.0 && progress <= main_down_end {
        let theta_s = (main_down_end - progress) / main_duration;
        ramp_lift + main_specific_lift(theta_s) * (1.0 - ramp_lift)
    } else if ramp_duration > 0.0 {
        let theta_s = (1.0 - progress) / ramp_duration;
        ramp_lift * ramp_specific_lift(theta_s)
    } else {
        0.0
    }
    .clamp(0.0, 1.0)
}

pub fn normalized_segment_fractions(profile: ValveLiftProfileDefinition) -> (f64, f64, f64) {
    let ramp = profile.ramp_duration_fraction.max(0.0);
    let main = profile.main_lift_duration_fraction.max(0.0);
    let dwell = profile.dwell_duration_fraction.max(0.0);
    let total = 2.0 * ramp + 2.0 * main + dwell;
    if total <= 0.0 {
        return (0.5, 0.0, 0.0);
    }
    (ramp / total, main / total, dwell / total)
}

/// Third-order specific ramp lift. This is the lower curve from the design
/// reference: flat at valve opening, steepening into the main-lift segment.
pub fn ramp_specific_lift(theta_s: f64) -> f64 {
    theta_s.clamp(0.0, 1.0).powi(3)
}

/// Third-order specific main lift. It starts with the same normalized slope as
/// `ramp_specific_lift` ends with, and eases to zero slope at full lift.
pub fn main_specific_lift(theta_s: f64) -> f64 {
    let theta_s = theta_s.clamp(0.0, 1.0);
    3.0 * theta_s - 3.0 * theta_s.powi(2) + theta_s.powi(3)
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
        // max_lift 1.7 mm on a 30 mm valve stays well under the port cap
        // (D/4 = 7.5 mm), so the valve is curtain-area limited at full lift.
        ValveEvent {
            open_angle_rad: 350.0_f64.to_radians(),
            close_angle_rad: 580.0_f64.to_radians(),
            max_lift_m: 0.0017,
            valve_diameter_m: 0.03,
            valve_count: 1,
            opening_ramp_fraction: 0.5,
            plateau_fraction: 0.0,
            lift_profile: ValveLiftProfileDefinition::default(),
            discharge_coefficient: 0.7,
        }
    }

    #[test]
    fn valve_is_closed_before_opening() {
        assert_approx_eq(valve().effective_area_m2(300.0_f64.to_radians()), 0.0);
    }

    #[test]
    fn valve_reaches_peak_area_at_mid_event() {
        let event = valve();
        let mid_angle = 465.0_f64.to_radians();

        assert_approx_eq(
            event.effective_area_m2(mid_angle),
            event.peak_effective_area_m2(),
        );
    }

    #[test]
    fn default_shape_reproduces_symmetric_cosine_lobe() {
        // opening_ramp = 0.5, plateau = 0.0 must match 0.5*(1 - cos(2*pi*u))
        // at arbitrary progress fractions through the event.
        let event = valve();
        let open = event.open_angle_rad;
        let duration = positive_cycle_delta_rad(open, event.close_angle_rad);
        for &progress in &[0.1, 0.25, 0.37, 0.5, 0.63, 0.8, 0.95] {
            let angle = open + duration * progress;
            let expected = 0.5 * (1.0 - (std::f64::consts::TAU * progress).cos());
            assert_approx_eq(event.lift_fraction(angle), expected);
        }
    }

    #[test]
    fn plateau_holds_full_lift_across_the_dwell() {
        let event = ValveEvent {
            opening_ramp_fraction: 0.25,
            plateau_fraction: 0.5,
            ..valve()
        };
        let open = event.open_angle_rad;
        let duration = positive_cycle_delta_rad(open, event.close_angle_rad);
        // Anywhere inside the [0.25, 0.75] plateau lift is exactly full.
        for &progress in &[0.26, 0.4, 0.5, 0.6, 0.74] {
            assert_approx_eq(event.lift_fraction(open + duration * progress), 1.0);
        }
        // The ramps still start and end closed.
        assert_approx_eq(event.lift_fraction(open), 0.0);
    }

    #[test]
    fn effective_area_is_port_capped_at_high_lift() {
        // 6 mm lift on a 20 mm valve exceeds D/4 = 5 mm, so the valve-head
        // circle caps the area instead of the curtain area.
        let event = ValveEvent {
            max_lift_m: 0.006,
            valve_diameter_m: 0.02,
            valve_count: 2,
            ..valve()
        };
        let port_area_m2 = std::f64::consts::FRAC_PI_4 * 0.02_f64.powi(2);
        assert_approx_eq(event.peak_effective_area_m2(), 2.0 * port_area_m2);
    }

    #[test]
    fn valve_count_scales_effective_area_linearly() {
        let single = ValveEvent {
            valve_count: 1,
            ..valve()
        };
        let twin = ValveEvent {
            valve_count: 2,
            ..valve()
        };
        assert_approx_eq(
            twin.peak_effective_area_m2(),
            2.0 * single.peak_effective_area_m2(),
        );
    }

    #[test]
    fn valve_handles_events_that_wrap_across_cycle_boundary() {
        let event = ValveEvent {
            open_angle_rad: 700.0_f64.to_radians(),
            close_angle_rad: 40.0_f64.to_radians(),
            ..valve()
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

    #[test]
    fn segmented_cubic_profile_is_symmetric_and_reaches_full_lift() {
        let profile = ValveLiftProfileDefinition {
            model: ValveLiftProfileModel::SegmentedCubic,
            ramp_lift_fraction: 0.2,
            ramp_duration_fraction: 20.0,
            main_lift_duration_fraction: 70.0,
            dwell_duration_fraction: 30.0,
        };

        assert_approx_eq(segmented_cubic_lift_fraction(0.0, profile), 0.0);
        assert_approx_eq(segmented_cubic_lift_fraction(1.0, profile), 0.0);
        assert_approx_eq(segmented_cubic_lift_fraction(0.5, profile), 1.0);
        for &progress in &[0.05, 0.17, 0.31, 0.44] {
            assert_approx_eq(
                segmented_cubic_lift_fraction(progress, profile),
                segmented_cubic_lift_fraction(1.0 - progress, profile),
            );
        }
    }

    #[test]
    fn segmented_cubic_valve_uses_absolute_crank_angles_in_radians() {
        let event = ValveEvent {
            open_angle_rad: 350.0_f64.to_radians(),
            close_angle_rad: 590.0_f64.to_radians(),
            lift_profile: ValveLiftProfileDefinition {
                model: ValveLiftProfileModel::SegmentedCubic,
                ramp_lift_fraction: 0.25,
                ramp_duration_fraction: 20.0,
                main_lift_duration_fraction: 80.0,
                dwell_duration_fraction: 40.0,
            },
            ..valve()
        };

        assert_approx_eq(event.lift_fraction(350.0_f64.to_radians()), 0.0);
        assert_approx_eq(event.lift_fraction(470.0_f64.to_radians()), 1.0);
        assert_approx_eq(event.lift_fraction(590.0_f64.to_radians()), 0.0);
    }
}
