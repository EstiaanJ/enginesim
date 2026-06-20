use serde::{Deserialize, Serialize};

use crate::profiles::SimulationProfile;

/// "How the engine is handled in the sim" — the controls/operating settings that
/// are NOT part of the physical engine definition. These live in a sibling
/// `<engine>.handling.json` file so the engine JSON only describes hardware.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EngineHandlingDefinition {
    #[serde(default = "default_timestep_seconds")]
    pub timestep_seconds: f64,
    #[serde(default = "default_redline_cut_time_seconds")]
    pub redline_cut_time_seconds: f64,
    #[serde(default = "default_max_added_inertia_kg_m2")]
    pub max_added_inertia_kg_m2: f64,
    /// Dyno RPM points the tuning slider snaps between.
    #[serde(default = "default_set_points_rpm")]
    pub set_points_rpm: Vec<f64>,
}

impl EngineHandlingDefinition {
    pub fn from_json_str(json: &str) -> serde_json::Result<Self> {
        serde_json::from_str(json)
    }

    pub fn to_json_string_pretty(&self) -> serde_json::Result<String> {
        serde_json::to_string_pretty(self)
    }

    /// Real-time profile with the configured timestep. Replaces the old
    /// `profile_from_definition` / `default_profile_for_gui` derivation that
    /// read `EngineDefinition.simulation.timestep_seconds`.
    pub fn to_profile(&self) -> SimulationProfile {
        SimulationProfile {
            timestep_seconds: self.timestep_seconds,
            ..SimulationProfile::real_time()
        }
    }
}

impl Default for EngineHandlingDefinition {
    fn default() -> Self {
        Self {
            timestep_seconds: default_timestep_seconds(),
            redline_cut_time_seconds: default_redline_cut_time_seconds(),
            max_added_inertia_kg_m2: default_max_added_inertia_kg_m2(),
            set_points_rpm: default_set_points_rpm(),
        }
    }
}

fn default_timestep_seconds() -> f64 {
    0.00005
}

fn default_redline_cut_time_seconds() -> f64 {
    0.10
}

fn default_max_added_inertia_kg_m2() -> f64 {
    0.05
}

fn default_set_points_rpm() -> Vec<f64> {
    vec![1000.0, 2500.0, 5000.0]
}

/// Snap an RPM value to the nearest configured set point. Returns `value`
/// unchanged when no set points are configured.
pub fn nearest_set_point_rpm(value: f64, set_points_rpm: &[f64]) -> f64 {
    set_points_rpm
        .iter()
        .copied()
        .min_by(|a, b| {
            (a - value)
                .abs()
                .partial_cmp(&(b - value).abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .unwrap_or(value)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_handling_definition_json() {
        let json = include_str!("../data/engines/gn250.handling.json");
        let handling =
            EngineHandlingDefinition::from_json_str(json).expect("handling JSON should parse");

        assert_eq!(handling.timestep_seconds, 0.00005);
        assert_eq!(handling.redline_cut_time_seconds, 0.18);
        assert_eq!(handling.max_added_inertia_kg_m2, 0.15);
        assert_eq!(handling.set_points_rpm, vec![1000.0, 2500.0, 5000.0]);
    }

    #[test]
    fn handling_definition_round_trips_through_json() {
        let json = include_str!("../data/engines/gn250.handling.json");
        let handling = EngineHandlingDefinition::from_json_str(json).expect("should parse");
        let serialized = handling
            .to_json_string_pretty()
            .expect("handling should serialize");
        let reparsed = EngineHandlingDefinition::from_json_str(&serialized).expect("should parse");

        assert_eq!(reparsed, handling);
    }

    #[test]
    fn defaults_match_legacy_simulation_definition_values() {
        // These defaults must equal the values the removed `SimulationDefinition`
        // used so that engines without a handling file behave as before.
        let handling = EngineHandlingDefinition::default();
        assert_eq!(handling.timestep_seconds, 0.00005);
        assert_eq!(handling.redline_cut_time_seconds, 0.10);
        assert_eq!(handling.max_added_inertia_kg_m2, 0.05);
    }

    #[test]
    fn to_profile_overrides_only_timestep() {
        let handling = EngineHandlingDefinition {
            timestep_seconds: 0.0001,
            ..EngineHandlingDefinition::default()
        };
        let profile = handling.to_profile();
        let baseline = SimulationProfile::real_time();

        assert_eq!(profile.timestep_seconds, 0.0001);
        assert_eq!(profile.chamber_substeps, baseline.chamber_substeps);
        assert_eq!(
            profile.one_d_mesh_cells_per_meter,
            baseline.one_d_mesh_cells_per_meter
        );
        assert_eq!(profile.kind, baseline.kind);
    }

    #[test]
    fn nearest_set_point_snaps_to_closest_value() {
        let set_points = [1000.0, 2500.0, 5000.0];
        assert_eq!(nearest_set_point_rpm(900.0, &set_points), 1000.0);
        assert_eq!(nearest_set_point_rpm(1600.0, &set_points), 1000.0);
        assert_eq!(nearest_set_point_rpm(1900.0, &set_points), 2500.0);
        assert_eq!(nearest_set_point_rpm(4000.0, &set_points), 5000.0);
        assert_eq!(nearest_set_point_rpm(9000.0, &set_points), 5000.0);
    }

    #[test]
    fn nearest_set_point_returns_value_when_empty() {
        assert_eq!(nearest_set_point_rpm(1234.0, &[]), 1234.0);
    }
}
