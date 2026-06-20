use crate::combustion::{MixtureLimits, WiebeParameters, default_mixture_limits};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct EngineDefinition {
    pub metadata: EngineMetadata,
    pub geometry: CylinderGeometryDefinition,
    pub gas: GasDefinition,
    pub boundaries: BoundaryDefinition,
    #[serde(default)]
    pub intake_exhaust: IntakeExhaustDefinition,
    pub valves: ValveTrainDefinition,
    pub crank: CrankDefinition,
    pub combustion: CombustionDefinition,
}

impl EngineDefinition {
    pub fn from_json_str(json: &str) -> serde_json::Result<Self> {
        serde_json::from_str(json)
    }

    pub fn to_json_string_pretty(&self) -> serde_json::Result<String> {
        serde_json::to_string_pretty(self)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct EngineMetadata {
    pub name: String,
    pub manufacturer: String,
    pub notes: String,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct CylinderGeometryDefinition {
    pub bore_m: f64,
    pub stroke_m: f64,
    pub connecting_rod_length_m: f64,
    pub compression_ratio: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct GasDefinition {
    pub gas_constant_j_per_kg_k: f64,
    pub specific_heat_ratio: f64,
    pub minimum_mass_kg: f64,
    pub minimum_temperature_k: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct BoundaryDefinition {
    pub intake_pressure_pa: f64,
    pub intake_temperature_k: f64,
    pub exhaust_pressure_pa: f64,
    pub exhaust_temperature_k: f64,
    pub crankcase_pressure_pa: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct IntakeExhaustDefinition {
    pub intake_plenum_volume_m3: f64,
    pub throttle_maximum_area_m2: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub idle_throttle_maximum_area_m2: Option<f64>,
    pub intake_runner: PipeDefinition,
    pub exhaust_runner: PipeDefinition,
}

impl Default for IntakeExhaustDefinition {
    fn default() -> Self {
        Self {
            intake_plenum_volume_m3: 0.0015,
            throttle_maximum_area_m2: 0.00048,
            idle_throttle_maximum_area_m2: None,
            intake_runner: PipeDefinition {
                number_of_cells: 8,
                total_length_m: 0.4,
                area_m2: 0.00048,
            },
            exhaust_runner: PipeDefinition {
                number_of_cells: 8,
                total_length_m: 0.4,
                area_m2: 0.000345,
            },
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct PipeDefinition {
    pub number_of_cells: usize,
    pub total_length_m: f64,
    pub area_m2: f64,
}

impl PipeDefinition {
    /// Per-cell length derived from the total runner length and cell count.
    /// The 1D solver still works on uniform cells, so this is how the
    /// `total_length_m` / `number_of_cells` data maps onto cell geometry.
    pub fn cell_length_m(&self) -> f64 {
        self.total_length_m / self.number_of_cells.max(1) as f64
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct ValveTrainDefinition {
    pub intake: ValveDefinition,
    pub exhaust: ValveDefinition,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct ValveDefinition {
    pub open_angle_deg: f64,
    pub close_angle_deg: f64,
    pub valve_diameter_m: f64,
    pub valve_count: u32,
    pub max_effective_area_m2: f64,
    pub discharge_coefficient: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct CrankDefinition {
    pub moment_of_inertia_kg_m2: f64,
    pub initial_speed_rpm: f64,
    pub initial_crank_angle_deg: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct CombustionDefinition {
    pub enabled: bool,
    pub lambda_target: f64,
    pub fuel_mass_per_cycle_kg: f64,
    pub fuel_lower_heating_value_j_per_kg: f64,
    pub combustion_efficiency: f64,
    pub heat_loss_fraction: f64,
    #[serde(default = "default_mixture_limits")]
    pub mixture_limits: MixtureLimits,
    pub spark_timing: SparkTimingDefinition,
    pub ignition_delay_deg: f64,
    pub wiebe: WiebeParameters,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SparkTimingDefinition {
    pub low_speed_rpm: f64,
    pub low_speed_advance_deg_btdc: f64,
    pub high_speed_rpm: f64,
    pub high_speed_advance_deg_btdc: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_engine_definition_json() {
        let json = include_str!("../data/engines/gn250.json");
        let definition = EngineDefinition::from_json_str(json).expect("GN250 JSON should parse");

        // Pin only the stable engine identity. Valve timing/diameters, runner
        // cell counts, lambda etc. are tunable (and saved back to this file from
        // the tuning GUI), so this parse test asserts invariants/ranges for them
        // rather than exact values that legitimately change when the engine is
        // retuned. Exact serialization fidelity is covered by the round-trip test.
        assert_eq!(definition.metadata.name, "Suzuki GN250 approximation");
        assert_eq!(definition.geometry.bore_m, 0.072);
        assert_eq!(definition.geometry.stroke_m, 0.0612);
        assert_eq!(definition.geometry.connecting_rod_length_m, 0.115);

        for valve in [&definition.valves.intake, &definition.valves.exhaust] {
            assert!((0.0..=720.0).contains(&valve.open_angle_deg));
            assert!((0.0..=720.0).contains(&valve.close_angle_deg));
            assert!(valve.valve_diameter_m > 0.0);
            assert!(valve.valve_count >= 1);
            assert!((0.0..=1.0).contains(&valve.discharge_coefficient));
        }

        let intake_runner = &definition.intake_exhaust.intake_runner;
        assert!(intake_runner.number_of_cells >= 1);
        assert!(intake_runner.total_length_m > 0.0);
        assert_eq!(
            intake_runner.cell_length_m(),
            intake_runner.total_length_m / intake_runner.number_of_cells as f64
        );
        assert!(definition.intake_exhaust.intake_plenum_volume_m3 > 0.0);
        assert!(definition.combustion.lambda_target > 0.0);
        assert_eq!(
            definition.combustion.mixture_limits,
            default_mixture_limits()
        );
        assert!(definition.combustion.enabled);
    }

    #[test]
    fn engine_definition_round_trips_through_json() {
        let json = include_str!("../data/engines/gn250.json");
        let definition = EngineDefinition::from_json_str(json).expect("GN250 JSON should parse");
        let serialized = definition
            .to_json_string_pretty()
            .expect("definition should serialize");
        let reparsed =
            EngineDefinition::from_json_str(&serialized).expect("serialized JSON should parse");

        assert_eq!(reparsed, definition);
    }
}
