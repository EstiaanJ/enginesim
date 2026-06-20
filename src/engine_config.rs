use crate::combustion::WiebeParameters;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct EngineDefinition {
    pub metadata: EngineMetadata,
    pub geometry: CylinderGeometryDefinition,
    pub gas: GasDefinition,
    pub boundaries: BoundaryDefinition,
    pub valves: ValveTrainDefinition,
    pub crank: CrankDefinition,
    pub combustion: CombustionDefinition,
    pub simulation: SimulationDefinition,
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

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SimulationDefinition {
    pub timestep_seconds: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_engine_definition_json() {
        let json = include_str!("../data/engines/gn250.json");
        let definition = EngineDefinition::from_json_str(json).expect("GN250 JSON should parse");

        assert_eq!(definition.metadata.name, "Suzuki GN250 approximation");
        assert_eq!(definition.geometry.bore_m, 0.072);
        assert_eq!(definition.geometry.stroke_m, 0.0612);
        assert_eq!(definition.geometry.connecting_rod_length_m, 0.115);
        assert_eq!(definition.valves.intake.open_angle_deg, 348.0);
        assert_eq!(definition.valves.intake.close_angle_deg, 582.0);
        assert_eq!(definition.valves.intake.valve_diameter_m, 0.026);
        assert_eq!(definition.valves.intake.valve_count, 2);
        assert_eq!(definition.valves.exhaust.open_angle_deg, 135.0);
        assert_eq!(definition.valves.exhaust.close_angle_deg, 370.0);
        assert_eq!(definition.valves.exhaust.valve_diameter_m, 0.022);
        assert_eq!(definition.valves.exhaust.valve_count, 2);
        assert_eq!(definition.combustion.lambda_target, 1.0);
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
