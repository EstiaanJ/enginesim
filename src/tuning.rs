use crate::engine_config::EngineDefinition;
use crate::engine_handling::EngineHandlingDefinition;

/// Holds the committed engine/handling config (what the running sim is built
/// from) alongside an editable draft. The GUI edits the draft; "Write Changes"
/// applies the draft and resets the sim, "Undo Changes" reverts the draft.
///
/// This is deliberately egui-free so the apply/undo/dirty logic is unit-testable.
#[derive(Debug, Clone, PartialEq)]
pub struct TuningSession {
    committed_engine: EngineDefinition,
    committed_handling: EngineHandlingDefinition,
    pub draft_engine: EngineDefinition,
    pub draft_handling: EngineHandlingDefinition,
}

impl TuningSession {
    pub fn new(engine: EngineDefinition, handling: EngineHandlingDefinition) -> Self {
        Self {
            committed_engine: engine.clone(),
            committed_handling: handling.clone(),
            draft_engine: engine,
            draft_handling: handling,
        }
    }

    pub fn committed_engine(&self) -> &EngineDefinition {
        &self.committed_engine
    }

    pub fn committed_handling(&self) -> &EngineHandlingDefinition {
        &self.committed_handling
    }

    /// True when the draft differs from the committed config (i.e. there are
    /// pending changes that "Write Changes" would apply).
    pub fn is_dirty(&self) -> bool {
        self.draft_engine != self.committed_engine || self.draft_handling != self.committed_handling
    }

    /// Apply the draft to the committed config. The caller should rebuild the
    /// sim from the committed config afterwards.
    pub fn write_changes(&mut self) {
        self.committed_engine = self.draft_engine.clone();
        self.committed_handling = self.draft_handling.clone();
    }

    /// Discard draft edits, reverting to the committed config.
    pub fn undo_changes(&mut self) {
        self.draft_engine = self.committed_engine.clone();
        self.draft_handling = self.committed_handling.clone();
    }

    /// Replace both committed and draft (e.g. after reloading from disk).
    pub fn replace(&mut self, engine: EngineDefinition, handling: EngineHandlingDefinition) {
        *self = Self::new(engine, handling);
    }
}

/// Circular cross-section diameter (in mm) that yields the given flow area.
pub fn diameter_mm_from_area_m2(area_m2: f64) -> f64 {
    if area_m2 <= 0.0 {
        return 0.0;
    }
    (4.0 * area_m2 / std::f64::consts::PI).sqrt() * 1000.0
}

/// Flow area (m^2) for a circular cross-section of the given diameter (in mm).
pub fn area_m2_from_diameter_mm(diameter_mm: f64) -> f64 {
    let diameter_m = (diameter_mm.max(0.0)) / 1000.0;
    std::f64::consts::PI * 0.25 * diameter_m * diameter_m
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixtures() -> (EngineDefinition, EngineHandlingDefinition) {
        let engine =
            EngineDefinition::from_json_str(include_str!("../data/engines/gn250.json")).unwrap();
        (engine, EngineHandlingDefinition::default())
    }

    #[test]
    fn new_session_is_clean() {
        let (engine, handling) = fixtures();
        let session = TuningSession::new(engine, handling);
        assert!(!session.is_dirty());
    }

    #[test]
    fn editing_draft_marks_dirty_and_write_commits() {
        let (engine, handling) = fixtures();
        let mut session = TuningSession::new(engine, handling);

        session.draft_engine.geometry.compression_ratio = 11.5;
        assert!(session.is_dirty());
        assert_eq!(session.committed_engine().geometry.compression_ratio, 8.9);

        session.write_changes();
        assert!(!session.is_dirty());
        assert_eq!(session.committed_engine().geometry.compression_ratio, 11.5);
    }

    #[test]
    fn undo_reverts_draft_to_committed() {
        let (engine, handling) = fixtures();
        let mut session = TuningSession::new(engine, handling);

        session.draft_engine.geometry.compression_ratio = 13.0;
        session.draft_handling.timestep_seconds = 0.0001;
        assert!(session.is_dirty());

        session.undo_changes();
        assert!(!session.is_dirty());
        assert_eq!(session.draft_engine.geometry.compression_ratio, 8.9);
        assert_eq!(session.draft_handling.timestep_seconds, 0.00005);
    }

    #[test]
    fn diameter_area_round_trip() {
        let area = 0.0018;
        let diameter_mm = diameter_mm_from_area_m2(area);
        let back = area_m2_from_diameter_mm(diameter_mm);
        assert!((back - area).abs() < 1.0e-12);
    }

    #[test]
    fn diameter_from_known_area() {
        // A 47.873 mm bore gives ~0.0018 m^2.
        let diameter_mm = diameter_mm_from_area_m2(0.0018);
        assert!((diameter_mm - 47.873).abs() < 0.01);
    }
}
