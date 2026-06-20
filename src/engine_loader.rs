use std::path::{Path, PathBuf};

use crate::engine_config::EngineDefinition;
use crate::engine_handling::EngineHandlingDefinition;

/// Where a loaded value came from, so the GUI can show whether it is editing
/// on-disk data or the binary's bundled fallback.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LoadSource {
    /// Parsed from the given file on disk.
    Disk(PathBuf),
    /// The file was missing or unparseable; the bundled default was used.
    BundledFallback { reason: String },
}

impl LoadSource {
    pub fn is_disk(&self) -> bool {
        matches!(self, LoadSource::Disk(_))
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct LoadedEngine {
    pub definition: EngineDefinition,
    pub handling: EngineHandlingDefinition,
    pub engine_source: LoadSource,
    pub handling_source: LoadSource,
}

/// Sibling handling path for an engine JSON path: `foo.json` -> `foo.handling.json`.
pub fn handling_path_for(engine_path: &Path) -> PathBuf {
    let stem = engine_path
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_default();
    let file_name = format!("{stem}.handling.json");
    match engine_path.parent() {
        Some(parent) => parent.join(file_name),
        None => PathBuf::from(file_name),
    }
}

/// Load an engine and its sibling handling config from disk, falling back to
/// the supplied bundled defaults when a file is missing or fails to parse.
///
/// This is what removes the recompile-to-tune requirement: editing the JSON on
/// disk and reloading takes effect without rebuilding the binary.
pub fn load_engine_and_handling(
    engine_path: &Path,
    bundled_definition: &EngineDefinition,
    bundled_handling: &EngineHandlingDefinition,
) -> LoadedEngine {
    let (definition, engine_source) =
        match load_parsed(engine_path, EngineDefinition::from_json_str) {
            Ok(value) => (value, LoadSource::Disk(engine_path.to_path_buf())),
            Err(reason) => (
                bundled_definition.clone(),
                LoadSource::BundledFallback { reason },
            ),
        };

    let handling_path = handling_path_for(engine_path);
    let (handling, handling_source) =
        match load_parsed(&handling_path, EngineHandlingDefinition::from_json_str) {
            Ok(value) => (value, LoadSource::Disk(handling_path)),
            Err(reason) => (
                bundled_handling.clone(),
                LoadSource::BundledFallback { reason },
            ),
        };

    LoadedEngine {
        definition,
        handling,
        engine_source,
        handling_source,
    }
}

/// Write an engine and its handling config back to disk as pretty JSON, to the
/// engine path and its sibling `<engine>.handling.json`. This is the inverse of
/// [`load_engine_and_handling`] and lets the GUI persist tuned values.
pub fn save_engine_and_handling(
    engine_path: &Path,
    definition: &EngineDefinition,
    handling: &EngineHandlingDefinition,
) -> Result<(), String> {
    let engine_json = definition
        .to_json_string_pretty()
        .map_err(|err| format!("serialize engine: {err}"))?;
    std::fs::write(engine_path, format!("{engine_json}\n"))
        .map_err(|err| format!("{engine_path:?}: {err}"))?;

    let handling_path = handling_path_for(engine_path);
    let handling_json = handling
        .to_json_string_pretty()
        .map_err(|err| format!("serialize handling: {err}"))?;
    std::fs::write(&handling_path, format!("{handling_json}\n"))
        .map_err(|err| format!("{handling_path:?}: {err}"))?;

    Ok(())
}

fn load_parsed<T, F>(path: &Path, parse: F) -> Result<T, String>
where
    F: Fn(&str) -> serde_json::Result<T>,
{
    let contents = std::fs::read_to_string(path).map_err(|err| format!("{path:?}: {err}"))?;
    parse(&contents).map_err(|err| format!("{path:?}: {err}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn handling_path_is_sibling_of_engine_path() {
        let engine = Path::new("data/engines/gn250.json");
        assert_eq!(
            handling_path_for(engine),
            PathBuf::from("data/engines/gn250.handling.json")
        );
    }

    #[test]
    fn falls_back_to_bundled_when_files_missing() {
        let bundled_definition =
            EngineDefinition::from_json_str(include_str!("../data/engines/gn250.json")).unwrap();
        let bundled_handling = EngineHandlingDefinition::default();
        let missing = Path::new("/nonexistent/path/to/engine.json");

        let loaded = load_engine_and_handling(missing, &bundled_definition, &bundled_handling);

        assert_eq!(loaded.definition, bundled_definition);
        assert!(matches!(
            loaded.engine_source,
            LoadSource::BundledFallback { .. }
        ));
        assert!(matches!(
            loaded.handling_source,
            LoadSource::BundledFallback { .. }
        ));
    }

    #[test]
    fn save_then_load_round_trips_engine_and_handling() {
        let mut definition =
            EngineDefinition::from_json_str(include_str!("../data/engines/gn250.json")).unwrap();
        definition.geometry.compression_ratio = 12.3;
        let handling = EngineHandlingDefinition {
            set_points_rpm: vec![1200.0, 4200.0],
            ..EngineHandlingDefinition::default()
        };

        let dir = std::env::temp_dir().join(format!("enginesim_save_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let engine_path = dir.join("probe.json");

        save_engine_and_handling(&engine_path, &definition, &handling).unwrap();
        let loaded = load_engine_and_handling(&engine_path, &definition, &handling);

        assert!(loaded.engine_source.is_disk());
        assert!(loaded.handling_source.is_disk());
        assert_eq!(loaded.definition.geometry.compression_ratio, 12.3);
        assert_eq!(loaded.handling.set_points_rpm, vec![1200.0, 4200.0]);

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn loads_real_gn250_files_from_disk() {
        // Run from the crate root, where data/engines exists.
        let engine_path = Path::new("data/engines/gn250.json");
        if !engine_path.exists() {
            return;
        }
        let bundled_definition =
            EngineDefinition::from_json_str(include_str!("../data/engines/gn250.json")).unwrap();
        let bundled_handling = EngineHandlingDefinition::default();

        let loaded = load_engine_and_handling(engine_path, &bundled_definition, &bundled_handling);

        assert!(loaded.engine_source.is_disk());
        assert!(loaded.handling_source.is_disk());
        assert_eq!(loaded.handling.set_points_rpm, vec![1000.0, 2500.0, 5000.0]);
    }
}
