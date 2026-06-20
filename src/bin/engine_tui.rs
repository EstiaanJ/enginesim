//! Headless terminal testing mode.
//!
//! Loads an engine definition and a JSON test profile, runs the configured grid
//! and/or sweep, and prints a report of the time-averaged telemetry to stdout.
//!
//! Usage:
//!     engine_tui --test <profile.json> [--engine <engine.json>]
//!
//! When `--engine` is omitted the bundled GN250 definition is used.

use std::process::ExitCode;

use enginesim::engine_config::EngineDefinition;
use enginesim::test_harness::{TestProfile, run_and_report};

const BUNDLED_GN250: &str = include_str!("../../data/engines/gn250.json");

const USAGE: &str = "\
engine_tui - headless engine test runner

USAGE:
    engine_tui --test <profile.json> [--engine <engine.json>]

OPTIONS:
    -t, --test <path>      JSON test profile to run (required)
    -e, --engine <path>    Engine definition JSON (defaults to the bundled GN250)
    -h, --help             Show this help
";

struct Args {
    test_path: Option<String>,
    engine_path: Option<String>,
}

fn parse_args() -> Result<Args, String> {
    let mut args = Args {
        test_path: None,
        engine_path: None,
    };
    let mut iter = std::env::args().skip(1);
    while let Some(flag) = iter.next() {
        match flag.as_str() {
            "-h" | "--help" => {
                print!("{USAGE}");
                std::process::exit(0);
            }
            "-t" | "--test" => {
                args.test_path = Some(
                    iter.next()
                        .ok_or_else(|| format!("{flag} requires a path argument"))?,
                );
            }
            "-e" | "--engine" => {
                args.engine_path = Some(
                    iter.next()
                        .ok_or_else(|| format!("{flag} requires a path argument"))?,
                );
            }
            other => return Err(format!("unrecognized argument: {other}")),
        }
    }
    Ok(args)
}

fn run() -> Result<(), String> {
    let args = parse_args()?;

    let test_path = args
        .test_path
        .ok_or_else(|| format!("missing required --test argument\n\n{USAGE}"))?;
    let test_json = std::fs::read_to_string(&test_path)
        .map_err(|error| format!("failed to read test profile {test_path}: {error}"))?;
    let test = TestProfile::from_json_str(&test_json)
        .map_err(|error| format!("failed to parse test profile {test_path}: {error}"))?;
    test.validate()?;

    let (engine_json, engine_source) = match &args.engine_path {
        Some(path) => {
            let json = std::fs::read_to_string(path)
                .map_err(|error| format!("failed to read engine {path}: {error}"))?;
            (json, path.clone())
        }
        None => (BUNDLED_GN250.to_string(), "bundled GN250".to_string()),
    };
    let definition = EngineDefinition::from_json_str(&engine_json)
        .map_err(|error| format!("failed to parse engine {engine_source}: {error}"))?;

    print!("{}", run_and_report(&definition, &test));
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(message) => {
            eprintln!("error: {message}");
            ExitCode::FAILURE
        }
    }
}
