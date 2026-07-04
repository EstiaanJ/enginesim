//! Offline exhaust-pulse renderer.
//!
//! Holds the engine at a fixed RPM and throttle, runs the 0D/1D simulation at a
//! chosen core frequency for a chosen wall of simulated time, bins the tailpipe
//! exit pressure against engine-cycle angle, and averages every cycle (after
//! discarding the first 10% as warmup) into one representative exhaust pulse.
//! That averaged single cycle is then resampled to a `.wav` file, looped to a
//! listenable length, so the steady-state exhaust note can be heard or imported.
//!
//! Example:
//!   cargo run --release --bin render_audio -- \
//!       --rpm 4000 --throttle 1.0 --seconds 2.0 --sim-hz 200000 --out exhaust.wav

use std::io::Write;

use enginesim::engine_config::EngineDefinition;
use enginesim::profiles::{SimulationProfile, SimulationProfileKind};
use enginesim::single_cylinder::{
    SingleCylinderEngine, SingleCylinderStepInputs, rpm_to_rad_per_s,
};
use enginesim::valve::ENGINE_CYCLE_RADIANS;

const DEFAULT_ENGINE_JSON: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/data/engines/gn250.json");
const WARMUP_FRACTION: f64 = 0.10;

struct Args {
    rpm: f64,
    throttle: f64,
    seconds: f64,
    sim_hz: f64,
    bins: usize,
    wav_seconds: f64,
    wav_rate: u32,
    engine_path: String,
    out_path: String,
}

fn main() {
    let args = match parse_args() {
        Ok(args) => args,
        Err(message) => {
            eprintln!("error: {message}\n");
            print_usage();
            std::process::exit(2);
        }
    };

    let json = match std::fs::read_to_string(&args.engine_path) {
        Ok(json) => json,
        Err(err) => {
            eprintln!("error: cannot read engine '{}': {err}", args.engine_path);
            std::process::exit(1);
        }
    };
    let definition = match EngineDefinition::from_json_str(&json) {
        Ok(definition) => definition,
        Err(err) => {
            eprintln!("error: cannot parse engine '{}': {err}", args.engine_path);
            std::process::exit(1);
        }
    };

    let averaged = render_average_cycle(&definition, &args);
    let samples = synthesize_wav_samples(&averaged, &args);

    if let Err(err) = write_wav_mono_i16(&args.out_path, args.wav_rate, &samples) {
        eprintln!("error: cannot write '{}': {err}", args.out_path);
        std::process::exit(1);
    }

    let fundamental_hz = args.rpm / 2.0 / 60.0; // one exhaust event per 2 rev (4-stroke)
    println!(
        "Rendered {:.0} rpm @ throttle {:.2} | {:.2}s @ {:.0} kHz sim, first {:.0}% discarded",
        args.rpm,
        args.throttle,
        args.seconds,
        args.sim_hz / 1000.0,
        WARMUP_FRACTION * 100.0
    );
    println!(
        "Exhaust fundamental {:.1} Hz | averaged-pulse peak {:.0} Pa rel, p-p {:.0} Pa",
        fundamental_hz, averaged.peak_abs_pa, averaged.peak_to_peak_pa
    );
    println!(
        "Wrote {} ({:.2}s, {} Hz mono, {} samples)",
        args.out_path,
        args.wav_seconds,
        args.wav_rate,
        samples.len()
    );
}

struct AveragedCycle {
    /// Exhaust exit pressure relative to the open boundary, per angle bin.
    relative_pa: Vec<f64>,
    peak_abs_pa: f64,
    peak_to_peak_pa: f64,
}

fn render_average_cycle(definition: &EngineDefinition, args: &Args) -> AveragedCycle {
    let timestep_seconds = 1.0 / args.sim_hz;
    let profile = SimulationProfile {
        kind: SimulationProfileKind::Render,
        timestep_seconds,
        chamber_substeps: 1,
        ..SimulationProfile::render()
    };
    let mut engine =
        SingleCylinderEngine::from_definition_with_profile(definition.clone(), profile);

    let crank_speed_rad_per_s = rpm_to_rad_per_s(args.rpm);
    let total_steps = (args.seconds / timestep_seconds).ceil() as usize;
    let warmup_steps = (total_steps as f64 * WARMUP_FRACTION) as usize;

    let inputs = SingleCylinderStepInputs {
        fixed_crank_speed_rad_per_s: Some(crank_speed_rad_per_s),
        throttle_effective_area_fraction: Some(args.throttle.clamp(0.0, 1.0)),
        lambda_target: definition.combustion.lambda_target,
        ..SingleCylinderStepInputs::default()
    };

    let bins = args.bins.max(8);
    let mut sum_pa = vec![0.0f64; bins];
    let mut count = vec![0u64; bins];
    let exhaust_boundary_pa = definition.boundaries.exhaust_pressure_pa;

    for step in 0..total_steps {
        let output = engine.step(inputs);
        if step < warmup_steps {
            continue;
        }
        let phase = output.crank_angle_rad.rem_euclid(ENGINE_CYCLE_RADIANS) / ENGINE_CYCLE_RADIANS;
        let bin = ((phase * bins as f64) as usize).min(bins - 1);
        sum_pa[bin] += output.exhaust_exit_pressure_pa - exhaust_boundary_pa;
        count[bin] += 1;
    }

    let relative_pa = fill_and_average(&sum_pa, &count);
    let peak_abs_pa = relative_pa.iter().fold(0.0f64, |m, &v| m.max(v.abs()));
    let min = relative_pa.iter().copied().fold(f64::INFINITY, f64::min);
    let max = relative_pa
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max);

    AveragedCycle {
        relative_pa,
        peak_abs_pa,
        peak_to_peak_pa: (max - min).max(0.0),
    }
}

/// Average each bin, filling any empty bins from the nearest populated bin
/// (searching circularly, since the cycle wraps).
fn fill_and_average(sum_pa: &[f64], count: &[u64]) -> Vec<f64> {
    let bins = sum_pa.len();
    let mut averaged = vec![0.0f64; bins];
    let mut any = false;
    for index in 0..bins {
        if count[index] > 0 {
            averaged[index] = sum_pa[index] / count[index] as f64;
            any = true;
        }
    }
    if !any {
        return averaged;
    }
    for index in 0..bins {
        if count[index] == 0 {
            averaged[index] = nearest_populated(sum_pa, count, index);
        }
    }
    averaged
}

fn nearest_populated(sum_pa: &[f64], count: &[u64], index: usize) -> f64 {
    let bins = sum_pa.len();
    for offset in 1..=bins / 2 + 1 {
        let forward = (index + offset) % bins;
        if count[forward] > 0 {
            return sum_pa[forward] / count[forward] as f64;
        }
        let backward = (index + bins - offset % bins) % bins;
        if count[backward] > 0 {
            return sum_pa[backward] / count[backward] as f64;
        }
    }
    0.0
}

/// Resample the averaged single-cycle pulse into a looping time-domain waveform.
/// DC is removed (sound is the pressure fluctuation), then it is peak-normalised
/// to 90% full scale. Because the source is periodic and we sample by phase, the
/// loop is seamless regardless of the requested duration.
fn synthesize_wav_samples(averaged: &AveragedCycle, args: &Args) -> Vec<i16> {
    let bins = averaged.relative_pa.len();
    let mean: f64 = averaged.relative_pa.iter().sum::<f64>() / bins as f64;
    let peak = averaged
        .relative_pa
        .iter()
        .fold(0.0f64, |m, &v| m.max((v - mean).abs()));
    let scale = if peak > 0.0 { 0.9 / peak } else { 0.0 };

    let cycle_seconds = ENGINE_CYCLE_RADIANS / rpm_to_rad_per_s(args.rpm);
    let total_samples = ((args.wav_rate as f64) * args.wav_seconds).round() as usize;
    let mut samples = Vec::with_capacity(total_samples);

    for sample_index in 0..total_samples {
        let time_seconds = sample_index as f64 / args.wav_rate as f64;
        let phase = (time_seconds / cycle_seconds).rem_euclid(1.0);
        let position = phase * bins as f64;
        let low = position.floor() as usize % bins;
        let high = (low + 1) % bins;
        let fraction = position - position.floor();
        let interpolated =
            averaged.relative_pa[low] * (1.0 - fraction) + averaged.relative_pa[high] * fraction;
        let normalized = ((interpolated - mean) * scale).clamp(-1.0, 1.0);
        samples.push((normalized * i16::MAX as f64) as i16);
    }
    samples
}

fn write_wav_mono_i16(path: &str, sample_rate: u32, samples: &[i16]) -> std::io::Result<()> {
    let mut file = std::io::BufWriter::new(std::fs::File::create(path)?);
    let data_len = (samples.len() * 2) as u32;
    let byte_rate = sample_rate * 2;

    file.write_all(b"RIFF")?;
    file.write_all(&(36 + data_len).to_le_bytes())?;
    file.write_all(b"WAVE")?;
    file.write_all(b"fmt ")?;
    file.write_all(&16u32.to_le_bytes())?; // PCM fmt chunk size
    file.write_all(&1u16.to_le_bytes())?; // audio format: PCM
    file.write_all(&1u16.to_le_bytes())?; // channels: mono
    file.write_all(&sample_rate.to_le_bytes())?;
    file.write_all(&byte_rate.to_le_bytes())?;
    file.write_all(&2u16.to_le_bytes())?; // block align
    file.write_all(&16u16.to_le_bytes())?; // bits per sample
    file.write_all(b"data")?;
    file.write_all(&data_len.to_le_bytes())?;
    for &sample in samples {
        file.write_all(&sample.to_le_bytes())?;
    }
    file.flush()
}

fn parse_args() -> Result<Args, String> {
    let raw: Vec<String> = std::env::args().skip(1).collect();
    if raw.iter().any(|a| a == "-h" || a == "--help") {
        print_usage();
        std::process::exit(0);
    }

    let mut rpm: f64 = 3000.0;
    let mut throttle: f64 = 1.0;
    let mut seconds: f64 = 2.0;
    let mut sim_hz: f64 = 200_000.0;
    let mut bins = 2048usize;
    let mut wav_seconds: f64 = 2.0;
    let mut wav_rate = 44_100u32;
    let mut engine_path = DEFAULT_ENGINE_JSON.to_string();
    let mut out_path: Option<String> = None;

    let mut index = 0;
    while index < raw.len() {
        let key = raw[index].as_str();
        let value = raw.get(index + 1);
        let need = |value: Option<&String>| -> Result<String, String> {
            value
                .cloned()
                .ok_or_else(|| format!("missing value for {key}"))
        };
        match key {
            "--rpm" => rpm = need(value)?.parse().map_err(|_| "invalid --rpm")?,
            "--throttle" => throttle = need(value)?.parse().map_err(|_| "invalid --throttle")?,
            "--seconds" => seconds = need(value)?.parse().map_err(|_| "invalid --seconds")?,
            "--sim-hz" => sim_hz = need(value)?.parse().map_err(|_| "invalid --sim-hz")?,
            "--bins" => bins = need(value)?.parse().map_err(|_| "invalid --bins")?,
            "--wav-seconds" => {
                wav_seconds = need(value)?.parse().map_err(|_| "invalid --wav-seconds")?
            }
            "--wav-rate" => wav_rate = need(value)?.parse().map_err(|_| "invalid --wav-rate")?,
            "--engine" => engine_path = need(value)?,
            "--out" => out_path = Some(need(value)?),
            other => return Err(format!("unknown argument '{other}'")),
        }
        index += 2;
    }

    if !rpm.is_finite() || rpm <= 0.0 {
        return Err("--rpm must be positive".into());
    }
    if !(0.0..=1.0).contains(&throttle) {
        return Err("--throttle must be in 0.0..=1.0".into());
    }
    if !seconds.is_finite() || seconds <= 0.0 {
        return Err("--seconds must be positive".into());
    }
    if !sim_hz.is_finite() || sim_hz <= 0.0 {
        return Err("--sim-hz must be positive".into());
    }
    if !wav_seconds.is_finite() || wav_seconds <= 0.0 {
        return Err("--wav-seconds must be positive".into());
    }
    if wav_rate == 0 {
        return Err("--wav-rate must be positive".into());
    }

    let out_path = out_path.unwrap_or_else(|| format!("exhaust_{:.0}rpm.wav", rpm));

    Ok(Args {
        rpm,
        throttle,
        seconds,
        sim_hz,
        bins,
        wav_seconds,
        wav_rate,
        engine_path,
        out_path,
    })
}

fn print_usage() {
    eprintln!(
        "render_audio - average the steady-state exhaust pulse to a .wav\n\n\
         Options (defaults in brackets):\n\
         \x20 --rpm <rpm>            steady engine speed [3000]\n\
         \x20 --throttle <0..1>      throttle level, 1.0 = WOT [1.0]\n\
         \x20 --seconds <s>          simulated render time; first 10% discarded [2.0]\n\
         \x20 --sim-hz <hz>          core sim frequency (1/timestep) [200000]\n\
         \x20 --bins <n>             engine-angle bins per cycle [2048]\n\
         \x20 --wav-seconds <s>      length of the looped output clip [2.0]\n\
         \x20 --wav-rate <hz>        output sample rate [44100]\n\
         \x20 --engine <path.json>   engine definition [bundled gn250]\n\
         \x20 --out <file.wav>       output path [exhaust_<rpm>rpm.wav]"
    );
}
