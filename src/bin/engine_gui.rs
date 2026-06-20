use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Instant;

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use eframe::egui::{self, Color32, RichText};
use egui_plot::{Line, Plot, PlotPoints, Points};

use enginesim::engine_config::EngineDefinition;
use enginesim::engine_handling::EngineHandlingDefinition;
use enginesim::engine_loader::{
    EngineCatalogEntry, LoadSource, default_engine_directory, default_engine_path,
    discover_engine_files, load_engine_and_handling,
};
use enginesim::profiles::SimulationProfile;
use enginesim::single_cylinder::{
    SingleCylinderEngine, SingleCylinderStepOutput, rad_per_s_to_rpm,
};
use enginesim::telemetry::{
    EngineControls, EngineFrameTelemetry, TelemetryAggregator, TelemetryAvailability,
    default_profile_for_gui,
};

const SUMMARY_PANEL_HEIGHT: f32 = 52.0;
const METRIC_CARD_WIDTH: f32 = 150.0;
const METRIC_CARD_HEIGHT: f32 = 36.0;
const PLOT_ROW_SPACING: f32 = 12.0;
const MIN_TWO_COLUMN_WIDTH: f32 = 900.0;
const MIN_THREE_COLUMN_WIDTH: f32 = 1_550.0;
const AUDIO_CAPTURE_WINDOW_SECONDS: f64 = 0.100;
const AUDIO_PRESSURE_SCALE_PA: f64 = 20_000.0;

struct RealtimeAudio {
    shared: Arc<Mutex<AudioShared>>,
    _stream: Option<cpal::Stream>,
    enabled: bool,
    exhaust_gain: f32,
    intake_gain: f32,
    status: String,
}

impl RealtimeAudio {
    fn new() -> Self {
        let host = cpal::default_host();
        let device = host.default_output_device();
        let Some(device) = device else {
            return Self::disabled("audio: no default output device");
        };
        let config = match device.default_output_config() {
            Ok(config) => config,
            Err(err) => return Self::disabled(format!("audio: no default output config ({err})")),
        };

        let sample_rate_hz = config.sample_rate().0 as f64;
        let channels = config.channels() as usize;
        let shared = Arc::new(Mutex::new(AudioShared::new(sample_rate_hz)));
        let err_fn = |err| eprintln!("engine_gui audio stream error: {err}");
        let stream_config: cpal::StreamConfig = config.clone().into();
        let stream = match config.sample_format() {
            cpal::SampleFormat::F32 => build_output_stream::<f32>(
                &device,
                &stream_config,
                channels,
                shared.clone(),
                err_fn,
            ),
            cpal::SampleFormat::I16 => build_output_stream::<i16>(
                &device,
                &stream_config,
                channels,
                shared.clone(),
                err_fn,
            ),
            cpal::SampleFormat::U16 => build_output_stream::<u16>(
                &device,
                &stream_config,
                channels,
                shared.clone(),
                err_fn,
            ),
            _ => Err(cpal::BuildStreamError::StreamConfigNotSupported),
        };

        let Ok(stream) = stream else {
            return Self::disabled("audio: output stream unsupported");
        };
        if let Err(err) = stream.play() {
            return Self::disabled(format!("audio: stream did not start ({err})"));
        }

        Self {
            shared,
            _stream: Some(stream),
            enabled: false,
            exhaust_gain: 0.4,
            intake_gain: 0.2,
            status: format!("audio: ready at {:.0} Hz", sample_rate_hz),
        }
    }

    fn disabled(status: impl Into<String>) -> Self {
        Self {
            shared: Arc::new(Mutex::new(AudioShared::new(48_000.0))),
            _stream: None,
            enabled: false,
            exhaust_gain: 0.0,
            intake_gain: 0.0,
            status: status.into(),
        }
    }

    fn capture_step_output(
        &mut self,
        output: SingleCylinderStepOutput,
        definition: &EngineDefinition,
        real_time_mode: bool,
    ) {
        if !real_time_mode || !self.enabled {
            return;
        }

        let exhaust_rel_pa =
            output.exhaust_exit_pressure_pa - definition.boundaries.exhaust_pressure_pa;
        let intake_rel_pa =
            definition.boundaries.intake_pressure_pa - output.intake_plenum_pressure_pa;
        if let Ok(mut shared) = self.shared.lock() {
            shared.capture(output.elapsed_time_seconds, exhaust_rel_pa, intake_rel_pa);
        }
    }

    fn reset_capture(&mut self) {
        if let Ok(mut shared) = self.shared.lock() {
            shared.reset_capture();
        }
    }

    fn sync_controls(&mut self) {
        if let Ok(mut shared) = self.shared.lock() {
            shared.enabled = self.enabled;
            shared.exhaust_gain = self.exhaust_gain;
            shared.intake_gain = self.intake_gain;
        }
    }

    fn status(&self) -> &str {
        &self.status
    }
}

struct AudioShared {
    sample_rate_hz: f64,
    exhaust_buffer: Vec<f32>,
    intake_buffer: Vec<f32>,
    write_index: usize,
    read_index: usize,
    available_samples: usize,
    last_time_seconds: Option<f64>,
    next_sample_time_seconds: f64,
    last_exhaust_sample: f32,
    last_intake_sample: f32,
    enabled: bool,
    exhaust_gain: f32,
    intake_gain: f32,
}

impl AudioShared {
    fn new(sample_rate_hz: f64) -> Self {
        let capacity = ((sample_rate_hz * AUDIO_CAPTURE_WINDOW_SECONDS).round() as usize).max(1);
        Self {
            sample_rate_hz,
            exhaust_buffer: vec![0.0; capacity],
            intake_buffer: vec![0.0; capacity],
            write_index: 0,
            read_index: 0,
            available_samples: 0,
            last_time_seconds: None,
            next_sample_time_seconds: 0.0,
            last_exhaust_sample: 0.0,
            last_intake_sample: 0.0,
            enabled: false,
            exhaust_gain: 0.0,
            intake_gain: 0.0,
        }
    }

    fn reset_capture(&mut self) {
        self.write_index = 0;
        self.read_index = 0;
        self.available_samples = 0;
        self.last_time_seconds = None;
        self.next_sample_time_seconds = 0.0;
        self.last_exhaust_sample = 0.0;
        self.last_intake_sample = 0.0;
        self.exhaust_buffer.fill(0.0);
        self.intake_buffer.fill(0.0);
    }

    fn capture(&mut self, time_seconds: f64, exhaust_rel_pa: f64, intake_rel_pa: f64) {
        let exhaust_sample = pressure_sample(exhaust_rel_pa);
        let intake_sample = pressure_sample(intake_rel_pa);
        let Some(last_time_seconds) = self.last_time_seconds else {
            self.last_time_seconds = Some(time_seconds);
            self.next_sample_time_seconds = time_seconds;
            self.last_exhaust_sample = exhaust_sample;
            self.last_intake_sample = intake_sample;
            return;
        };

        if time_seconds <= last_time_seconds {
            self.last_time_seconds = Some(time_seconds);
            self.next_sample_time_seconds = time_seconds;
            self.last_exhaust_sample = exhaust_sample;
            self.last_intake_sample = intake_sample;
            return;
        }

        let sample_period_seconds = 1.0 / self.sample_rate_hz;
        let mut emitted = 0usize;
        let max_emitted = self.exhaust_buffer.len();
        while self.next_sample_time_seconds <= time_seconds && emitted < max_emitted {
            let fraction = ((self.next_sample_time_seconds - last_time_seconds)
                / (time_seconds - last_time_seconds))
                .clamp(0.0, 1.0) as f32;
            let exhaust =
                self.last_exhaust_sample + (exhaust_sample - self.last_exhaust_sample) * fraction;
            let intake =
                self.last_intake_sample + (intake_sample - self.last_intake_sample) * fraction;
            self.push_sample(exhaust, intake);
            self.next_sample_time_seconds += sample_period_seconds;
            emitted += 1;
        }

        self.last_time_seconds = Some(time_seconds);
        self.last_exhaust_sample = exhaust_sample;
        self.last_intake_sample = intake_sample;
    }

    fn push_sample(&mut self, exhaust: f32, intake: f32) {
        if self.available_samples == self.exhaust_buffer.len() {
            self.read_index = (self.read_index + 1) % self.exhaust_buffer.len();
            self.available_samples -= 1;
        }
        self.exhaust_buffer[self.write_index] = exhaust;
        self.intake_buffer[self.write_index] = intake;
        self.write_index = (self.write_index + 1) % self.exhaust_buffer.len();
        self.available_samples += 1;
    }

    fn next_output_sample(&mut self) -> f32 {
        if !self.enabled || self.available_samples == 0 {
            return 0.0;
        }
        let sample = self.exhaust_buffer[self.read_index] * self.exhaust_gain
            + self.intake_buffer[self.read_index] * self.intake_gain;
        self.read_index = (self.read_index + 1) % self.exhaust_buffer.len();
        self.available_samples -= 1;
        sample.clamp(-1.0, 1.0)
    }
}

fn pressure_sample(relative_pressure_pa: f64) -> f32 {
    (relative_pressure_pa / AUDIO_PRESSURE_SCALE_PA).tanh() as f32
}

trait AudioSample: cpal::SizedSample {
    fn from_unit_sample(value: f32) -> Self;
}

impl AudioSample for f32 {
    fn from_unit_sample(value: f32) -> Self {
        value
    }
}

impl AudioSample for i16 {
    fn from_unit_sample(value: f32) -> Self {
        (value.clamp(-1.0, 1.0) * i16::MAX as f32) as i16
    }
}

impl AudioSample for u16 {
    fn from_unit_sample(value: f32) -> Self {
        ((value.clamp(-1.0, 1.0) * 0.5 + 0.5) * u16::MAX as f32) as u16
    }
}

fn build_output_stream<T>(
    device: &cpal::Device,
    config: &cpal::StreamConfig,
    channels: usize,
    shared: Arc<Mutex<AudioShared>>,
    err_fn: impl FnMut(cpal::StreamError) + Send + 'static,
) -> Result<cpal::Stream, cpal::BuildStreamError>
where
    T: AudioSample,
{
    device.build_output_stream(
        config,
        move |data: &mut [T], _: &cpal::OutputCallbackInfo| {
            write_audio_data(data, channels, &shared);
        },
        err_fn,
        None,
    )
}

fn write_audio_data<T>(data: &mut [T], channels: usize, shared: &Arc<Mutex<AudioShared>>)
where
    T: AudioSample,
{
    if let Ok(mut shared) = shared.try_lock() {
        for frame in data.chunks_mut(channels) {
            let sample = T::from_unit_sample(shared.next_output_sample());
            for output in frame {
                *output = sample;
            }
        }
    } else {
        for output in data {
            *output = T::from_unit_sample(0.0);
        }
    }
}

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "EngineSim GUI",
        options,
        Box::new(|_cc| Ok(Box::new(EngineGuiApp::new()))),
    )
}

struct EngineGuiApp {
    engine_path: PathBuf,
    engine_options: Vec<EngineCatalogEntry>,
    definition: EngineDefinition,
    handling: EngineHandlingDefinition,
    profile: SimulationProfile,
    engine: SingleCylinderEngine,
    controls: EngineControls,
    telemetry: TelemetryAggregator,
    latest_frame: EngineFrameTelemetry,
    audio: RealtimeAudio,
    running: bool,
    redline_cut_remaining_seconds: f64,
    last_wall_clock: Instant,
    load_status: String,
}

fn bundled_definition() -> EngineDefinition {
    EngineDefinition::from_json_str(include_str!("../../data/engines/gn250.json"))
        .expect("bundled GN250 JSON should parse")
}

fn bundled_handling() -> EngineHandlingDefinition {
    EngineHandlingDefinition::from_json_str(include_str!("../../data/engines/gn250.handling.json"))
        .expect("bundled GN250 handling JSON should parse")
}

fn engine_options_or_default() -> Vec<EngineCatalogEntry> {
    match discover_engine_files(&default_engine_directory()) {
        Ok(entries) if !entries.is_empty() => entries,
        _ => vec![EngineCatalogEntry::from_path(default_engine_path())],
    }
}

fn initial_engine_path(options: &[EngineCatalogEntry]) -> PathBuf {
    let default_path = default_engine_path();
    options
        .iter()
        .find(|entry| entry.path == default_path)
        .or_else(|| options.first())
        .map(|entry| entry.path.clone())
        .unwrap_or(default_path)
}

fn describe_source(label: &str, source: &LoadSource) -> String {
    match source {
        LoadSource::Disk(path) => format!("{label}: {}", path.display()),
        LoadSource::BundledFallback { .. } => format!("{label}: bundled default"),
    }
}

fn load_status(engine_source: &LoadSource, handling_source: &LoadSource) -> String {
    format!(
        "{} | {}",
        describe_source("engine", engine_source),
        describe_source("handling", handling_source)
    )
}

impl EngineGuiApp {
    fn new() -> Self {
        let engine_options = engine_options_or_default();
        let engine_path = initial_engine_path(&engine_options);
        let loaded =
            load_engine_and_handling(&engine_path, &bundled_definition(), &bundled_handling());
        let load_status = load_status(&loaded.engine_source, &loaded.handling_source);
        let profile = default_profile_for_gui(&loaded.handling);
        let mut controls = EngineControls::default();
        align_controls_to_engine(&mut controls, &loaded.definition, &loaded.handling);
        let engine =
            SingleCylinderEngine::from_definition_with_profile(loaded.definition.clone(), profile);
        let telemetry = TelemetryAggregator::new(loaded.definition.clone(), controls);
        let latest_frame = telemetry.snapshot();
        let audio = RealtimeAudio::new();

        Self {
            engine_path,
            engine_options,
            definition: loaded.definition,
            handling: loaded.handling,
            profile,
            engine,
            controls,
            telemetry,
            latest_frame,
            audio,
            running: true,
            redline_cut_remaining_seconds: 0.0,
            last_wall_clock: Instant::now(),
            load_status,
        }
    }

    fn load_engine(&mut self, engine_path: PathBuf) {
        let loaded =
            load_engine_and_handling(&engine_path, &bundled_definition(), &bundled_handling());
        self.engine_path = engine_path;
        self.load_status = load_status(&loaded.engine_source, &loaded.handling_source);
        self.definition = loaded.definition;
        self.handling = loaded.handling;
        self.profile = match self.profile.kind {
            enginesim::profiles::SimulationProfileKind::RealTime => {
                default_profile_for_gui(&self.handling)
            }
            enginesim::profiles::SimulationProfileKind::Render => SimulationProfile::render(),
        };
        align_controls_to_engine(&mut self.controls, &self.definition, &self.handling);
        self.reset_engine();
    }

    fn selected_engine_label(&self) -> String {
        self.engine_options
            .iter()
            .find(|entry| entry.path == self.engine_path)
            .map(|entry| entry.label.clone())
            .unwrap_or_else(|| self.definition.metadata.name.clone())
    }

    fn engine_selector(&mut self, ui: &mut egui::Ui) {
        ui.label("Engine");
        let mut selected_path = self.engine_path.clone();
        egui::ComboBox::from_id_salt("engine_selector")
            .selected_text(self.selected_engine_label())
            .show_ui(ui, |ui| {
                for entry in &self.engine_options {
                    ui.selectable_value(&mut selected_path, entry.path.clone(), &entry.label);
                }
            });
        if selected_path != self.engine_path {
            self.load_engine(selected_path);
        }
        ui.small(self.load_status.clone());
    }

    fn reset_engine(&mut self) {
        self.engine = SingleCylinderEngine::from_definition_with_profile(
            self.definition.clone(),
            self.profile,
        );
        self.telemetry = TelemetryAggregator::new(self.definition.clone(), self.controls);
        self.latest_frame = self.telemetry.snapshot();
        self.audio.reset_capture();
        self.redline_cut_remaining_seconds = 0.0;
        self.last_wall_clock = Instant::now();
    }

    fn simulate_until_now(&mut self) {
        let now = Instant::now();
        let mut wall_delta_seconds = (now - self.last_wall_clock).as_secs_f64();
        self.last_wall_clock = now;
        if !self.running {
            return;
        }

        wall_delta_seconds = wall_delta_seconds.clamp(0.0, 0.05);
        let timestep_seconds = self.profile.timestep_seconds;
        let steps = (wall_delta_seconds / timestep_seconds).ceil() as usize;

        self.telemetry.set_controls(self.controls);
        for _ in 0..steps.max(1) {
            let current_rpm = rad_per_s_to_rpm(self.engine.crank_speed_rad_per_s());
            if current_rpm >= self.controls.redline_spark_cut_rpm {
                self.redline_cut_remaining_seconds =
                    self.handling.redline_cut_time_seconds.max(0.0);
            }
            let redline_cut_active = self.redline_cut_remaining_seconds > 0.0;
            let output = self
                .engine
                .step(self.controls.to_step_inputs_with_redline_cut_active(
                    &self.definition,
                    current_rpm,
                    redline_cut_active,
                ));
            self.audio.capture_step_output(
                output,
                &self.definition,
                self.profile.kind == enginesim::profiles::SimulationProfileKind::RealTime,
            );
            self.telemetry.ingest_step_output(output, timestep_seconds);
            self.redline_cut_remaining_seconds =
                (self.redline_cut_remaining_seconds - timestep_seconds).max(0.0);
            if let Some(frame) = self.telemetry.publish_ready() {
                self.latest_frame = frame;
            }
        }
    }
}

fn align_controls_to_engine(
    controls: &mut EngineControls,
    definition: &EngineDefinition,
    handling: &EngineHandlingDefinition,
) {
    controls.lambda_target = definition.combustion.lambda_target.clamp(0.1, 2.0);
    controls.dyno_target_rpm = definition.crank.initial_speed_rpm.max(1.0);
    controls.added_inertia_kg_m2 = controls
        .added_inertia_kg_m2
        .clamp(0.0, handling.max_added_inertia_kg_m2.max(0.0));
}

impl eframe::App for EngineGuiApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.simulate_until_now();

        egui::SidePanel::left("controls")
            .resizable(true)
            .default_width(260.0)
            .show(ctx, |ui| {
                ui.heading("Controls");
                self.engine_selector(ui);
                ui.separator();
                ui.checkbox(&mut self.running, "Run");
                if ui.button("Reset").clicked() {
                    self.reset_engine();
                }
                ui.separator();
                ui.label("Profile");
                let mut profile_kind = self.profile.kind;
                egui::ComboBox::from_label("")
                    .selected_text(format!("{profile_kind:?}"))
                    .show_ui(ui, |ui| {
                        ui.selectable_value(
                            &mut profile_kind,
                            enginesim::profiles::SimulationProfileKind::RealTime,
                            "RealTime",
                        );
                        ui.selectable_value(
                            &mut profile_kind,
                            enginesim::profiles::SimulationProfileKind::Render,
                            "Render",
                        );
                    });
                if profile_kind != self.profile.kind {
                    self.profile = match profile_kind {
                        enginesim::profiles::SimulationProfileKind::RealTime => {
                            default_profile_for_gui(&self.handling)
                        }
                        enginesim::profiles::SimulationProfileKind::Render => {
                            SimulationProfile::render()
                        }
                    };
                    self.reset_engine();
                }
                ui.separator();
                ui.add(
                    egui::Slider::new(&mut self.controls.throttle_position, 0.0..=1.0)
                        .text("Throttle")
                        .suffix(" frac"),
                );
                ui.add(
                    egui::Slider::new(&mut self.controls.idle_throttle_fraction, 0.0..=1.0)
                        .text("Idle Throttle")
                        .suffix(" frac"),
                );
                ui.add(
                    egui::Slider::new(&mut self.controls.lambda_target, 0.1..=2.0)
                        .text("Lambda Target"),
                );
                ui.separator();
                ui.checkbox(&mut self.controls.starter_enabled, "Starter");
                ui.add(
                    egui::Slider::new(&mut self.controls.starter_torque_nm, 0.0..=120.0)
                        .text("Starter Torque")
                        .suffix(" Nm"),
                );
                ui.add(
                    egui::Slider::new(&mut self.controls.starter_speed_limit_rpm, 0.0..=3000.0)
                        .text("Starter Limit")
                        .suffix(" rpm"),
                );
                ui.add(
                    egui::Slider::new(
                        &mut self.controls.added_inertia_kg_m2,
                        0.0..=self.handling.max_added_inertia_kg_m2.max(0.0),
                    )
                    .text("Added Inertia")
                    .suffix(" kg m^2"),
                );
                ui.add(
                    egui::Slider::new(&mut self.controls.added_torque_load_nm, 0.0..=40.0)
                        .text("Load")
                        .suffix(" Nm"),
                );
                ui.checkbox(&mut self.controls.spark_enabled, "Spark");
                ui.checkbox(&mut self.controls.fuel_enabled, "Fuel");
                ui.add(
                    egui::Slider::new(&mut self.controls.redline_spark_cut_rpm, 2000.0..=12000.0)
                        .text("Spark Cut")
                        .suffix(" rpm"),
                );
                ui.separator();
                ui.checkbox(&mut self.audio.enabled, "Audio");
                ui.add(
                    egui::Slider::new(&mut self.audio.exhaust_gain, 0.0..=8.0).text("Exhaust Gain"),
                );
                ui.add(
                    egui::Slider::new(&mut self.audio.intake_gain, 0.0..=8.0).text("Intake Gain"),
                );
                self.audio.sync_controls();
                ui.small(self.audio.status());
                ui.separator();
                ui.checkbox(&mut self.controls.dyno_mode_enabled, "Dyno Mode");
                ui.add(
                    egui::Slider::new(&mut self.controls.dyno_target_rpm, 1.0..=9000.0)
                        .text("Dyno Target")
                        .suffix(" rpm"),
                );
                ui.separator();
                ui.small("Throttle and idle throttle feed the intake plenum.");
            });

        egui::TopBottomPanel::top("summary")
            .resizable(false)
            .exact_height(SUMMARY_PANEL_HEIGHT)
            .show(ctx, |ui| {
                egui::ScrollArea::horizontal()
                    .id_salt("summary_scroll")
                    .auto_shrink([false, true])
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            metric(
                                ui,
                                "RPM",
                                format!("{:.0}", self.latest_frame.engine_data.rpm),
                            );
                            metric(
                                ui,
                                "RPM Delta",
                                format!("{:.1}", self.latest_frame.engine_data.rpm_delta),
                            );
                            metric(
                                ui,
                                "Torque",
                                format!("{:.2} Nm", self.latest_frame.engine_data.torque_nm),
                            );
                            metric(
                                ui,
                                "Power",
                                format!("{:.2} kW", self.latest_frame.engine_data.power_kw),
                            );
                            metric(
                                ui,
                                "Fuel Flow",
                                format!(
                                    "{:.1} mg/s",
                                    self.latest_frame.engine_data.fuel_flow_mg_per_s
                                ),
                            );
                            metric(
                                ui,
                                "Air Flow",
                                format!(
                                    "{:.2} g/s",
                                    self.latest_frame.engine_data.air_flow_g_per_s
                                ),
                            );
                            metric(
                                ui,
                                "BMEP Est",
                                format!("{:.0} kPa", self.latest_frame.engine_data.bmep_kpa),
                            );
                            metric(
                                ui,
                                "VE",
                                format!(
                                    "{:.0} %",
                                    self.latest_frame.engine_data.volumetric_efficiency_percent
                                ),
                            );
                            scalar_metric(
                                ui,
                                "Chamber Lambda",
                                self.latest_frame.engine_data.chamber_lambda,
                            );
                            scalar_metric(
                                ui,
                                "Exhaust Lambda",
                                self.latest_frame.engine_data.exhaust_lambda,
                            );
                            metric(
                                ui,
                                "Peak Temp",
                                format!(
                                    "{:.0} C",
                                    kelvin_to_celsius(
                                        self.latest_frame.engine_data.peak_cylinder_temperature_k
                                    )
                                ),
                            );
                            metric(
                                ui,
                                "Peak Pressure",
                                format!(
                                    "{:.0} kPa rel",
                                    relative_kpa(
                                        self.latest_frame.engine_data.peak_cylinder_pressure_pa,
                                        self.latest_frame.engine_data.ambient_pressure_pa,
                                    )
                                ),
                            );
                            metric(
                                ui,
                                "MAP",
                                format!("{:.0} kPa", self.latest_frame.engine_data.map_pa / 1000.0),
                            );
                            metric(
                                ui,
                                "Combustion Ratio",
                                format!(
                                    "{:.2}",
                                    self.latest_frame.engine_data.combustion_event_ratio
                                ),
                            );
                        });
                    });
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            plot_dashboard(ui, &self.latest_frame);
        });

        ctx.request_repaint();
    }
}

fn metric(ui: &mut egui::Ui, label: &str, value: String) {
    ui.scope(|ui| {
        ui.set_width(METRIC_CARD_WIDTH);
        ui.group(|ui| {
            ui.set_min_size(egui::vec2(METRIC_CARD_WIDTH - 16.0, METRIC_CARD_HEIGHT));
            ui.label(RichText::new(label).small());
            ui.label(RichText::new(value).strong());
        });
    });
}

fn scalar_metric(ui: &mut egui::Ui, label: &str, scalar: enginesim::telemetry::TelemetryScalar) {
    let suffix = match scalar.availability {
        TelemetryAvailability::Measured => "",
        TelemetryAvailability::Placeholder => " placeholder",
        TelemetryAvailability::Unavailable => " unavailable",
    };
    let text = scalar
        .value
        .map(|value| {
            if value.is_infinite() {
                format!("lean{suffix}")
            } else {
                format!("{value:.2}{suffix}")
            }
        })
        .unwrap_or_else(|| suffix.trim().to_string());
    metric(ui, label, text);
}

#[derive(Debug, Clone, Copy)]
enum PlotPanel {
    CylinderPressure,
    CylinderTemperature,
    ValveEffectiveArea,
    ChamberMass,
    ExhaustExitPressureAngle,
    RpmTime,
    TorqueTime,
    MapTime,
    ExhaustExitPressureTime,
    TorqueRpm,
    PowerRpm,
}

const PLOT_PANELS: [PlotPanel; 11] = [
    PlotPanel::CylinderPressure,
    PlotPanel::CylinderTemperature,
    PlotPanel::ValveEffectiveArea,
    PlotPanel::ChamberMass,
    PlotPanel::ExhaustExitPressureAngle,
    PlotPanel::RpmTime,
    PlotPanel::TorqueTime,
    PlotPanel::MapTime,
    PlotPanel::ExhaustExitPressureTime,
    PlotPanel::TorqueRpm,
    PlotPanel::PowerRpm,
];

fn plot_dashboard(ui: &mut egui::Ui, telemetry: &EngineFrameTelemetry) {
    let available_width = ui.available_width();
    let column_count = if available_width >= MIN_THREE_COLUMN_WIDTH {
        3
    } else if available_width >= MIN_TWO_COLUMN_WIDTH {
        2
    } else {
        1
    };

    egui::ScrollArea::vertical()
        .id_salt("plot_dashboard_scroll")
        .auto_shrink([false, false])
        .show(ui, |ui| {
            for row in PLOT_PANELS.chunks(column_count) {
                ui.columns(column_count, |columns| {
                    for (column_index, panel) in row.iter().enumerate() {
                        render_plot_panel(&mut columns[column_index], *panel, telemetry);
                    }
                });
                ui.add_space(PLOT_ROW_SPACING);
            }
        });
}

fn render_plot_panel(ui: &mut egui::Ui, panel: PlotPanel, telemetry: &EngineFrameTelemetry) {
    match panel {
        PlotPanel::CylinderPressure => angle_plot(
            ui,
            "Cylinder Pressure",
            "Pressure (kPa rel)",
            &[(
                "Pressure",
                &relative_pressure_points(
                    &telemetry.angle_trace.pressure_pa,
                    telemetry.engine_data.ambient_pressure_pa,
                ),
                Color32::LIGHT_RED,
            )],
        ),
        PlotPanel::CylinderTemperature => angle_plot(
            ui,
            "Cylinder Temperature",
            "Temperature (C)",
            &[(
                "Temperature",
                &celsius_points(&telemetry.angle_trace.temperature_k),
                Color32::LIGHT_RED,
            )],
        ),
        PlotPanel::ValveEffectiveArea => angle_plot(
            ui,
            "Valve Effective Area",
            "Effective Area (m^2)",
            &[
                (
                    "Intake",
                    &telemetry.angle_trace.intake_effective_area_m2,
                    Color32::LIGHT_RED,
                ),
                (
                    "Exhaust",
                    &telemetry.angle_trace.exhaust_effective_area_m2,
                    Color32::LIGHT_BLUE,
                ),
            ],
        ),
        PlotPanel::ChamberMass => angle_plot(
            ui,
            "Charge Air vs Fuel x AFR",
            "Mass (g)",
            &[
                (
                    "Air mass",
                    &telemetry.angle_trace.chamber_air_mass_g,
                    Color32::LIGHT_RED,
                ),
                (
                    "Fuel x AFR",
                    &telemetry.angle_trace.chamber_fuel_scaled_g,
                    Color32::LIGHT_YELLOW,
                ),
            ],
        ),
        PlotPanel::ExhaustExitPressureAngle => angle_plot(
            ui,
            "Exhaust Exit Pressure",
            "Pressure (kPa rel)",
            &[(
                "Pressure",
                &relative_pressure_points(
                    &telemetry.angle_trace.exhaust_exit_pressure_pa,
                    telemetry.engine_data.ambient_pressure_pa,
                ),
                Color32::from_rgb(230, 150, 110),
            )],
        ),
        PlotPanel::RpmTime => time_plot(
            ui,
            "RPM Over Time",
            "RPM",
            &telemetry.time_history,
            |point| [point.time_seconds, point.rpm],
        ),
        PlotPanel::TorqueTime => time_plot(
            ui,
            "Torque Over Time",
            "Torque (Nm)",
            &telemetry.time_history,
            |point| [point.time_seconds, point.torque_nm],
        ),
        PlotPanel::MapTime => time_plot(
            ui,
            "MAP Over Time",
            "MAP (kPa)",
            &telemetry.time_history,
            |point| [point.time_seconds, point.map_pa / 1000.0],
        ),
        PlotPanel::ExhaustExitPressureTime => time_plot(
            ui,
            "Exhaust Exit Pressure Over Time",
            "Pressure (kPa rel)",
            &telemetry.time_history,
            |point| {
                [
                    point.time_seconds,
                    relative_kpa(
                        point.exhaust_exit_pressure_pa,
                        telemetry.engine_data.ambient_pressure_pa,
                    ),
                ]
            },
        ),
        PlotPanel::TorqueRpm => rpm_history_plot(
            ui,
            "Cycle Mean Torque vs RPM",
            "Torque (Nm)",
            telemetry,
            |point| point.torque_nm,
            Color32::from_rgb(100, 200, 255),
        ),
        PlotPanel::PowerRpm => rpm_history_plot(
            ui,
            "Cycle Mean Power vs RPM",
            "Power (kW)",
            telemetry,
            |point| point.power_kw,
            Color32::from_rgb(255, 180, 80),
        ),
    }
}

fn angle_plot(
    ui: &mut egui::Ui,
    title: &str,
    y_label: &str,
    series: &[(&str, &Vec<[f64; 2]>, Color32)],
) {
    plot_title(ui, title);
    Plot::new(format!("{title}_plot"))
        .height(plot_height(ui.available_width()))
        .x_axis_label("Crank Angle (deg)")
        .y_axis_label(y_label)
        .default_x_bounds(0.0, 720.0)
        .allow_drag(false)
        .allow_zoom(false)
        .show(ui, |plot_ui| {
            plot_ui.set_plot_bounds_x(0.0..=720.0);
            for (label, points, color) in series {
                plot_ui.line(Line::new(*label, PlotPoints::from((*points).clone())).color(*color));
            }
        });
}

fn time_plot(
    ui: &mut egui::Ui,
    title: &str,
    y_label: &str,
    history: &[enginesim::telemetry::TimePlotPoint],
    map: impl Fn(&enginesim::telemetry::TimePlotPoint) -> [f64; 2],
) {
    plot_title(ui, title);
    let points: Vec<[f64; 2]> = history.iter().map(map).collect();
    Plot::new(format!("{title}_plot"))
        .height(plot_height(ui.available_width()))
        .x_axis_label("Time (s)")
        .y_axis_label(y_label)
        .include_y(0.0)
        .show(ui, |plot_ui| {
            plot_ui.line(Line::new(title, PlotPoints::from(points)));
        });
}

fn rpm_history_plot(
    ui: &mut egui::Ui,
    title: &str,
    y_label: &str,
    telemetry: &EngineFrameTelemetry,
    map: impl Fn(&enginesim::telemetry::RpmPlotPoint) -> f64,
    color: Color32,
) {
    plot_title(ui, title);
    let points: Vec<[f64; 2]> = telemetry
        .rpm_history
        .iter()
        .map(|point| [point.rpm, map(point)])
        .collect();
    Plot::new(format!("{title}_plot"))
        .height(plot_height(ui.available_width()))
        .x_axis_label("RPM")
        .y_axis_label(y_label)
        .include_y(0.0)
        .show(ui, |plot_ui| {
            plot_ui.line(Line::new(title, PlotPoints::from(points.clone())).color(color));
            plot_ui.points(
                Points::new(title, PlotPoints::from(points))
                    .radius(2.5)
                    .color(color),
            );
        });
}

fn plot_title(ui: &mut egui::Ui, title: &str) {
    ui.label(RichText::new(title).strong());
}

fn plot_height(available_width: f32) -> f32 {
    (available_width / 2.35).clamp(185.0, 310.0)
}

fn relative_kpa(pressure_pa: f64, ambient_pressure_pa: f64) -> f64 {
    (pressure_pa - ambient_pressure_pa) / 1000.0
}

fn kelvin_to_celsius(temperature_k: f64) -> f64 {
    temperature_k - 273.15
}

fn relative_pressure_points(points: &[[f64; 2]], ambient_pressure_pa: f64) -> Vec<[f64; 2]> {
    points
        .iter()
        .map(|point| [point[0], relative_kpa(point[1], ambient_pressure_pa)])
        .collect()
}

fn celsius_points(points: &[[f64; 2]]) -> Vec<[f64; 2]> {
    points
        .iter()
        .map(|point| [point[0], kelvin_to_celsius(point[1])])
        .collect()
}
