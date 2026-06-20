//! Tuning GUI: a dedicated front-end for tuning the engine.
//!
//! Built with the same techniques as `engine_gui` (eframe app over the library
//! crate, telemetry aggregation, egui_plot), but focused on tuning. It exposes
//! the full tunable-parameter set from `design_information/Features.MD`:
//! per-runner geometry, compression ratio, valve timing, throttle body, plenum,
//! Wiebe/combustion/spark parameters, a direct MAP override, set-point snapping,
//! the engine-angle plots and the data boxes.
//!
//! Engine-defining parameters are edited on a draft and applied with
//! "Write Changes" (which resets the sim — not a recompile). Operating inputs
//! (throttle, RPM, lambda, MAP override) take effect live.

use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::PathBuf;
use std::time::Instant;

use eframe::egui::{self, Color32, DragValue, RichText};
use egui_plot::{Line, Plot, PlotPoints};

use enginesim::engine_config::{EngineDefinition, PipeDefinition, ValveDefinition};
use enginesim::engine_handling::{EngineHandlingDefinition, nearest_set_point_rpm};
use enginesim::engine_loader::{LoadSource, load_engine_and_handling, save_engine_and_handling};
use enginesim::profiles::SimulationProfile;
use enginesim::single_cylinder::{SingleCylinderEngine, rad_per_s_to_rpm};
use enginesim::telemetry::{
    EngineControls, EngineFrameTelemetry, MAP_OVERRIDE_MAX_PA, TelemetryAggregator,
    TelemetryAvailability, TelemetryScalar, default_profile_for_gui,
};
use enginesim::tuning::{TuningSession, area_m2_from_diameter_mm, diameter_mm_from_area_m2};

const ENGINE_JSON_PATH: &str = "data/engines/gn250.json";
const ENGINE_CYCLE_DEG: f64 = 720.0;

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "EngineSim Tuning GUI",
        options,
        Box::new(|_cc| Ok(Box::new(TuningGuiApp::new()))),
    )
}

/// Whether crank-angle entry boxes show absolute engine position (default) or
/// the conventional degrees-before-top-dead-centre.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AngleMode {
    Absolute,
    Btdc,
}

struct TuningGuiApp {
    engine_path: PathBuf,
    session: TuningSession,
    profile: SimulationProfile,
    engine: SingleCylinderEngine,
    controls: EngineControls,
    telemetry: TelemetryAggregator,
    latest_frame: EngineFrameTelemetry,
    running: bool,
    redline_cut_remaining_seconds: f64,
    last_wall_clock: Instant,
    load_status: String,
    angle_mode: AngleMode,
    /// Set when a simulation step panicked (e.g. the 1D solver hit a
    /// non-positive pressure after an aggressive parameter change). The sim is
    /// paused and the engine state is considered poisoned until the next reset.
    diverged: Option<String>,
}

fn bundled_definition() -> EngineDefinition {
    EngineDefinition::from_json_str(include_str!("../../data/engines/gn250.json"))
        .expect("bundled GN250 JSON should parse")
}

fn bundled_handling() -> EngineHandlingDefinition {
    EngineHandlingDefinition::from_json_str(include_str!("../../data/engines/gn250.handling.json"))
        .expect("bundled GN250 handling JSON should parse")
}

fn describe_source(label: &str, source: &LoadSource) -> String {
    match source {
        LoadSource::Disk(path) => format!("{label}: {}", path.display()),
        LoadSource::BundledFallback { .. } => format!("{label}: bundled default"),
    }
}

impl TuningGuiApp {
    fn new() -> Self {
        let engine_path = PathBuf::from(ENGINE_JSON_PATH);
        let loaded =
            load_engine_and_handling(&engine_path, &bundled_definition(), &bundled_handling());
        let load_status = format!(
            "{} | {}",
            describe_source("engine", &loaded.engine_source),
            describe_source("handling", &loaded.handling_source)
        );
        let session = TuningSession::new(loaded.definition, loaded.handling);
        let profile = default_profile_for_gui(session.committed_handling());
        let controls = EngineControls::default();
        let engine = SingleCylinderEngine::from_definition_with_profile(
            session.committed_engine().clone(),
            profile,
        );
        let telemetry = TelemetryAggregator::new(session.committed_engine().clone(), controls);
        let latest_frame = telemetry.snapshot();

        Self {
            engine_path,
            session,
            profile,
            engine,
            controls,
            telemetry,
            latest_frame,
            running: true,
            redline_cut_remaining_seconds: 0.0,
            last_wall_clock: Instant::now(),
            load_status,
            angle_mode: AngleMode::Absolute,
            diverged: None,
        }
    }

    fn reset_engine(&mut self) {
        self.engine = SingleCylinderEngine::from_definition_with_profile(
            self.session.committed_engine().clone(),
            self.profile,
        );
        self.telemetry =
            TelemetryAggregator::new(self.session.committed_engine().clone(), self.controls);
        self.latest_frame = self.telemetry.snapshot();
        self.redline_cut_remaining_seconds = 0.0;
        self.last_wall_clock = Instant::now();
        // A fresh engine replaces any poisoned state from a previous divergence.
        self.diverged = None;
    }

    fn profile_for_committed(&self) -> SimulationProfile {
        default_profile_for_gui(self.session.committed_handling())
    }

    fn write_changes(&mut self) {
        self.session.write_changes();
        self.profile = self.profile_for_committed();
        self.reset_engine();
    }

    /// Persist the committed config back to the JSON files on disk. Saves the
    /// committed (applied) config, so use Write Changes first to include any
    /// pending draft edits.
    fn save_to_disk(&mut self) {
        match save_engine_and_handling(
            &self.engine_path,
            self.session.committed_engine(),
            self.session.committed_handling(),
        ) {
            Ok(()) => {
                self.load_status = format!("saved to {}", self.engine_path.display());
            }
            Err(err) => {
                self.load_status = format!("save failed: {err}");
            }
        }
    }

    fn reload_from_disk(&mut self) {
        let loaded = load_engine_and_handling(
            &self.engine_path,
            self.session.committed_engine(),
            self.session.committed_handling(),
        );
        self.load_status = format!(
            "{} | {}",
            describe_source("engine", &loaded.engine_source),
            describe_source("handling", &loaded.handling_source)
        );
        self.session.replace(loaded.definition, loaded.handling);
        self.profile = self.profile_for_committed();
        self.reset_engine();
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
        let steps = (wall_delta_seconds / timestep_seconds).ceil().max(1.0) as usize;

        self.telemetry.set_controls(self.controls);
        // The 1D solver asserts on non-physical states (e.g. a non-positive
        // pipe pressure) that an aggressive parameter change can briefly
        // produce. Catch that instead of taking the whole GUI down: pause and
        // surface it. The engine is left poisoned, so we must not step it again
        // until the next reset (Resume re-runs from the committed config).
        let outcome = catch_unwind(AssertUnwindSafe(|| self.run_steps(steps, timestep_seconds)));
        if outcome.is_err() {
            self.running = false;
            self.diverged = Some(
                "Simulation diverged (a pipe pressure went non-positive). Paused — \
                 Undo/adjust parameters and Write Changes, Reload from disk, or Resume."
                    .to_string(),
            );
        }
    }

    fn run_steps(&mut self, steps: usize, timestep_seconds: f64) {
        for _ in 0..steps {
            let current_rpm = rad_per_s_to_rpm(self.engine.crank_speed_rad_per_s());
            if current_rpm >= self.controls.redline_spark_cut_rpm {
                self.redline_cut_remaining_seconds = self
                    .session
                    .committed_handling()
                    .redline_cut_time_seconds
                    .max(0.0);
            }
            let redline_cut_active = self.redline_cut_remaining_seconds > 0.0;
            let step_inputs = self.controls.to_step_inputs_with_redline_cut_active(
                self.session.committed_engine(),
                current_rpm,
                redline_cut_active,
            );
            let output = self.engine.step(step_inputs);
            self.telemetry.ingest_step_output(output, timestep_seconds);
            self.redline_cut_remaining_seconds =
                (self.redline_cut_remaining_seconds - timestep_seconds).max(0.0);
            if let Some(frame) = self.telemetry.publish_ready() {
                self.latest_frame = frame;
            }
        }
    }

    // -- Operating controls (live) -------------------------------------------

    fn operating_controls(&mut self, ui: &mut egui::Ui) {
        ui.heading("Operating");
        ui.checkbox(&mut self.running, "Run");
        if ui.button("Reset Sim").clicked() {
            self.reset_engine();
        }
        ui.separator();

        // Throttle slider + input box (default WOT).
        ui.label("Throttle (default WOT)");
        ui.horizontal(|ui| {
            ui.add(
                egui::Slider::new(&mut self.controls.throttle_position, 0.0..=1.0)
                    .show_value(false),
            );
            ui.add(
                DragValue::new(&mut self.controls.throttle_position)
                    .speed(0.01)
                    .range(0.0..=1.0)
                    .fixed_decimals(2),
            );
        });

        // RPM entry box. The sim clamps to >= 0 (no reverse rotation), so the
        // box is non-negative; entering a value drives the dyno target.
        ui.horizontal(|ui| {
            ui.label("RPM");
            if ui
                .add(
                    DragValue::new(&mut self.controls.dyno_target_rpm)
                        .speed(10.0)
                        .range(0.0..=12_000.0)
                        .suffix(" rpm"),
                )
                .changed()
            {
                self.controls.dyno_mode_enabled = true;
            }
        });
        ui.checkbox(&mut self.controls.dyno_mode_enabled, "Dyno (force RPM)");

        ui.add(egui::Slider::new(&mut self.controls.lambda_target, 0.1..=2.0).text("Lambda"));

        ui.separator();
        // Direct MAP override.
        ui.checkbox(&mut self.controls.map_override_enabled, "MAP override");
        let mut map_kpa = self.controls.map_override_pa / 1000.0;
        if ui
            .add_enabled(
                self.controls.map_override_enabled,
                DragValue::new(&mut map_kpa)
                    .speed(1.0)
                    .range(0.1..=MAP_OVERRIDE_MAX_PA / 1000.0)
                    .suffix(" kPa"),
            )
            .changed()
        {
            self.controls.map_override_pa = map_kpa * 1000.0;
        }

        ui.separator();
        ui.checkbox(&mut self.controls.spark_enabled, "Spark");
        ui.checkbox(&mut self.controls.fuel_enabled, "Fuel");
        ui.checkbox(&mut self.controls.starter_enabled, "Starter");
        ui.add(
            egui::Slider::new(&mut self.controls.starter_torque_nm, 0.0..=120.0)
                .text("Starter Torque")
                .suffix(" Nm"),
        );
        ui.add(
            egui::Slider::new(&mut self.controls.added_torque_load_nm, 0.0..=40.0)
                .text("Load")
                .suffix(" Nm"),
        );

        self.set_point_controls(ui);
    }

    fn set_point_controls(&mut self, ui: &mut egui::Ui) {
        let set_points = self.session.committed_handling().set_points_rpm.clone();
        if set_points.is_empty() {
            return;
        }
        ui.separator();
        ui.label("Set Points (snap)");
        let nearest = nearest_set_point_rpm(self.controls.dyno_target_rpm, &set_points);
        let mut index = set_points
            .iter()
            .position(|&rpm| rpm == nearest)
            .unwrap_or(0);
        let max_index = set_points.len() - 1;
        let formatter_points = set_points.clone();
        let response = ui.add(
            egui::Slider::new(&mut index, 0..=max_index)
                .integer()
                .custom_formatter(move |value, _| {
                    let i = (value as usize).min(formatter_points.len() - 1);
                    format!("{:.0} rpm", formatter_points[i])
                }),
        );
        if response.changed() {
            self.controls.dyno_target_rpm = set_points[index];
            self.controls.dyno_mode_enabled = true;
        }
        ui.horizontal(|ui| {
            for &rpm in &set_points {
                if ui.button(format!("{rpm:.0}")).clicked() {
                    self.controls.dyno_target_rpm = rpm;
                    self.controls.dyno_mode_enabled = true;
                }
            }
        });
    }

    // -- Tuning controls (draft, applied with Write Changes) -----------------

    fn tuning_controls(&mut self, ui: &mut egui::Ui) {
        ui.heading("Tuning");
        ui.small(self.load_status.clone());
        ui.horizontal(|ui| {
            let dirty = self.session.is_dirty();
            if ui
                .add_enabled(dirty, egui::Button::new("Write Changes"))
                .clicked()
            {
                self.write_changes();
            }
            if ui
                .add_enabled(dirty, egui::Button::new("Undo Changes"))
                .clicked()
            {
                self.session.undo_changes();
            }
            if ui.button("Reload from disk").clicked() {
                self.reload_from_disk();
            }
            if ui
                .button("Save to disk")
                .on_hover_text(
                    "Writes the committed config; Write Changes first to include pending edits",
                )
                .clicked()
            {
                self.save_to_disk();
            }
        });

        let angle_mode = self.angle_mode;
        let draft = &mut self.session.draft_engine;

        egui::CollapsingHeader::new("Geometry")
            .default_open(true)
            .show(ui, |ui| {
                ui.add(
                    egui::Slider::new(&mut draft.geometry.compression_ratio, 6.0..=16.0)
                        .text("Compression Ratio"),
                );
            });

        egui::CollapsingHeader::new("Runners")
            .default_open(true)
            .show(ui, |ui| {
                runner_controls(ui, "Intake Runner", &mut draft.intake_exhaust.intake_runner);
                ui.separator();
                runner_controls(
                    ui,
                    "Exhaust Runner",
                    &mut draft.intake_exhaust.exhaust_runner,
                );
            });

        egui::CollapsingHeader::new("Intake / Throttle")
            .default_open(false)
            .show(ui, |ui| {
                diameter_input(
                    ui,
                    "Throttle body",
                    &mut draft.intake_exhaust.throttle_maximum_area_m2,
                );
                let mut idle_area = draft
                    .intake_exhaust
                    .idle_throttle_maximum_area_m2
                    .unwrap_or(draft.intake_exhaust.throttle_maximum_area_m2 * 0.1);
                if diameter_input(ui, "Idle throttle", &mut idle_area) {
                    draft.intake_exhaust.idle_throttle_maximum_area_m2 = Some(idle_area);
                }
                let mut plenum_cc = draft.intake_exhaust.intake_plenum_volume_m3 * 1.0e6;
                ui.horizontal(|ui| {
                    ui.label("Plenum volume");
                    if ui
                        .add(
                            DragValue::new(&mut plenum_cc)
                                .speed(1.0)
                                .range(1.0..=10_000.0)
                                .suffix(" cc"),
                        )
                        .changed()
                    {
                        draft.intake_exhaust.intake_plenum_volume_m3 = plenum_cc / 1.0e6;
                    }
                });
            });

        egui::CollapsingHeader::new("Valve Timing")
            .default_open(false)
            .show(ui, |ui| {
                valve_controls(ui, "Intake", &mut draft.valves.intake);
                ui.separator();
                valve_controls(ui, "Exhaust", &mut draft.valves.exhaust);
            });

        egui::CollapsingHeader::new("Combustion")
            .default_open(false)
            .show(ui, |ui| {
                ui.checkbox(&mut draft.combustion.enabled, "Combustion enabled");
                labelled_drag(
                    ui,
                    "Default lambda",
                    &mut draft.combustion.lambda_target,
                    0.001,
                    0.1..=2.0,
                    "",
                );
                labelled_drag(
                    ui,
                    "Combustion eff.",
                    &mut draft.combustion.combustion_efficiency,
                    0.001,
                    0.0..=1.0,
                    "",
                );
                labelled_drag(
                    ui,
                    "Heat loss frac",
                    &mut draft.combustion.heat_loss_fraction,
                    0.001,
                    0.0..=1.0,
                    "",
                );
                let mut fuel_mg = draft.combustion.fuel_mass_per_cycle_kg * 1.0e6;
                if labelled_drag(ui, "Fuel/cycle", &mut fuel_mg, 0.1, 0.0..=200.0, " mg") {
                    draft.combustion.fuel_mass_per_cycle_kg = fuel_mg / 1.0e6;
                }
                let mut lhv_mj = draft.combustion.fuel_lower_heating_value_j_per_kg / 1.0e6;
                if labelled_drag(ui, "Fuel LHV", &mut lhv_mj, 0.1, 10.0..=60.0, " MJ/kg") {
                    draft.combustion.fuel_lower_heating_value_j_per_kg = lhv_mj * 1.0e6;
                }
                labelled_drag(
                    ui,
                    "Ignition delay",
                    &mut draft.combustion.ignition_delay_deg,
                    0.1,
                    0.0..=60.0,
                    " deg",
                );
            });

        egui::CollapsingHeader::new("Wiebe")
            .default_open(false)
            .show(ui, |ui| {
                let mut duration_deg = draft.combustion.wiebe.combustion_duration_rad.to_degrees();
                if labelled_drag(
                    ui,
                    "Burn duration",
                    &mut duration_deg,
                    0.5,
                    5.0..=180.0,
                    " deg",
                ) {
                    draft.combustion.wiebe.combustion_duration_rad = duration_deg.to_radians();
                }
                labelled_drag(
                    ui,
                    "Efficiency coeff (a)",
                    &mut draft.combustion.wiebe.efficiency_coefficient,
                    0.05,
                    1.0..=10.0,
                    "",
                );
                labelled_drag(
                    ui,
                    "Shape factor (m)",
                    &mut draft.combustion.wiebe.shape_factor,
                    0.05,
                    0.5..=5.0,
                    "",
                );
            });

        egui::CollapsingHeader::new("Spark")
            .default_open(false)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Angle entry:");
                    ui.selectable_value(&mut self.angle_mode, AngleMode::Absolute, "Absolute");
                    ui.selectable_value(&mut self.angle_mode, AngleMode::Btdc, "BTDC");
                });
                let spark = &mut draft.combustion.spark_timing;
                labelled_drag(
                    ui,
                    "Low-speed RPM",
                    &mut spark.low_speed_rpm,
                    10.0,
                    0.0..=12_000.0,
                    " rpm",
                );
                advance_input(
                    ui,
                    "Low-speed advance",
                    &mut spark.low_speed_advance_deg_btdc,
                    angle_mode,
                );
                labelled_drag(
                    ui,
                    "High-speed RPM",
                    &mut spark.high_speed_rpm,
                    10.0,
                    0.0..=12_000.0,
                    " rpm",
                );
                advance_input(
                    ui,
                    "High-speed advance",
                    &mut spark.high_speed_advance_deg_btdc,
                    angle_mode,
                );
            });

        // Kept at the very bottom so it appearing/disappearing as you edit does
        // not push the input boxes around mid-typing.
        ui.separator();
        if self.session.is_dirty() {
            ui.colored_label(
                Color32::YELLOW,
                "Pending changes — Write Changes applies them and resets the sim.",
            );
        } else {
            ui.small("No pending changes.");
        }
    }

    // -- Data displays -------------------------------------------------------

    fn data_boxes(&self, ui: &mut egui::Ui) {
        let data = &self.latest_frame.engine_data;
        egui::ScrollArea::horizontal()
            .id_salt("data_boxes_scroll")
            .auto_shrink([false, true])
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    data_box(ui, "Torque", format!("{:.2} Nm", data.torque_nm), true);
                    data_box(ui, "Power", format!("{:.2} kW", data.power_kw), true);
                    data_box(
                        ui,
                        "VE",
                        format!("{:.0} %", data.volumetric_efficiency_percent),
                        true,
                    );
                    scalar_box(ui, "Exhaust Lambda", data.exhaust_lambda, true);
                    data_box(ui, "MAP", format!("{:.0} kPa", data.map_pa / 1000.0), true);
                    data_box(ui, "BMEP Est", format!("{:.0} kPa", data.bmep_kpa), false);
                    scalar_box(ui, "Chamber Lambda", data.chamber_lambda, false);
                    data_box(
                        ui,
                        "Peak Temp",
                        format!("{:.0} C", data.peak_cylinder_temperature_k - 273.15),
                        false,
                    );
                    data_box(
                        ui,
                        "Peak Pressure",
                        format!(
                            "{:.0} kPa rel",
                            (data.peak_cylinder_pressure_pa - data.ambient_pressure_pa) / 1000.0
                        ),
                        false,
                    );
                    data_box(
                        ui,
                        "Combustion Ratio",
                        format!("{:.2}", data.combustion_event_ratio),
                        false,
                    );
                    data_box(ui, "RPM", format!("{:.0}", data.rpm), false);
                });
            });
    }

    fn angle_plots(&self, ui: &mut egui::Ui) {
        let telemetry = &self.latest_frame;
        egui::ScrollArea::vertical()
            .id_salt("angle_plots_scroll")
            .auto_shrink([false, false])
            .show(ui, |ui| {
                angle_plot(
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
                );
                angle_plot(
                    ui,
                    "Cylinder Temperature",
                    "Temperature (C)",
                    &[(
                        "Temperature",
                        &celsius_points(&telemetry.angle_trace.temperature_k),
                        Color32::LIGHT_RED,
                    )],
                );
                angle_plot(
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
                );
            });
    }
}

impl eframe::App for TuningGuiApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.simulate_until_now();

        egui::SidePanel::left("operating")
            .resizable(true)
            .default_width(250.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical()
                    .id_salt("operating_scroll")
                    .show(ui, |ui| {
                        self.operating_controls(ui);
                    });
            });

        egui::SidePanel::right("tuning")
            .resizable(true)
            .default_width(320.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical()
                    .id_salt("tuning_scroll")
                    .show(ui, |ui| {
                        self.tuning_controls(ui);
                    });
            });

        egui::TopBottomPanel::top("data")
            .resizable(false)
            .exact_height(56.0)
            .show(ctx, |ui| {
                self.data_boxes(ui);
            });

        let mut resume = false;
        egui::TopBottomPanel::bottom("status")
            .resizable(false)
            .show(ctx, |ui| match &self.diverged {
                Some(message) => {
                    ui.horizontal(|ui| {
                        ui.colored_label(Color32::from_rgb(255, 90, 90), message);
                        resume = ui.button("Resume").clicked();
                    });
                }
                None => {
                    ui.small("Sim OK");
                }
            });
        if resume {
            // Retry from the committed config. If it is genuinely unstable this
            // will simply diverge again; Undo/Reload give a known-good state.
            self.reset_engine();
            self.running = true;
        }

        egui::CentralPanel::default().show(ctx, |ui| {
            self.angle_plots(ui);
        });

        ctx.request_repaint();
    }
}

// ---------------------------------------------------------------------------
// Tuning input widgets
// ---------------------------------------------------------------------------

fn runner_controls(ui: &mut egui::Ui, label: &str, runner: &mut PipeDefinition) {
    ui.label(RichText::new(label).strong());
    ui.add(
        egui::Slider::new(&mut runner.total_length_m, 0.01..=2.5)
            .text("Length")
            .suffix(" m"),
    );
    ui.add(
        egui::Slider::new(&mut runner.number_of_cells, 1..=200)
            .text("Cells")
            .integer(),
    );
    diameter_input(ui, "Diameter", &mut runner.area_m2);
    ui.small(format!(
        "cell length {:.4} m, area {:.2e} m^2",
        runner.cell_length_m(),
        runner.area_m2
    ));
}

/// Edit a flow area as a circular diameter in mm. Returns true on change.
fn diameter_input(ui: &mut egui::Ui, label: &str, area_m2: &mut f64) -> bool {
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.label(label);
        let mut diameter_mm = diameter_mm_from_area_m2(*area_m2);
        if ui
            .add(
                DragValue::new(&mut diameter_mm)
                    .speed(0.1)
                    .range(1.0..=120.0)
                    .suffix(" mm"),
            )
            .changed()
        {
            *area_m2 = area_m2_from_diameter_mm(diameter_mm);
            changed = true;
        }
    });
    changed
}

fn valve_controls(ui: &mut egui::Ui, label: &str, valve: &mut ValveDefinition) {
    ui.label(RichText::new(label).strong());
    // Start time (absolute engine position, 0..720).
    ui.horizontal(|ui| {
        ui.label("Start (abs)");
        let mut start = valve.open_angle_deg;
        let mut duration =
            (valve.close_angle_deg - valve.open_angle_deg).rem_euclid(ENGINE_CYCLE_DEG);
        if ui
            .add(
                DragValue::new(&mut start)
                    .speed(1.0)
                    .range(0.0..=ENGINE_CYCLE_DEG)
                    .suffix(" deg"),
            )
            .changed()
        {
            valve.open_angle_deg = start;
            // Keep duration constant when the start moves.
            valve.close_angle_deg = (start + duration).rem_euclid(ENGINE_CYCLE_DEG);
        }
        ui.label("Duration");
        if ui
            .add(
                DragValue::new(&mut duration)
                    .speed(1.0)
                    .range(0.0..=ENGINE_CYCLE_DEG)
                    .suffix(" deg"),
            )
            .changed()
        {
            valve.close_angle_deg = (valve.open_angle_deg + duration).rem_euclid(ENGINE_CYCLE_DEG);
        }
    });
}

/// Edit a spark advance, honouring the absolute/BTDC angle-entry toggle. The
/// stored value is always degrees BTDC.
fn advance_input(ui: &mut egui::Ui, label: &str, advance_btdc: &mut f64, mode: AngleMode) {
    ui.horizontal(|ui| {
        ui.label(label);
        match mode {
            AngleMode::Btdc => {
                ui.add(
                    DragValue::new(advance_btdc)
                        .speed(0.5)
                        .range(0.0..=ENGINE_CYCLE_DEG)
                        .suffix(" deg BTDC"),
                );
            }
            AngleMode::Absolute => {
                let mut absolute = (ENGINE_CYCLE_DEG - *advance_btdc).rem_euclid(ENGINE_CYCLE_DEG);
                if ui
                    .add(
                        DragValue::new(&mut absolute)
                            .speed(0.5)
                            .range(0.0..=ENGINE_CYCLE_DEG)
                            .suffix(" deg abs"),
                    )
                    .changed()
                {
                    *advance_btdc = (ENGINE_CYCLE_DEG - absolute).rem_euclid(ENGINE_CYCLE_DEG);
                }
            }
        }
    });
}

fn labelled_drag(
    ui: &mut egui::Ui,
    label: &str,
    value: &mut f64,
    speed: f64,
    range: std::ops::RangeInclusive<f64>,
    suffix: &str,
) -> bool {
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.label(label);
        changed = ui
            .add(
                DragValue::new(value)
                    .speed(speed)
                    .range(range)
                    .suffix(suffix.to_string()),
            )
            .changed();
    });
    changed
}

// ---------------------------------------------------------------------------
// Data displays
// ---------------------------------------------------------------------------

fn data_box(ui: &mut egui::Ui, label: &str, value: String, prominent: bool) {
    let width = if prominent { 168.0 } else { 140.0 };
    ui.scope(|ui| {
        ui.set_width(width);
        ui.group(|ui| {
            ui.set_min_size(egui::vec2(width - 16.0, 40.0));
            let mut label_text = RichText::new(label).small();
            if prominent {
                label_text = label_text.color(Color32::LIGHT_YELLOW);
            }
            ui.label(label_text);
            let value_text = if prominent {
                RichText::new(value).strong().size(18.0)
            } else {
                RichText::new(value).strong()
            };
            ui.label(value_text);
        });
    });
}

fn scalar_box(ui: &mut egui::Ui, label: &str, scalar: TelemetryScalar, prominent: bool) {
    let suffix = match scalar.availability {
        TelemetryAvailability::Measured => "",
        TelemetryAvailability::Placeholder => " (est)",
        TelemetryAvailability::Unavailable => " (n/a)",
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
    data_box(ui, label, text, prominent);
}

fn angle_plot(
    ui: &mut egui::Ui,
    title: &str,
    y_label: &str,
    series: &[(&str, &Vec<[f64; 2]>, Color32)],
) {
    ui.label(RichText::new(title).strong());
    Plot::new(format!("{title}_plot"))
        .height(plot_height(ui.available_height()))
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

fn plot_height(available_height: f32) -> f32 {
    (available_height / 3.0 - 28.0).clamp(140.0, 320.0)
}

fn relative_pressure_points(points: &[[f64; 2]], ambient_pressure_pa: f64) -> Vec<[f64; 2]> {
    points
        .iter()
        .map(|point| [point[0], (point[1] - ambient_pressure_pa) / 1000.0])
        .collect()
}

fn celsius_points(points: &[[f64; 2]]) -> Vec<[f64; 2]> {
    points
        .iter()
        .map(|point| [point[0], point[1] - 273.15])
        .collect()
}
